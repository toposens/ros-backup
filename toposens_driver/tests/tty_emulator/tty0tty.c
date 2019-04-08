// Forked from @freemed tty0tty project
// Modified for accurate emulation of a TS device


/* ########################################################################

   tty0tty - linux null modem emulator 

   ########################################################################

   Copyright (c) : 2013  Luis Claudio Gambôa Lopes and Maximiliano Pin max.pin@bitroit.com

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

   For e-mail suggestions :  lcgamboa@yahoo.com
   ######################################################################## */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <errno.h>

#include <termio.h>

#define name_sz 64
#define buff_sz 1024

static char buffer[buff_sz];
static const int baud = B921600;


int tty0tty_open(char *pts_name, char *pts_name_s , int pts_namesz)
{
  char* ptr;
  int   fdm;

  strncpy(pts_name, "/dev/ptmx", pts_namesz);
  pts_name[pts_namesz - 1] = '\0';

  fdm = posix_openpt(O_RDWR | O_NONBLOCK);
  if (fdm < 0) return(-1);

  if (grantpt(fdm) < 0) {
      close(fdm);
      return(-2);
  }

  if (unlockpt(fdm) < 0) {
      close(fdm);
      return(-3);
  }

  if ((ptr = ptsname(fdm)) == NULL) {
      close(fdm);
      return(-4);
  }
  
  strncpy(pts_name_s, ptr, pts_namesz);
  pts_name[pts_namesz - 1] = '\0';

  return fdm;        
}


int tty0tty_conf(int serialDev)
{
  int rc;
  struct termios params;

  // Get terminal atributes
  rc = tcgetattr(serialDev, &params);

  // Modify terminal attributes
  cfmakeraw(&params);

  rc = cfsetispeed(&params, baud);
  rc = cfsetospeed(&params, baud);

  // CREAD - Enable port to read data
  // CLOCAL - Ignore modem control lines
  params.c_cflag |= (baud |CS8 | CLOCAL | CREAD);

  // Set serial attributes
  rc = tcsetattr(serialDev, TCSANOW, &params);

  // Flush serial device of both non-transmitted
  // output data and non-read input data
  tcflush(serialDev, TCIOFLUSH);
  return EXIT_SUCCESS;
}


void tty0tty_copy(int fdfrom, int fdto)
{
  ssize_t br, bw;
  char *pbuf = buffer;
  br = read(fdfrom, buffer, buff_sz);
  if (br < 0) {
    if (errno == EAGAIN || errno == EIO) {
      br = 0;
    } else {
      perror("read");
      exit(1);
    }
  }

  if (br > 0) {
    do {
      do {
        bw = write(fdto, pbuf, br);
        if (bw > 0) {
          pbuf += bw;
          br -= bw;
        }
      } while (br > 0 && bw > 0);
    } while (bw < 0 && errno == EAGAIN);
    
    if (bw <= 0) {
      // kernel buffer may be full, but we can recover
      fprintf(stderr, "Write error, br=%d bw=%d\n", (int) br, (int) bw);
      usleep(500000);
      // discard input
      while (read(fdfrom, buffer, buff_sz) > 0)
        ;
    }
  } else usleep(100000);
}


int main(int argc, char* argv[])
{
  char master_1[name_sz];
  char slave_1[name_sz];
  char master_2[name_sz];
  char slave_2[name_sz];

  int fd1;
  int fd2;

  fd_set rfds;
  int retval;

  fd1 = tty0tty_open(master_1, slave_1, name_sz);
  fd2 = tty0tty_open(master_2, slave_2, name_sz);

  if (argc >= 3) {
    unlink(argv[1]);
    unlink(argv[2]);

    if (symlink(slave_1, argv[1]) < 0) {
      fprintf(stderr, "Cannot create: %s\n", argv[1]);
      return 1;
    }

    if (symlink(slave_2, argv[2]) < 0) {
      fprintf(stderr, "Cannot create: %s\n", argv[2]);
      return 1;
    }
    printf("(%s) <=> (%s)\n", argv[1], argv[2]);
  } else printf("(%s) <=> (%s)\n", slave_1, slave_2);

  tty0tty_conf(fd1);
  tty0tty_conf(fd2);

  while(1) {
    FD_ZERO(&rfds);
    FD_SET(fd1, &rfds);
    FD_SET(fd2, &rfds);

    retval = select(fd2 + 1, &rfds, NULL, NULL, NULL);
    if (retval == -1) {
      perror("select");
      return 1;
    }

    if (FD_ISSET(fd1, &rfds)) tty0tty_copy(fd1, fd2);
    if (FD_ISSET(fd2, &rfds)) tty0tty_copy(fd2, fd1);
  }

  close(fd1);
  close(fd2);
  return EXIT_SUCCESS;
}
