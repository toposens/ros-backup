/** @file     serial.h
 *  @author   Adi Singh, Sebastian Dengler
 *  @date     January 2019
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>
#include <sstream>

namespace toposens_driver
{

/** @brief  Provides low-level I/O access to TS data stream using
 *  native Unix API.
 *  @details  Sets up a UART bridge to access raw data packets
 *  from a TS device. Maintains simple read-write access at the
 *  given baud rate. Methods defined here are independent of ROS.
 */
class Serial
{
  public:
    /** Opens a persistent serial connection at the given port.
    *  @param port Device endpoint usually of the form /dev/ttyUSB*.
    */
    Serial(std::string port);
    ~Serial();

    /** Checks whether or not serial connection is still open.
     *  @returns True if connection is open, false otherwise.
     */
    bool isAlive();

    /** Extracts a single TS data frame into the given iostream.
      * @param data Points to a string stream expecting a sensor frame.
      */
    void getFrame(std::stringstream &data);

    /** Writes the given bytes to the serial stream.
      * Usually used for updating sensor settings during runtime. 
      * @param bytes Data in a TS-specific command format (Cxxxxx00000).
      * @returns True if data was transmitted without error.
      * No sensor handshake is received to confirm settings update.
      */
    bool send(char* bytes);

  private:
    int _fd;    /**< Linux file descriptor pointing to TS device port.*/
    std::string _port;  /**< Stored device port for future access.*/
    const unsigned int kBaud = B921600; /**< Baud rate needed for TS device comms.*/

};

} // namespace toposens_driver

#endif // SERIAL_H