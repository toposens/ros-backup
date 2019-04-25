/** @file     sensor_test.cpp
 *  @author   Adi Singh, Christopher Lang, Roua Mokchah
 *  @date     March 2019
 */

#include <gtest/gtest.h>
#include <toposens_driver/sensor.h>
#include <fcntl.h>
#include <fstream>

using namespace toposens_driver;

class SensorTest : public ::testing::Test
{
  protected:
    Sensor* dev;
    toposens_msgs::TsScan scan;
    ros::Subscriber sub;
    ros::NodeHandle* private_nh;
    std::string curr_frame;

    void SetUp()
    {
      ros::NodeHandle nh;
      private_nh = new ros::NodeHandle("~");
      sub = nh.subscribe(kScansTopic, kQueueSize, &SensorTest::store, this);

      dev = new Sensor(nh, *private_nh);
      scan.header.stamp = ros::Time::now();
      scan.header.frame_id = "toposens";
      scan.points.clear();
    }

    void TearDown()
    {
      scan.points.clear();
      dev->shutdown();
      delete dev;
      delete private_nh;
    }

    void store(const toposens_msgs::TsScan::ConstPtr& msg)
    {
      scan = *msg;
    }

  //@todo by testing poll, there's a bunch of duplicity introduced with serial
    // tests. Remove duplicate testing.
    void pollMockTx(std::string frame)
    {
      curr_frame = frame;
      std::string mock_port;
      private_nh->getParam("mock_port", mock_port);

      write(
        open(mock_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY),
        frame.c_str(), frame.size()
      );

      dev->poll();
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }


    // Test attributes of point against expected x, y, z, v values
    void EXPECT_POINT(auto &pt, std::vector<float> exp, std::string caseMsg)
    {
      EXPECT_FLOAT_EQ(exp[0], pt.location.x) << "Failed for case: " << caseMsg;
      EXPECT_FLOAT_EQ(exp[1], pt.location.y) << "Failed for case: " << caseMsg;
      EXPECT_FLOAT_EQ(exp[2], pt.location.z) << "Failed for case: " << caseMsg;
      EXPECT_FLOAT_EQ(exp[3], pt.intensity)  << "Failed for case: " << caseMsg;
    }


    void EXPECT_EMPTY(std::string caseMsg)
    {
      EXPECT_FILLED(caseMsg, 0);
    }


    // Clear scan anyway so failure conditions are not propograted further
    void EXPECT_FILLED(std::string caseMsg, uint num_exp)
    {
      auto p = scan.points;
      if (p.size() == num_exp) {
        while (num_exp > 0) {
          EXPECT_POINT(p[--num_exp], {-0.415, 0.01, 0.257, 0.61}, caseMsg);
        }
      } else {
        ADD_FAILURE() << "Invalid scan for case: " << caseMsg << "\r" << curr_frame;
      }
      scan.points.clear();
    }
};

/**
 * Testing Sensor::poll with valid data frames from dump file.
 */
TEST_F(SensorTest, ReadFromDumpFile)
{
  std::string dumpfilename;
  private_nh->getParam("file", dumpfilename);

  std::ifstream infile;
  infile.open(dumpfilename);

  std::string line;

  /** read each single scan from the dump file : Example
  S000020P0000X-0104Y-0229Z00206V00036ES000025P0000X-0156Y-0229Z00206V00048E
  line 1 : empty -> continue
  line 2 : 000020P0000X-0104Y-0229Z00206V00036E
  line 3 : 000025P0000X-0156Y-0229Z00206V00048E */

  while (std::getline(infile,line,'S')) {

    if (line.empty()) continue;

    testing::internal::CaptureStderr();
    EXPECT_NO_THROW(pollMockTx(line));
    if(!testing::internal::GetCapturedStderr().empty()) {
      ADD_FAILURE() << "Unintended behavior with dumped data frame" ;
    }
  }
  infile.close();
}

/**
 * Testing Sensor::poll with a valid data frame
 * Data frame consisting of 4 known points
 */

TEST_F(SensorTest, pollValidFrame)
{
  std::string frame = "S000016"
                        "P0000X-0415Y00010Z00257V00061"
                        "P0000X-0235Y00019Z00718V00055"
                        "P0000X-0507Y00043Z00727V00075"
                        "P0000X00142Y00360Z01555V00052"
                      "E";
  pollMockTx(frame);

  EXPECT_EQ(scan.points.size(), (uint)4);
  EXPECT_POINT(scan.points[0], {-0.415, 0.01,  0.257, 0.61}, "Valid point P1");
  EXPECT_POINT(scan.points[1], {-0.235, 0.019, 0.718, 0.55}, "Valid point P2");
  EXPECT_POINT(scan.points[2], {-0.507, 0.043, 0.727, 0.75}, "Valid point P3");
  EXPECT_POINT(scan.points[3], { 0.142, 0.360, 1.555, 0.52}, "Valid point P4");
}

/** Testing Sensor::poll with empty data frames
*  Desired behavior: extract no points from empty data frame.
*  No error is thrown, the frame should simply be ignored.
**/
TEST_F(SensorTest, pollEmptyFrames)
{
  pollMockTx("S000000E");
  EXPECT_EMPTY("Valid empty frame 1");

  pollMockTx("S153698E");
  EXPECT_EMPTY("Valid empty frame 2");

  pollMockTx("");
  EXPECT_EMPTY("Invalid empty frame 1");

  pollMockTx("SE");
  EXPECT_EMPTY("Invalid empty frame 2");

  pollMockTx("SXYZV");
  EXPECT_EMPTY("Invalid empty frame 3");
}


/**
 * Test for detecting invalid data frames.
 * Desired behavior: Detect and discard invalid data frames.
 */
TEST_F(SensorTest, pollInvalidPoints)
{
  pollMockTx("S000016P0000X-0415Y00010Z00257E");
  EXPECT_EMPTY("Missing intensity value");

  pollMockTx("S000016P0000X-0415Y00010Z00257V061E");
  EXPECT_EMPTY("Intensity with <5 bytes");

  pollMockTx("S000016P0000X-0415Y00010V00061E");
  EXPECT_EMPTY("Missing Z coordinate");

  pollMockTx("S000016P0000-0415Y00010Z00257V00061E");
  EXPECT_EMPTY("Missing X tag");

  pollMockTx("S000016P0000X-0415Y00010Y00257V00061E");
  EXPECT_EMPTY("Double Y coordinate");

  pollMockTx("S000016P0000-04150001000257V00061E");
  EXPECT_EMPTY("Missing XYZ tags");

  pollMockTx("S000016P0000X-04E5Y00010Z00257V00061E");
  EXPECT_EMPTY("Characters in X coordinate");
}


/** note: adjacent strings are auto concatenated by compiler
  * Test for handling invalid data frames.
  * Desired behavior: faulty data point(s) are discarded, valid data points are kept.
  */
TEST_F(SensorTest, pollTwoMixedPoints)
{
  pollMockTx("S000016"
                "P0000X-0415Y00010Z00257V00061"
                "P0000X-0235Y00019Z00718"
              "E");
  EXPECT_FILLED("Double Y coordinate", 1);


  pollMockTx("S000016"
                "P0000X-0415Y00010Z00257V00061"
                "P0000X-0235Y00019Z00718"
              "E");
  EXPECT_FILLED("Missing intensity value in P2", 1);


  pollMockTx("S000016"
                "P0000X-0415Y00010Z00257V00061"
                "P0000X-0235Y00019ZV00055"
              "E");
  EXPECT_FILLED("Missing Z coordinate in P2", 1);


  pollMockTx("S000016"
                "P0000-0235Y00019Z00718V00055"
                "P0000-0415Y00010Z00257V00061"
              "E");
  EXPECT_EMPTY("Missing X tag in both P1 and P2");


  pollMockTx("S000016"
                "P0000X-0415Y00010Z00257V00061"
                "PV00055"
              "E");
  EXPECT_FILLED("XYZ tags missing in P2", 1);
}


TEST_F(SensorTest, pollThreeMixedPoints)
{
  pollMockTx("S000016"
                "P0000X-0415Y00010Z00257V00061"
                "P0000X-0415Y00010Z00257V00061"
                "P0000X-0235Y00019Z0071800055"
              "E");
  EXPECT_FILLED("Missing intensity tag in P3", 2);


  pollMockTx("S000016"
                "P0000X-0235Y00019Y00718V00055"
                "P0000X-0415Y00010Z00257V00061"
                "P0000X-0415Y00010Z00257Y00061"
              "E");
  EXPECT_FILLED("Double Y coordinate in P1 and P3", 1);


  pollMockTx("S000016"
                "P0000X-0415Y00010Z00257V00061"
                "P0000X-0Z35Y00019Z00718V00055"
                "P0000X-0415Y00010Z00257V00061"
              "E");
  EXPECT_FILLED("Non-numerical characters in X coordinate of P2", 2);
}

/**
 * Testing Sensor::poll with large data frame.
 * Desired behavior: Method does not raise any exception, even for large point clouds.
 */
TEST_F(SensorTest, pollLargeFrame)
{
  uint numPoints = 500;
  std::string frame = "S000016";
  while (numPoints--) frame += "P0000X-0415Y00010Z00257V00061";
  frame += "E";

  testing::internal::CaptureStderr();
  EXPECT_NO_THROW(pollMockTx(frame));
  if(!testing::internal::GetCapturedStderr().empty()) {
    ADD_FAILURE() << "Unintended behavior with large data frame" ;
  }

  // @note some points will invitably get dropped due to
  // bad serial stream characters
  // but they will always be less than equal to tx points
  EXPECT_LE(scan.points.size(), numPoints);
}


/**
* Testing Sensor::shutdown.
* Desired output: Shutting down the sensor should delete underlying serial and config server objects,
* allow to start another sensor application and unsubscribe/ stop advertising on topics.
*/
TEST_F(SensorTest, shutdown)
{
  ros::NodeHandle nh;
  testing::internal::CaptureStderr();
  dev->shutdown();
  EXPECT_NO_THROW(std::make_unique<Sensor>(nh, *private_nh));

  if(!testing::internal::GetCapturedStderr().empty()) {
    ADD_FAILURE() << "Improper sensor shutdown detected";
  }
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_test");
  return RUN_ALL_TESTS();
}