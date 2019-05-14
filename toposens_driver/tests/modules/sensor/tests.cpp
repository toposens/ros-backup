/** @file     sensor_test.cpp
 *  @author   Adi Singh, Christopher Lang, Roua Mokchah
 *  @date     March 2019
 */

#include <fcntl.h>
#include <fstream>
#include <gtest/gtest.h>
#include <toposens_driver/sensor.h>

using namespace toposens_driver;

class SensorTest : public ::testing::Test
{
public:
  const std::string TAG = "\033[36m[DriverSensorTest]\033[00m - ";

protected:
  Sensor* dev;
  toposens_msgs::TsScan scan;
  ros::Subscriber sub;
  ros::NodeHandle* private_nh;
  std::string mock_sensor, driver_port, curr_frame;

  void SetUp()
  {
    ros::NodeHandle nh;
    private_nh = new ros::NodeHandle("~");
    private_nh->getParam("mock_sensor", mock_sensor);
    private_nh->getParam("port", driver_port);
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
    std::cerr << TAG << "\033[33m" << "\tReceived scan of size "
      << scan.points.size() << "\033[00m\n";
  }

  //@todo by testing poll, there's a bunch of duplicity introduced with serial
  // tests. Remove duplicate testing.
  void pollMockTx(std::string frame)
  {
    int n_bytes = frame.size();
    std::cerr << TAG << "\tTx nBytes: " << n_bytes << "\n";
    if (n_bytes < 100) std::cerr << TAG << "\tTx data: " << frame << "\n";
    std::cerr << TAG << "\tManually transmitting over: " << mock_sensor << "...";

    curr_frame = frame;
    write(open(mock_sensor.c_str(), O_RDWR | O_NOCTTY | O_NDELAY), frame.c_str(), n_bytes);
    std::cerr << "done\n";

    std::cerr << TAG << "\tSensor::poll over: " << driver_port << "...\n";
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
    if (p.size() == num_exp)
    {
      while (num_exp > 0) EXPECT_POINT(p[--num_exp], {-0.415, 0.01, 0.257, 0.61}, caseMsg);
    }
    else ADD_FAILURE() << "Invalid scan for case: " << caseMsg << "\r" << curr_frame;
    scan.points.clear();
  }
};


/**
 * Testing Sensor::poll with a valid data frame
 * Data frame consisting of 4 known points
 */

TEST_F(SensorTest, pollValidFrame)
{
  std::cerr << TAG << "<pollValidFrame>\n";

  std::string frame = "S000016"
                        "P0000X-0415Y00010Z00257V00061"
                        "P0000X-0235Y00019Z00718V00055"
                        "P0000X-0507Y00043Z00727V00075"
                        "P0000X00142Y00360Z01555V00052"
                      "E";
  this->pollMockTx(frame);

  ASSERT_EQ(scan.points.size(), (uint)4) << "Not enough points in scan: " << scan.points.size();
  EXPECT_POINT(scan.points[0], {-0.415, 0.01,  0.257, 0.61}, "Valid point P1");
  EXPECT_POINT(scan.points[1], {-0.235, 0.019, 0.718, 0.55}, "Valid point P2");
  EXPECT_POINT(scan.points[2], {-0.507, 0.043, 0.727, 0.75}, "Valid point P3");
  EXPECT_POINT(scan.points[3], { 0.142, 0.360, 1.555, 0.52}, "Valid point P4");

  std::cerr << TAG << "</pollValidFrame>\n";
}


/** Testing Sensor::poll with empty data frames
*  Desired behavior: extract no points from empty data frame.
*  No error is thrown, the frame should simply be ignored.
**/
TEST_F(SensorTest, pollEmptyFrames)
{
  std::cerr << TAG << "<pollEmptyFrames>\n";

  this->pollMockTx("S000000E");
  EXPECT_EMPTY("Valid empty frame 1");

  this->pollMockTx("S153698E");
  EXPECT_EMPTY("Valid empty frame 2");

  this->pollMockTx("");
  EXPECT_EMPTY("Invalid empty frame 1");

  this->pollMockTx("SE");
  EXPECT_EMPTY("Invalid empty frame 2");

  this->pollMockTx("SXYZV");
  EXPECT_EMPTY("Invalid empty frame 3");

  std::cerr << TAG << "</pollEmptyFrames>\n";
}


/**
 * Test for detecting invalid data frames.
 * Desired behavior: Detect and discard invalid data frames.
 */
TEST_F(SensorTest, pollInvalidPoints)
{
  std::cerr << TAG << "<pollInvalidPoints>\n";

  this->pollMockTx("S000016P0000X-0415Y00010Z00257E");
  EXPECT_EMPTY("Missing intensity value");

  this->pollMockTx("S000016P0000X-0415Y00010Z00257V061E");
  EXPECT_EMPTY("Intensity with <5 bytes");

  this->pollMockTx("S000016P0000X-0415Y00010V00061E");
  EXPECT_EMPTY("Missing Z coordinate");

  this->pollMockTx("S000016P0000-0415Y00010Z00257V00061E");
  EXPECT_EMPTY("Missing X tag");

  this->pollMockTx("S000016P0000X-0415Y00010Y00257V00061E");
  EXPECT_EMPTY("Double Y coordinate");

  this->pollMockTx("S000016P0000-04150001000257V00061E");
  EXPECT_EMPTY("Missing XYZ tags");

  this->pollMockTx("S000016P0000X-04E5Y00010Z00257V00061E");
  EXPECT_EMPTY("Characters in X coordinate");

  std::cerr << TAG << "</pollInvalidPoints>\n";
}


/** note: adjacent strings are auto concatenated by compiler
  * Test for handling invalid data frames.
  * Desired behavior: faulty data point(s) are discarded, valid data points are kept.
  */
TEST_F(SensorTest, pollTwoMixedPoints)
{
  std::cerr << TAG << "<pollTwoMixedPoints>\n";

  this->pollMockTx("S000016"
                    "P0000X-0415Y00010Z00257V00061"
                    "P0000X-0235Y00019Z00718"
                   "E");
  EXPECT_FILLED("Double Y coordinate", 1);


  this->pollMockTx("S000016"
                    "P0000X-0415Y00010Z00257V00061"
                    "P0000X-0235Y00019Z00718"
                   "E");
  EXPECT_FILLED("Missing intensity value in P2", 1);


  this->pollMockTx("S000016"
                    "P0000X-0415Y00010Z00257V00061"
                    "P0000X-0235Y00019ZV00055"
                   "E");
  EXPECT_FILLED("Missing Z coordinate in P2", 1);


  this->pollMockTx("S000016"
                    "P0000-0235Y00019Z00718V00055"
                    "P0000-0415Y00010Z00257V00061"
                   "E");
  EXPECT_EMPTY("Missing X tag in both P1 and P2");


  this->pollMockTx("S000016"
                    "P0000X-0415Y00010Z00257V00061"
                    "PV00055"
                   "E");
  EXPECT_FILLED("XYZ tags missing in P2", 1);

  std::cerr << TAG << "</pollTwoMixedPoints>\n";
}


TEST_F(SensorTest, pollThreeMixedPoints)
{
  std::cerr << TAG << "<pollThreeMixedPoints>\n";

  this->pollMockTx("S000016"
                    "P0000X-0415Y00010Z00257V00061"
                    "P0000X-0415Y00010Z00257V00061"
                    "P0000X-0235Y00019Z0071800055"
                   "E");
  EXPECT_FILLED("Missing intensity tag in P3", 2);


  this->pollMockTx("S000016"
                    "P0000X-0235Y00019Y00718V00055"
                    "P0000X-0415Y00010Z00257V00061"
                    "P0000X-0415Y00010Z00257Y00061"
                   "E");
  EXPECT_FILLED("Double Y coordinate in P1 and P3", 1);


  this->pollMockTx("S000016"
                    "P0000X-0415Y00010Z00257V00061"
                    "P0000X-0Z35Y00019Z00718V00055"
                    "P0000X-0415Y00010Z00257V00061"
                   "E");
  EXPECT_FILLED("Non-numerical characters in X coordinate of P2", 2);

  std::cerr << TAG << "</pollThreeMixedPoints>\n";
}


/**
 * Testing Sensor::poll with large data frame.
 * Desired behavior: Method does not raise any exception, even for large point clouds.
 */
TEST_F(SensorTest, pollLargeFrame)
{
  std::cerr << TAG << "<pollLargeFrame>\n";

  uint numPoints = 500;
  std::string frame = "S000016";
  while (numPoints--) frame += "P0000X-0415Y00010Z00257V00061";
  frame += "E";
  EXPECT_NO_THROW(this->pollMockTx(frame));

  // @note some points will invitably get dropped due to bad serial
  // stream characters but they will always be less than equal to tx points
  // @todo confirm if EXPECT_GT is working and try to get more points: ~500
  //EXPECT_LE(scan.points.size(), numPoints);
  EXPECT_GT(scan.points.size(), (long unsigned int)350);

  std::cerr << TAG << "</pollLargeFrame>\n";
}

/**
 * Testing Sensor::poll with valid data frames from dump file.
  read each single scan from the dump file : Example
  S000020P0000X-0104Y-0229Z00206V00036ES000025P0000X-0156Y-0229Z00206V00048E
  line 1 : empty -> continue
  line 2 : 000020P0000X-0104Y-0229Z00206V00036E
  line 3 : 000025P0000X-0156Y-0229Z00206V00048E
 */
TEST_F(SensorTest, pollStreamDump)
{
  std::cerr << TAG << "<pollStreamDump>\n";

  std::string filename;
  private_nh->getParam("file", filename);

  std::ifstream infile;
  infile.open(filename);

  std::string line;
  while (std::getline(infile, line, 'S'))
  {
    if (line.empty()) continue;
    EXPECT_NO_THROW(this->pollMockTx("S" + line));
  }
  infile.close();

  std::cerr << TAG << "</pollStreamDump>\n";
}


/**
* Testing Sensor::shutdown.
* Desired output: Shutting down the sensor should delete underlying serial and config server objects,
* allow to start another sensor application and unsubscribe/ stop advertising on topics.
*/
TEST_F(SensorTest, devShutdown)
{
  std::cerr << TAG << "<devShutdown>\n";
  std::cerr << TAG << "\tShutting down device on " << driver_port << "...";

  ros::NodeHandle nh;
  dev->shutdown();
  std::cerr << "done\n";

  EXPECT_NO_THROW(std::make_unique<Sensor>(nh, *private_nh));

  std::cerr << TAG << "</devShutdown>\n";
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_sensor_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
