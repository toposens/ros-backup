#include <gtest/gtest.h>
#include <toposens_driver/sensor.h>
#include <fcntl.h>

using namespace toposens_driver;

  class SensorTest : public ::testing::Test
  {
    protected:
      Sensor* dev;
      int _mock_sensor_fd;
      std::stringstream data;

      void SetUp() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Serial::send needs a destination or no bytes are written
        _mock_sensor_fd = open("/dev/tnt0", O_RDWR | O_NOCTTY | O_NDELAY);

        dev = new Sensor(nh, private_nh);
        data.str(std::string());
      }

      void TearDown() {
        data.clear();
        dev->shutdown();
        delete dev;
        close(_mock_sensor_fd);
      }
  };

  /**
   * Testing the parse method with a valid data frame
   */
  TEST_F(SensorTest, sensorParseValidFrame)
  {
    // Data frame consisting of 4 measured points
    data << "S000016"
            "P0000X-0415Y00010Z00257V00061"
            "P0000X-0235Y00019Z00718V00055"
            "P0000X-0507Y00043Z00727V00075"
            "P0000X00142Y00360Z01555V00052"
            "E";

    toposens_msgs::TsScan scan;
    dev->parse(scan, data);

    std::vector<toposens_msgs::TsPoint>::iterator pt = scan.points.begin();

    // Test point P1(-0.415, 0.010, 0.257, 0.61)
    EXPECT_FLOAT_EQ(-0.415, pt->location.x);
    EXPECT_FLOAT_EQ(0.01,   pt->location.y);
    EXPECT_FLOAT_EQ(0.257,  pt->location.z);
    EXPECT_FLOAT_EQ(0.61,   pt->intensity);

    // Test point P2(-0.235, 0.019, 0.718, 0.55);
    pt++;
    EXPECT_FLOAT_EQ(-0.235, pt->location.x);
    EXPECT_FLOAT_EQ(0.019,  pt->location.y);
    EXPECT_FLOAT_EQ(0.718,  pt->location.z);
    EXPECT_FLOAT_EQ(0.55,   pt->intensity);

    // Test point P3(-0.507, 0.043, 0.727, 0.75)
    pt++;
    EXPECT_FLOAT_EQ(-0.507, pt->location.x);
    EXPECT_FLOAT_EQ(0.043,  pt->location.y);
    EXPECT_FLOAT_EQ(0.727,  pt->location.z);
    EXPECT_FLOAT_EQ(0.75,   pt->intensity);

    // Test point P4(0.142, 0.360, 1.555, 0.52)
    pt++;
    EXPECT_FLOAT_EQ(0.142, pt->location.x);
    EXPECT_FLOAT_EQ(0.360, pt->location.y);
    EXPECT_FLOAT_EQ(1.555, pt->location.z);
    EXPECT_FLOAT_EQ(0.52,  pt->intensity);


  }


  // Parse valid calibration confirmation
  TEST_F(SensorTest, sensorParseEmptyFrame)
  {
    data << "S001020E";
    toposens_msgs::TsScan scan;
    dev->parse(scan, data);
    EXPECT_TRUE(scan.points.size() == 0) ;
  }


  /**
   * Test parsing of bad data frame.
   * Desired behavior: extract no points from empty data frame.
   */
  TEST_F(SensorTest, sensorParseBadDataFrame)
  {
    data << "SE";
    toposens_msgs::TsScan scan;
    dev->parse(scan, data);
    EXPECT_TRUE(scan.points.size() == 0)<< "SE data frame did generate points";
  }


  /**
 * Test parsing of empty data frame.
 * Desired behavior: extract no points from empty data frame.
 */
  TEST_F(SensorTest, parseEmptyDataFrame)
  {
    data << "S153698E";
    toposens_msgs::TsScan scan;
    dev->parse(scan, data);
    EXPECT_TRUE(scan.points.size() == 0) << "Empty data frame did generate points!";
    data.clear();

    data << "SXYZVE";
    dev->parse(scan, data);
    EXPECT_TRUE(scan.points.size() == 0) << "Empty data frame did generate points!";
  }


  /**
   * Test for detecting invalid data frames.
   * Desired behavior: Detect and discard invalid data frames.
   */
   //TODO: essentially the same tests as parseDropInvalidPoints, so maybe we can omit this one.
  TEST_F(SensorTest, sensorParseInvalid)
  {
    // Pair of invalid frame and error description (what makes frame invalid)
    std::list<std::pair<std::string, std::string> > invalidFrames;

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010Z00257E",
                                              "Missing intensity value not detected!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010Z0025700061E",
                                              "Missing intensity tag not detected!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010V00061E",
                                              "Missing Z coordinate not detected!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y0001000257V00061E",
                                              "Missing Z tag not detected!"));

//    invalidFrames.emplace_back(std::make_pair("000016P0000X-0415Y00010Z00257V00061E",
//                                              "Missing S tag ignored!"));
//
//    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010Z00257V00061",
//                                              "Missing E tag ignored!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010Y00257V00061E",
                                              "Double Y coordinate not detected!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000-04150001000257V00061E",
                                              "One point missing XYZ tags!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-04E5Y00010Z00257V00061E",
                                              "Characters in X coordinate not detected!"));

    for (auto frame : invalidFrames) {
      data << frame.first;

      toposens_msgs::TsScan scan;

      scan.header.stamp = ros::Time::now();
      scan.header.frame_id = "toposens";

      dev->parse(scan, data);

      if (scan.points.size() != 0) ADD_FAILURE() << frame.second;

    }
  }


  /**
    * Test for handling invalid data frames.
    * Desired behavior: faulty data point(s) are discarded, valid data points are kept.
    */
  TEST_F(SensorTest, sensorParseDropInvalidPoints)
  {

    // Pair of invalid frame and error description (what makes frame invalid)
    std::list<std::pair<std::string, std::string> > invalidFrames;

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010Z00257V00061P0000X-0235Y00019Z00718E",
                                              "Missing intensity value in second point not detected!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010Z00257V00061P0000X-0235Y00019Z0071800055E",
                                              "Missing intensity tag in second point not detected!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010Z00257V00061P0000X-0235Y00019ZV00055E",
                                              "Missing Z coordinate in second point not detected!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010Z00257V00061P0000X-0235Y0001900718V00055E",
                                              "Missing Z tag in second point not detected!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010Z00257V00061P0000X-0235Y00019Y00718V00055E",
                                              "Double Y coordinate in second point not detected!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010Z00257V00061PV00055E",
                                              "Second point missing XYZ not detected!"));

    invalidFrames.emplace_back(std::make_pair("S000016P0000X-0415Y00010Z00257V00061P0000X-0Z35Y00019Z00718V00055E",
                                              "Characters in X coordinate in second point not detected!"));

    for (auto frame : invalidFrames) {
      data << frame.first;

      toposens_msgs::TsScan scan;
      dev->parse(scan, data);

      if (scan.points.size() == 2) {
        ADD_FAILURE() << frame.second;
      } else if (scan.points.size() == 0) {
        ADD_FAILURE() << "Valid point not detected for case: " << frame.second;
      } else if (scan.points.size() < 0 || scan.points.size() > 2) {
        ADD_FAILURE() << "Invalid points added for case: " << frame.second;
      } else {
        auto pt = scan.points.front();
        EXPECT_FLOAT_EQ(-0.415, pt.location.x) << " for case: " << frame.second;
        EXPECT_FLOAT_EQ(0.01, pt.location.y) << " for case: " << frame.second;;
        EXPECT_FLOAT_EQ(0.257, pt.location.z) << " for case: " << frame.second;;
        EXPECT_FLOAT_EQ(0.61, pt.intensity) << " for case: " << frame.second;
      }
    }
  }


  /**
   * Test for exceptions if large data frame is processed.
   * Desired behavior: Method does not raise any exception, even for large point clouds.
   */
  TEST_F(SensorTest, sensorParseLargeDataFrame)
  {
    data << "S000016";

    int numPoints = 5000;
    while (--numPoints) data << "P0000X00000Y00000Z00000V00001";
    data << "E";
    toposens_msgs::TsScan scan;
    EXPECT_NO_THROW(dev->parse(scan, data));
  }


  /**
   * Test whether stringstream is properly reset, ensure stationary behavior.
   * Desired behavior: stringstream is empty and all flags are in the same state as before the method call.
   */
  TEST_F(SensorTest, sensorParseResetStringStream)
  {
    toposens_msgs::TsScan scan;

    auto beforeState = data.rdstate();

    data << "Hello";

    dev->parse(scan, data);

    EXPECT_STREQ("", data.str().c_str())<< "Stringstream was not properly reset";

    EXPECT_EQ(beforeState, data.rdstate());
  }

  /**
 * Testing the sensor shutdown.
 * Desired output: Shutting down the sensor should free the serial port and allow to start another sensor application.
 */
  TEST_F(SensorTest, shutdown)
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    dev->shutdown();
    EXPECT_NO_THROW(std::make_unique<Sensor>(nh, private_nh));
  }


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_sensor_test_node");
  return RUN_ALL_TESTS();
}