/** @file     mapping/tests.cpp
 *  @author   Roua Mokchah
 *  @date     April 2019
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <toposens_driver/sensor.h>
#include <toposens_pointcloud/mapping.h>


using namespace toposens_pointcloud;

class MappingTest : public ::testing::Test
{
public:
  const std::string TAG = "\033[36m[PointcloudMappingTest]\033[00m - ";

protected:
  // Defined in static tf broadcast in launch file
  const double tf_x = 0.05;
  const double tf_y = -0.1;
  const double tf_z = 0.0;

  const int kNumPoints = 10;
  const std::string filename = "pointcloud_test";
  ros::NodeHandle* priv_nh;
  Mapping* m;

  TsCloud cloud;

  ros::Subscriber cloud_sub;
  ros::Publisher scans_pub;
  toposens_msgs::TsScan scan;
  std::vector<toposens_msgs::TsPoint> rcvd_points;

  void SetUp()
  {
    ros::NodeHandle nh;
    priv_nh = new ros::NodeHandle("~");
    m = new Mapping(nh, *priv_nh);

    scans_pub = nh.advertise<toposens_msgs::TsScan>(
      toposens_driver::kScansTopic, toposens_driver::kQueueSize);

    cloud_sub = nh.subscribe(kPointCloudTopic, 100, &MappingTest::store, this);

    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "toposens";
  }


  void TearDown()
  {
    rcvd_points.clear();
    scan.points.clear();
    delete m;
    delete priv_nh;
  }


  void store(const pcl::PointCloud<toposens_msgs::TsPoint>::ConstPtr &msg)
  {
    for (auto p : msg->points) rcvd_points.push_back(p);
    std::cerr << TAG << "\033[33m" << "\tReceived "
      << rcvd_points.size() << " PCL points\n" << "\033[00m";
  }


  void listen()
  {
    std::cerr << TAG << "\tListening for PCL points on " << kPointCloudTopic << "...\n";
    ros::Time end = ros::Time::now() + ros::Duration(1.0);
    while(ros::Time::now() < end)
    {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
  }


  void read()
  {
    std::cerr << TAG << "\tReading PCD data from " << filename << ".pcd...";
    pcl::io::loadPCDFile<toposens_msgs::TsPoint>(
      ros::package::getPath("toposens_pointcloud") + "/" + filename + ".pcd", cloud);
    std::cerr << "done\n";
  }

};


/**
 * Tests if an empty scan generates PointCloud messages.
 * Desired behavior: Zero PointCloud messages if an empty scan (zero-initialized) was published.
 */
TEST_F(MappingTest, emptyScan)
{
  std::cerr << TAG << "<emptyScan>\n";
  std::cerr << TAG << "\tPublishing empty scan...";

  scans_pub.publish(scan);
  std::cerr << "done\n";

  this->listen();
  EXPECT_EQ(rcvd_points.size(), 0);
  std::cerr << TAG << "</emptyScan>\n";
}



TEST_F(MappingTest, zeroIntensityScan)
{
  std::cerr << TAG << "<zeroIntensityScan>\n";
  std::cerr << TAG << "\tPublishing scan with zero-intensity points...";

  for(int i = 0; i < kNumPoints; i++)
  {
    toposens_msgs::TsPoint pt;
    pt.location.x = pt.location.y = pt.location.z = (i + 1)/100.0;
    pt.intensity = 0;
    scan.points.push_back(pt);
  }

  scans_pub.publish(scan);
  std::cerr << "done\n";

  this->listen();
  EXPECT_EQ(rcvd_points.size(), 0);
  std::cerr << TAG << "</zeroIntensityScan>\n";
}



/**
 *  Tests that each incoming scan is converted to a new PointCloud message of
 *  template XYZI.
 */
TEST_F(MappingTest, validPoints)
{
  std::cerr << TAG << "<validPoints>\n";
  std::cerr << TAG << "\tPublishing scan with plottable points...";

  for(int i = 0; i < kNumPoints; i++)
  {
    toposens_msgs::TsPoint pt;
    pt.location.x = pt.location.y = pt.location.z = (i + 1)/100.0;
    pt.intensity = i + 1;
    scan.points.push_back(pt);
  }

  scans_pub.publish(scan);
  std::cerr << "done\n";

  this->listen();
  EXPECT_EQ(rcvd_points.size(), kNumPoints);

  for (int i = 0; i < kNumPoints; i++)
  {
    auto expc_pt = scan.points.at(i);
    auto rcvd_pt = rcvd_points.at(i);

    // @todo confirm if EXPECT_NEAR is working
    EXPECT_NEAR(expc_pt.location.x + tf_x, rcvd_pt.location.x, 1.0e-15);
    EXPECT_NEAR(expc_pt.location.y + tf_y, rcvd_pt.location.y, 1.0e-15);
    EXPECT_NEAR(expc_pt.location.z + tf_z, rcvd_pt.location.z, 1.0e-15);
    EXPECT_NEAR(expc_pt.intensity,         rcvd_pt.intensity,  1.0e-15);
  }
  std::cerr << TAG << "</validPoints>\n";
}


TEST_F(MappingTest, saveData)
{
  std::cerr << TAG << "<saveData>\n";
  std::cerr << TAG << "\tPublishing scan with mixed points...";

  for(int i = 0; i < kNumPoints; i++)
  {
    toposens_msgs::TsPoint pt;
    pt.location.x = pt.location.y = pt.location.z = (i + 1)/100.0;
    pt.intensity = (i % 2) ? i : 0; // alternate intensity between 0 and i
    scan.points.push_back(pt);
  }

  scans_pub.publish(scan);
  std::cerr << "done\n";

  this->listen();
  std::cerr << TAG << "\tMapping::save on " << filename << "...";
  m->save(filename);
  std::cerr << "done\n";

  this->read();
  EXPECT_EQ(cloud.width, kNumPoints/2);
/*
// checks for expected number of points and that the data in each point is correct

  for (int i = 0; i < cloud.width; i++)
  {
    int index = (2 * i) + 1;
    float pos = (index + 1)/100.0;
    toposens_msgs::TsPoint pt = cloud.points.at(i);

    EXPECT_FLOAT_EQ(pos, pt.location.x);
    EXPECT_FLOAT_EQ(pos, pt.location.y);
    EXPECT_FLOAT_EQ(pos, pt.location.z);
    EXPECT_FLOAT_EQ(pos, pt.intensity);
  }
*/
  std::cerr << TAG << "</saveData>\n";
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_pointcloud_mapping_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
