/** @file     pointcloud_test.cpp
 *  @author   Roua Mokchah
 *  @date     April 2019
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <toposens_driver/sensor.h>
#include <toposens_pointcloud/cloud.h>


using namespace toposens_pointcloud;

class CloudTest : public ::testing::Test
{

public:
  const std::string TAG = "[POINTCLOUD_CLOUD_TEST] - ";

protected:
  // Defined in static tf broadcast in launch file
  const double tf_x = 0.05;
  const double tf_y = -0.1;
  const double tf_z = 0.0;

  const int kNumPoints = 10;
//@todo rename cloud class to something else to not confuse with TsCloud
  // ex, see Markers changed to Plot

  ros::NodeHandle* priv_nh;
  Cloud* c;

  TsCloud cloud;

  ros::Subscriber cloud_sub;
  ros::Publisher scans_pub;
  toposens_msgs::TsScan scan;
  std::vector<toposens_msgs::TsPoint> rcvd_points;

  void SetUp()
  {
    ros::NodeHandle nh;
    priv_nh = new ros::NodeHandle("~");
    c = new Cloud(nh, *priv_nh);

    scans_pub = nh.advertise<toposens_msgs::TsScan>(
                  toposens_driver::kScansTopic, 
                  toposens_driver::kQueueSize
                );
    cloud_sub = nh.subscribe(kPointCloudTopic, 100, &CloudTest::store, this);

    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "toposens";
  }


  void TearDown()
  {
    rcvd_points.clear();
    scan.points.clear();
    delete c;
    delete priv_nh;
  }


  void store(const pcl::PointCloud<toposens_msgs::TsPoint>::ConstPtr &msg)
  {
    for (auto p : msg->points) rcvd_points.push_back(p);
  }


  void listen()
  {
    std::cerr << TAG << "Listening for points...";
    ros::Time end = ros::Time::now() + ros::Duration(1.0);
    while(ros::Time::now() < end)
    {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
    std::cerr << "completed" << std::endl;
  }


  void read(std::string filename)
  {
    std::cerr << TAG << "Reading PCD data from " << filename << ".pcd...";
    pcl::io::loadPCDFile<toposens_msgs::TsPoint>(
      ros::package::getPath("toposens_pointcloud") + "/" + filename + ".pcd",
      cloud
    );
    std::cerr << "done" << std::endl;
  }

};


/**
 * Tests if an empty scan generates PointCloud messages.
 * Desired behavior: Zero PointCloud messages if an empty scan (zero-initialized) was published.
 */
TEST_F(CloudTest, emptyScan)
{
  std::cerr << TAG << "Publishing empty scan...";
  scans_pub.publish(scan);
  std::cerr << "done" << std::endl;

  listen();
  EXPECT_EQ(rcvd_points.size(), 0) << "Empty scan produced pointcloud.";
}


TEST_F(CloudTest, zeroIntensityScan)
{
  std::cerr << TAG << "Publishing scan with zero-intensity points...";

  for(int i = 0; i < kNumPoints; i++)
  {
    toposens_msgs::TsPoint pt;
    pt.location.x = pt.location.y = pt.location.z = (i + 1)/100.0;
    pt.intensity = 0;
    scan.points.push_back(pt);
  }

  scans_pub.publish(scan);
  std::cerr << "done" << std::endl;

  listen();
  EXPECT_EQ(rcvd_points.size(), 0) << "Zero-intensity points produced pointcloud.";
}



/**
 *  Tests that each incoming scan is converted to a new PointCloud message of
 *  template XYZI.
 */
TEST_F(CloudTest, validScan)
{
  std::cerr << TAG << "Publishing scan with plottable points...";

  for(int i = 0; i < kNumPoints; i++)
  {
    toposens_msgs::TsPoint pt;
    pt.location.x = pt.location.y = pt.location.z = (i + 1)/100.0;
    pt.intensity = i + 1;
    scan.points.push_back(pt);
  }

  scans_pub.publish(scan);
  std::cerr << "done" << std::endl;

  listen();
  EXPECT_EQ(rcvd_points.size(), kNumPoints) << "Valid points omitted in pointcloud.";

  for (int i = 0; i < kNumPoints; i++)
  {
    auto expc_pt = scan.points.at(i);
    auto rcvd_pt = rcvd_points.at(i);

    EXPECT_FLOAT_EQ(expc_pt.location.x + tf_x, rcvd_pt.location.x);
    EXPECT_FLOAT_EQ(expc_pt.location.y + tf_y, rcvd_pt.location.y);
    EXPECT_FLOAT_EQ(expc_pt.location.z + tf_z, rcvd_pt.location.z);
    EXPECT_FLOAT_EQ(expc_pt.intensity,         rcvd_pt.intensity );
  }
}


TEST_F(CloudTest, saveData)
{
  std::cerr << TAG << "Publishing scan with mixed points...";

  for(int i = 0; i < kNumPoints; i++)
  {
    toposens_msgs::TsPoint pt;
    pt.location.x = pt.location.y = pt.location.z = (i + 1)/100.0;
    pt.intensity = (i % 2) ? i : 0; // alternate intensity between 0 and i
    scan.points.push_back(pt);
  }

  scans_pub.publish(scan);
  std::cerr << "done" << std::endl;

  listen();
  std::string filename = "pointcloud_test";
  c->save(filename);

  read(filename);
  EXPECT_EQ(cloud.width, kNumPoints/2) << "Valid points not found in PCD file.";
/*
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
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_pointcloud_cloud_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
