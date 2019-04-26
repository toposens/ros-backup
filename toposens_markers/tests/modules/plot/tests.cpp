/** @file     point_test.cpp
 *  @author   Roua Mokchah
 *  @date     March 2019
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <toposens_markers/plot.h>
#include <toposens_driver/sensor.h>

using namespace toposens_markers;

class PlotTest : public ::testing::Test
{
  protected:
    const int kNumPoints = 10;
    ros::NodeHandle* priv_nh;
    Plot* p;

    ros::Subscriber markers_sub;
    ros::Publisher scans_pub;
    toposens_msgs::TsScan scan;
    std::vector<visualization_msgs::Marker> markers;

    void SetUp()
    {
      ros::NodeHandle nh;
      priv_nh = new ros::NodeHandle("~");
      p = new Plot(nh, *priv_nh);

      scans_pub = nh.advertise<toposens_msgs::TsScan>(
                    toposens_driver::kScansTopic, 
                    toposens_driver::kQueueSize
                  );
    
      markers_sub = nh.subscribe(kMarkersTopic, 100, &PlotTest::store, this);

      scan.header.stamp = ros::Time::now();
      scan.header.frame_id = "toposens";
    }

    void TearDown()
    {
      markers.clear();
      scan.points.clear();
      delete p;
      delete priv_nh;
    }


    void listen()
    {
      std::cerr << "[TEST] Listening for markers...";
      ros::Time end = ros::Time::now() + ros::Duration(1.0);
      while(ros::Time::now() < end)
      {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
      }
      std::cerr << "completed" << std::endl;
    }


    void store(const visualization_msgs::MarkerArray::ConstPtr &msg)
    {
      for (auto &m : msg->markers)
      {
        if (m.ns == kPointsNs) markers.push_back(m);
      }
    }

};


/**
*  Tests that no empty scans are plotted.
*  Desired behavior: Points to be plotted is zero if an empty scan (zero-initialized) was published.
*/
TEST_F(PlotTest, publishEmptyScan)
{
  std::cerr << "[TEST] Publishing empty scan...";
  scans_pub.publish(scan);
  std::cerr << "done" << std::endl;

  listen();
  EXPECT_EQ(markers.size(), 0) << "Empty scan produced markers.";
}




TEST_F(PlotTest, publishZeroIntensityScan)
{
  std::cerr << "[TEST] Publishing scan with zero-intensity points...";
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
  EXPECT_EQ(markers.size(), 0) << "Zero-intensity points produced markers.";
}



/**
 *  Tests that all the received points/scans are plotted.
 *  Desired behavior: All valid points/scans are plotted in the same order with which they were published.
 */
TEST_F(PlotTest, publishValidScans)
{
  std::cerr << "[TEST] Publishing scan with plottable points...";

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
  EXPECT_EQ(markers.size(), kNumPoints) << "Valid points omitted in plot.";

  for (int i = 0; i < kNumPoints; i++)
  {
    auto exp_pt = scan.points.at(i);
    auto rec_pt = markers.at(i);

    EXPECT_FLOAT_EQ(exp_pt.location.x, rec_pt.pose.position.x);
    EXPECT_FLOAT_EQ(exp_pt.location.y, rec_pt.pose.position.y);
    EXPECT_FLOAT_EQ(exp_pt.location.z, rec_pt.pose.position.z);

    if (i > 0) EXPECT_GT(rec_pt.scale.x, markers.at(i - 1).scale.x);
  }
}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_markers_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}