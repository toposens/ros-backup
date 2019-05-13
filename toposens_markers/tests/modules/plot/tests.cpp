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
public:
  const std::string TAG = "\033[36m[MarkersPlotTest]\033[00m - ";

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
      toposens_driver::kScansTopic, toposens_driver::kQueueSize);
  
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
    std::cerr << TAG << "\tListening for markers on " << kMarkersTopic << "...\n";

    ros::Time end = ros::Time::now() + ros::Duration(1.0);
    while(ros::Time::now() < end)
    {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
  }


  void store(const visualization_msgs::MarkerArray::ConstPtr &msg)
  {
    for (auto &m : msg->markers)
    {
      if (m.ns == kMarkersNs) markers.push_back(m);
    }
    std::cerr << TAG << "\033[33m" << "\tReceived "
      << markers.size() << " markers\n" << "\033[00m";
  }
};


/**
*  Tests that no empty scans are plotted.
*  Desired behavior: Points to be plotted is zero if an empty scan (zero-initialized) was published.
*/
TEST_F(PlotTest, emptyScan)
{
  std::cerr << TAG << "<emptyScan>\n";
  std::cerr << TAG << "\tPublishing empty scan...";

  scans_pub.publish(scan);
  std::cerr << "done\n";

  this->listen();
  EXPECT_EQ(markers.size(), 0);

  std::cerr << TAG << "</emptyScan>\n";
}



TEST_F(PlotTest, zeroIntensityScan)
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
  EXPECT_EQ(markers.size(), 0);

  std::cerr << TAG << "</zeroIntensityScan>\n";
}



/**
 *  Tests that all the received points/scans are plotted.
 *  Desired behavior: All valid points/scans are plotted in the same order with which they were published.
 */
TEST_F(PlotTest, validPoints)
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
  ASSERT_EQ(markers.size(), kNumPoints);

  for (int i = 0; i < kNumPoints; i++)
  {
    auto exp_pt = scan.points.at(i);
    auto rec_pt = markers.at(i);

    EXPECT_FLOAT_EQ(exp_pt.location.x, rec_pt.pose.position.x);
    EXPECT_FLOAT_EQ(exp_pt.location.y, rec_pt.pose.position.y);
    EXPECT_FLOAT_EQ(exp_pt.location.z, rec_pt.pose.position.z);

    // marker scale is directly proportional (but not equal) to
    // point intenstiy. since point intensities are added in increasing
    // order (i+1), we can at least expect that their corresponding
    // markers will be arranged in order of increasing scale
    if (i > 0) EXPECT_GT(rec_pt.scale.x, markers.at(i - 1).scale.x);
  }

  std::cerr << TAG << "</validPoints>\n";
}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_markers_plot_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}