/** @file     reconfig_test.cpp
 *  @author   Adi Singh, Roua Mokchah
 *  @date     April 2019
 */

#include <ros/ros.h>
#include <gtest/gtest.h>

#include "toposens_markers/plot.h"

#include <dynamic_reconfigure/server.h>
#include <toposens_markers/TsMarkersConfig.h>

/**
 * Testing Markers::_reconfig.
 */
using namespace toposens_markers ;

class ReconfigTest : public ::testing::Test
{

public:
  const std::string TAG = "[MARKERS_RECONFIG_TEST] - ";

protected:
  const float listen_period = 1.0;

  ros::Subscriber markers_sub;
  ros::Publisher scans_pub;
  toposens_msgs::TsScan scan;

  std::vector<visualization_msgs::Marker> markers;


  void SetUp()
  {
    ros::NodeHandle nh;
    scans_pub = nh.advertise<toposens_msgs::TsScan>(
                  toposens_driver::kScansTopic,
                  toposens_driver::kQueueSize
                );

    markers_sub = nh.subscribe(kMarkersTopic, 100, &ReconfigTest::store, this);
    ros::Duration(0.5).sleep(); // delay to allow node to start and set everything up
  }


  void TearDown()
  {
    markers.clear();
  }


  void store(const visualization_msgs::MarkerArray::ConstPtr &msg)
  {
    for (auto &m : msg->markers) {
      if (m.ns == kPointsNs) markers.push_back(m);
    }
  }


  void publishAndListen()
  {
    // publish a unit scan
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "toposens";
    scan.points.clear();

    toposens_msgs::TsPoint pt;
    pt.intensity = 1.0;
    scan.points.push_back(pt);
    scans_pub.publish(scan);

    // listen for produced marker(s)
    std::cerr << TAG << "Listening for markers...";
    ros::Time end = ros::Time::now() + ros::Duration(listen_period);
    while(ros::Time::now() < end) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
    std::cerr <<"completed" << std::endl;
  }



  void updateCfg(std::string param_name, double param_value)
  {
    std::cerr << TAG << "Setting plot " << param_name << " to " << param_value << std::endl;
    
    dynamic_reconfigure::DoubleParameter double_param;
    double_param.name = param_name;
    double_param.value = param_value;

    dynamic_reconfigure::Config conf;
    conf.doubles.push_back(double_param);

    dynamic_reconfigure::ReconfigureRequest req;
    req.config = conf;

    dynamic_reconfigure::ReconfigureResponse res;
    ros::service::call("/ts_markers_node/set_parameters", req, res);
  }

};


TEST_F(ReconfigTest, changeScale)
{
  double def_scale = 1.0;
  double new_scale = 4.0;

  updateCfg("scale", def_scale);
  publishAndListen();

  updateCfg("scale", new_scale);
  publishAndListen();

  double def_size = markers.at(0).scale.x;
  double new_size = markers.at(1).scale.x;

  double exp_size = def_size * (new_scale / def_scale);
  EXPECT_EQ(exp_size, new_size);

}

// note... this is a complex function to test because of race conditions
// @todo cleanup documenation here
// @note spin() is being called by markers node. so you cannot wait to call callback
// the first listen lapses a finite amount of time, so the slep duration we need is only 
// the remainder of time
// to see this fail, change min_sleep duration to a value between 0 and (new_lifetime - listen_period)
TEST_F(ReconfigTest, changeLifetime)
{
  double new_lifetime = 1.2;
  double min_sleep = new_lifetime - listen_period;
  if (min_sleep < 0) return ADD_FAILURE() << TAG <<  "Min sleep duration cannot be negative: " << min_sleep;

  updateCfg("lifetime", new_lifetime);
  publishAndListen();     // plot has scan after stored after this

  // Let the lifetime lapse... liftime  = listen_period  + sleep
  ros::Duration(min_sleep).sleep();
  markers.clear();

  // then execute marker callback after pause
  // need to publish a scan once more to trigger plot callback
  // because lifetime comparison and saniztizing happens in callback
  publishAndListen();  // this scan has fresh timestamp. the previous one should be expired by now.

  // If lifetime works, only latest scan should produce marker.
  // If lifetime failed, 2 markers are produced: 1 for past scan + 1 for latest scan
  EXPECT_EQ(markers.size(), 1)<< "Unexpected behavior after lifetime update.";
  
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_markers_reconfig_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

