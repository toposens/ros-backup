/** @file     integration_tests.cpp
 *  @author   Christopher Lang, Roua Mokchah
 *  @date     March 2019
 */
#include <vector>
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <toposens_msgs/TsScan.h>
#include <toposens_driver/TsDriverConfig.h>
#include <toposens_driver/sensor.h>

/**
 * Test that changes in sensor parameters are being applied.
 */

namespace toposens_driver {

  class ParamTest : public ::testing::Test {

    protected:
      TsDriverConfig cfg;
      std::vector<toposens_msgs::TsScan> _scans;
      ros::Subscriber _scans_sub;  /**< Handler for subscribing to TsScans.*/
      /**< Data structure for storing incoming scans.*/

      typedef dynamic_reconfigure::Server<TsDriverConfig> Cfg_server;
      std::unique_ptr<Cfg_server> param_server;

      // Variables to communicate with the dynamic_reconfiguration server
      dynamic_reconfigure::ReconfigureRequest srv_req;
      dynamic_reconfigure::ReconfigureResponse srv_resp;
      dynamic_reconfigure::IntParameter int_param;
      dynamic_reconfigure::GroupState group_state;
      dynamic_reconfigure::Config conf;

      virtual void SetUp() {
        ros::NodeHandle nh;

        // Subscribe to topic with TS scans
        _scans_sub = nh.subscribe(toposens_driver::kScansTopic, 100, &ParamTest::_accumulate, this);

        param_server = std::make_unique<Cfg_server>(nh);
        cfg.__fromServer__(nh);
      }

    public:
    /** All incoming scans are stored in a double-ended queue, which
     *  allows for easy iteration, sequential arrangement of data and
     *  efficient chronological cleaning in susequent runs.
     */
      void _accumulate(const toposens_msgs::TsScan::ConstPtr &msg) {
        ROS_DEBUG("Received.");

        _scans.push_back(*msg);
      }

    /**
      * Computes intensity median over TsScans in buffer, if buffer is empty waits for the first message.
      * @return median over all point intensities
      */
    float computeIntensitySum(){

      float intensities;

      while(_scans.size() < 1) ros::spinOnce();

      for(auto scan : _scans){
        for(auto point : scan.points)
          intensities += point.intensity*point.intensity;
      }

      return intensities;

    }

    void updateConfigurations(TsDriverConfig cfg) {

        int_param.name = "sig_strength";
        int_param.value = cfg.sig_strength;
        conf.ints.push_back(int_param);

        int_param.name = "noise_thresh";
        int_param.value = cfg.noise_thresh;
        conf.ints.push_back(int_param);

        int_param.name = "x_min";
        int_param.value = cfg.x_min;
        conf.ints.push_back(int_param);

        int_param.name = "x_max";
        int_param.value = cfg.x_max;
        conf.ints.push_back(int_param);

        int_param.name = "y_min";
        int_param.value = cfg.y_min;
        conf.ints.push_back(int_param);

        int_param.name = "y_max";
        int_param.value = cfg.y_max;
        conf.ints.push_back(int_param);

        int_param.name = "z_min";
        int_param.value = cfg.z_min;
        conf.ints.push_back(int_param);

        int_param.name = "z_max";
        int_param.value = cfg.z_max;
        conf.ints.push_back(int_param);

        group_state.name = "voxel";
        group_state.state = true;
        group_state.id = 2;
        group_state.parent = 0;
        conf.groups.push_back(group_state);

        srv_req.config = conf;

        ros::service::call("/ts_driver_node/set_parameters", srv_req, srv_resp);

        conf.ints.clear();
        conf.groups.clear();

        _scans.clear(); // Reset scans from all frames with old sensor settings
      }
  };


  /**
   * Test that changes in signal strength are being applied.
   *  Desired behavior: - set signal strength to 0 ==> No points are expected
   *                    - increase signal strength ==> median point intensity should increase
   */
  TEST_F(ParamTest, parameterSigStrength) {

    float intensity_before, intensity_after;

    // initialize the intensity
    cfg.sig_strength = 0;
    updateConfigurations(cfg);

    intensity_before = computeIntensitySum();

    // increase the intensity
    cfg.sig_strength = cfg.sig_strength + 10;
    updateConfigurations(cfg);

    intensity_after = computeIntensitySum();

    EXPECT_LT(intensity_before, intensity_after) << "Higher reception intensity with zero signal strength!";
  }


  /**
   * Test that changes in noise threshold are being applied.
   * Desired behavior: no point intensities lower than noise_thresh
   */
  TEST_F(ParamTest, parameterNoiseThreshold){

    int num_points_before, num_points_after;

    // Disable noise threshold
    cfg.noise_thresh = 0;
    updateConfigurations(cfg);

    while(_scans.size() == 0) ros::spinOnce();
    num_points_before = _scans.size();

    // Set noise threshold
    cfg.noise_thresh = 10;
    updateConfigurations(cfg);

    while(_scans.size() == 0) ros::spinOnce();
    num_points_after = _scans.size();

    EXPECT_GE(num_points_after, num_points_before) << "Noise shows no effect!";
  }


  /**
   * Test that changes in VoxelLimits are being applied.
   * Desired behavior: Sensor only reports point coordinates inside voxel limits
   */
  TEST_F(ParamTest, parameterVoxelLimits) {

    // change configurations here
    auto timestamp = ros::Time::now();

    cfg.x_max = 300;
    updateConfigurations(cfg);

    while(_scans.size() == 0) ros::spinOnce();

    for(auto scan : _scans){
      if(scan.header.stamp < timestamp) continue;

      for(auto point : scan.points){
        EXPECT_GE(point.location.x, cfg.x_min) << "Voxel x limit not adhered to!";
        EXPECT_LE(point.location.x, cfg.x_max) << "Voxel x limit not adhered to!";

        EXPECT_GE(point.location.y, cfg.y_min) << "Voxel y limit not adhered to!";
        EXPECT_LE(point.location.y, cfg.y_max) << "Voxel y limit not adhered to!";

        EXPECT_GE(point.location.z, cfg.z_min) << "Voxel z limit not adhered to!";
        EXPECT_LE(point.location.z, cfg.z_max) << "Voxel z limit not adhered to!";
      }
    }
  }

}// namespace toposens


int main(int argc, char** argv)
{
  ros::init(argc, argv, "TsDriverParamTestNode");
  testing::InitGoogleTest(&argc, argv);

  auto res = RUN_ALL_TESTS();

  ros::shutdown();

  return res;
}