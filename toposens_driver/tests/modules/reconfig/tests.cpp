/** @file     reconfig_test.cpp
 *  @author   Adi Singh, Christopher Lang, Roua Mokchah
 *  @date     April 2019
 */
#include <vector>
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <dynamic_reconfigure/server.h>
#include <toposens_driver/TsDriverConfig.h>

#include <termios.h>
#include <fcntl.h>

/**
 * Testing Sensor::_reconfig.
 */
using namespace toposens_driver;

class ReconfigTest : public ::testing::Test
{

  const std::string TAG = "[DRIVER_RECONFIG_TEST] - ";

  protected:
    ros::NodeHandle* private_nh;
    int mock_sensor;
    char init_buff[100];

    void SetUp()
    {
      int n_bytes = 0;
      private_nh = new ros::NodeHandle("~");
      std::string mock_port;
      private_nh->getParam("sensor_port", mock_port);

      mock_sensor = open(mock_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

      // Flushes out initial commands sent by _init
      while((n_bytes < 1) && (std::strlen(init_buff)==0)) {
        n_bytes = read(mock_sensor, &init_buff, sizeof(init_buff));
      }
    }

    void TearDown()
    {
      close(mock_sensor);
      delete private_nh;
    }

    void updateCfg(std::string param_name, int param_value)
    {
      dynamic_reconfigure::IntParameter int_param;
      int_param.name = param_name;
      int_param.value = param_value;

      dynamic_reconfigure::Config conf;
      conf.ints.push_back(int_param);

      dynamic_reconfigure::ReconfigureRequest req;
      req.config = conf;

      dynamic_reconfigure::ReconfigureResponse res;
      ros::service::call("/ts_driver_node/set_parameters", req, res);
    }
};



/**
 * Tests that changes in the sensor parameters are being updated.
 * Desired behavior: Message is well formed and output clipped to paramter value limits.
 */

TEST_F(ReconfigTest, changeSigStrength)
{
  char buff[100];
  int n_bytes = 0;
  memset(&buff, '\0', sizeof(buff));

  updateCfg("sig_strength", 12);

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  while((n_bytes < 1))n_bytes = read(mock_sensor, &buff, sizeof(buff));

  EXPECT_EQ(std::string(buff),"CnWave00012\r");

}


/**
 * Tests that the Sensor settings are initialized.
 * Desired behavior: Message is well formed and output clipped to paramter value limits.
 */

TEST_F(ReconfigTest, checkInitConfig)
{
 EXPECT_EQ(std::string(init_buff),"CnWave00005\rCfiltr00020\rCdThre00005\rCboost00500\r");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_reconfig_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

