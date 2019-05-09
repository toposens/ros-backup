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

char kReconfigTestInitBuff[100] = "";

class ReconfigTest : public ::testing::Test
{

public:
  const std::string TAG = "[DRIVER_RECONFIG_TEST] - ";

protected:
    ros::NodeHandle* private_nh;
    int conn_handle;


    void SetUp()
    {
      private_nh = new ros::NodeHandle("~");
      std::string mock_sensor;
      private_nh->getParam("mock_sensor", mock_sensor);
      conn_handle = open(mock_sensor.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

      // Reads in initial commands sent by Sensor::_init
      int n_bytes = 0;
      while(!strlen(kReconfigTestInitBuff))
      {
        n_bytes = read(conn_handle,
                    &kReconfigTestInitBuff,
                    sizeof(kReconfigTestInitBuff)
                  );
        if (n_bytes > 0) break;
        ros::Duration(0.01).sleep();
      }
    }

    void TearDown()
    {
      close(conn_handle);
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
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }
};



/**
 * Tests that the Sensor settings are initialized.
 * Desired behavior: Message is well formed and output clipped to paramter value limits.
 */

TEST_F(ReconfigTest, checkInitConfig)
{
  EXPECT_STREQ(kReconfigTestInitBuff, "CnWave00005\rCfiltr00020\rCdThre00005\rCboost00500\r");
} 


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

  while((n_bytes < 1))n_bytes = read(conn_handle, &buff, sizeof(buff));
  EXPECT_EQ(std::string(buff),"CnWave00012\r");
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_reconfig_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

