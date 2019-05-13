/** @file     reconfig_test.cpp
 *  @author   Adi Singh, Christopher Lang, Roua Mokchah
 *  @date     April 2019
 */

#include <fcntl.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <dynamic_reconfigure/server.h>


/**
 * Testing Sensor::_reconfig.
 */

char kReconfigTestBuff[100] = "";

class ReconfigTest : public ::testing::Test
{
public:
  const std::string TAG = "\033[36m[DriverReconfigTest]\033[00m - ";

protected:
  ros::NodeHandle* private_nh;
  std::string mock_sensor;
  int conn_handle;
  char buff[100];

  void SetUp()
  {
    memset(&buff, '\0', sizeof(buff));
    private_nh = new ros::NodeHandle("~");
    private_nh->getParam("mock_sensor", mock_sensor);
    conn_handle = open(mock_sensor.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    // Reads in initial commands sent by Sensor::_init
    int n_bytes = 0;
    while(!strlen(kReconfigTestBuff))
    {
      n_bytes = read(conn_handle, &kReconfigTestBuff, sizeof(kReconfigTestBuff));
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
    std::cerr << TAG << "\tUpdating parameter server with "
      << param_name << " of " << param_value << "...";

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

    std::cerr << "done\n";
  }


  void listen()
  {
    std::cerr << TAG << "\tListening for commands on " << mock_sensor << "...";

    int n_bytes = 0;
    while(n_bytes < 1)
    {
      n_bytes = read(conn_handle, &buff, sizeof(buff));
      ros::Duration(0.01).sleep();
    } 
    std::cerr << "done\n";
  }
};



/**
 * Tests that the Sensor settings are initialized.
 * Desired behavior: Message is well formed and output clipped to paramter value limits.
 */

TEST_F(ReconfigTest, checkInitConfig)
{
  std::cerr << TAG << "<checkInitConfig>\n";

  std::string exp = "CnWave00005\rCfiltr00020\rCdThre00005\rCboost00500\r";
  std::cerr << TAG << "\tChecking initial command flush...";
  EXPECT_STREQ(kReconfigTestBuff, exp.c_str());
  std::cerr << "done\n";

  std::cerr << TAG << "</checkInitConfig>\n";
} 


/**
 * Tests that changes in the sensor parameters are being updated.
 * Desired behavior: Message is well formed and output clipped to paramter value limits.
 */

TEST_F(ReconfigTest, changeSigStrength)
{
  std::cerr << TAG << "<changeSigStrength>\n";

  this->updateCfg("sig_strength", 12);
  this->listen();
  EXPECT_EQ(std::string(buff),"CnWave00012\r");

  std::cerr << TAG << "</changeSigStrength>\n";
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_reconfig_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

