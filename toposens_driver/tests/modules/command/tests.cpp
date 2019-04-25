/** @file     command_test.cpp
 *  @author   Christopher Lang, Roua Mokchah, Adi Singh
 *  @date     March 2019
 */
 
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <toposens_driver/command.h>

using namespace toposens_driver;

class CommandTest : public ::testing::Test
{
  protected:
    static const int CMD_LEN = 12;

    // @note keep test fixture, may be needed later
    void SetUp() {
    }

    void TearDown() {
    }

};


/**
 * Testing the Command constructor with valid values.
 *  Tests if the command message is well formed.
 *  Desired behavior: The constructor should generate the correct command message
 *  using the given parameters.
 */

TEST_F(CommandTest, validValues)
{
  Command cmd_1(Command::SigStrength, 5);
  EXPECT_STREQ(cmd_1.getBytes(), "CnWave00005\r");

  Command cmd_2(Command::FilterSize, 42);
  EXPECT_STREQ(cmd_2.getBytes(), "Cfiltr00042\r");

  Command cmd_3(Command::NoiseThresh, 15);
  EXPECT_STREQ(cmd_3.getBytes(), "CdThre00015\r");

 // @todo rename this to slopf
  Command cmd_4(Command::SNRBoostNear, 0);
  EXPECT_STREQ(cmd_4.getBytes(), "Cslop100000\r");

  Command cmd_5(Command::CalibTemp, -3);
  EXPECT_STREQ(cmd_5.getBytes(), "CDTemp-0003\r");
}


// @todo extend command interface for other new commands
// @todo remove glim
// @todo consider what new commands to include in ROS api
// @todo Sensor::reconfig should be tested too!
/**
 * Testing the behavior of the Command constructor with invalid Inputs.
 * Desired behavior: Generated command bytes should consist of a fixed length and a valid param name.
 */
TEST_F(CommandTest, invalidValues)
{
  Command cmd_1(Command::SigStrength, -4678);
  EXPECT_STREQ(cmd_1.getBytes(), "CnWave-4678\r");

  Command cmd_2(Command::FilterSize, NULL);
  EXPECT_STREQ(cmd_2.getBytes(), "Cfiltr00000\r");

  Command cmd_3(Command::NoiseThresh, 000000);
  EXPECT_STREQ(cmd_3.getBytes(), "CdThre00000\r");

 // @todo rename this to slopf
  // Test conversions between number system representations.
  Command cmd_4(Command::SNRBoostNear, 00604);
  EXPECT_STREQ(cmd_4.getBytes(), "Cslop100388\r");

  Command cmd_5(Command::CalibTemp, INT_MAX);
  EXPECT_STREQ(cmd_5.getBytes(), "CDTemp09999\r");
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_test");
  return RUN_ALL_TESTS();
}