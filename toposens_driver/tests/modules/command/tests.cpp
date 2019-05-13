/** @file     command_test.cpp
 *  @author   Christopher Lang, Roua Mokchah, Adi Singh
 *  @date     March 2019
 */
 
#include <gtest/gtest.h>
#include <toposens_driver/command.h>

using namespace toposens_driver;

class CommandTest : public ::testing::Test
{
public:
  const std::string TAG = "\033[36m[DriverCommandTest]\033[00m - ";

protected:
  void SetUp() {}

  void TearDown() {}

  void matchCmd(const char* msg, Command::Parameter p, int val, const char* exp)
  {
    std::cerr << TAG << "\t" << msg << "...";

    Command cmd(p, val);
    EXPECT_STREQ(cmd.getBytes(), exp);

    std::cerr << "done\n";
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
  std::cerr << TAG << "<validValues>\n";

  this->matchCmd("Signal strength with 5",   Command::SigStrength,  5,   "CnWave00005\r");
  this->matchCmd("Filter size with 42",      Command::FilterSize,   42,  "Cfiltr00042\r");
  this->matchCmd("Noise threshold with 15",  Command::NoiseThresh,  15,  "CdThre00015\r");
  this-> matchCmd("SNR boost with 100",      Command::SNRBoost,     100, "Cboost00100\r");
  this->matchCmd("Calibration temp with -3", Command::CalibTemp,    -3,  "CDTemp-0003\r");

  std::cerr << TAG << "</validValues>\n";
}


// @todo extend command interface for other new commands
/**
 * Testing the behavior of the Command constructor with invalid Inputs.
 * Desired behavior: Generated command bytes should consist of a fixed length and a valid param name.
 */
TEST_F(CommandTest, outOfRangeValues)
{
  std::cerr << TAG << "<outOfRangeValues>\n";

  this->matchCmd("Signal strength with -4678",   Command::SigStrength, -4678,   "CnWave-4678\r");
  this->matchCmd("Filter size with NULL",        Command::FilterSize,  NULL,    "Cfiltr00000\r");
  this->matchCmd("Noise threshold with 000000",  Command::NoiseThresh, 000000,  "CdThre00000\r");
  this->matchCmd("SNR boost with 00604",         Command::SNRBoost,    00604,   "Cboost00388\r");
  this->matchCmd("Noise threshold with INT_MAX", Command::CalibTemp,   INT_MAX, "CDTemp09999\r");

  std::cerr << TAG << "</outOfRangeValues>\n";
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_command_test");
  return RUN_ALL_TESTS();
}