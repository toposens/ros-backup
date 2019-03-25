/** @file     command_test.cpp
 *  @author   Christopher Lang, Roua Mokchah
 *  @date     March 2019
 */
 
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <toposens_driver/command.h>

namespace toposens_driver {


void generateAllCommands(std::list<Command> commands, int val){

  typedef TsDriverConfig::DEFAULT::VOXEL TsVoxel;
  TsVoxel vxl;
  vxl.x_max = val, vxl.x_min = val;
  vxl.y_max = val, vxl.y_min = val;
  vxl.z_max = val, vxl.z_min = val;

  commands.clear();

  commands.emplace_back(Command(Command::SigStrength, val));
  commands.emplace_back(Command(Command::FilterSize, val));
  commands.emplace_back(Command(Command::NoiseThresh, val));
  commands.emplace_back(Command(Command::BoostShortR, val));
  commands.emplace_back(Command(Command::BoostMidR, val));
  commands.emplace_back(Command(Command::BoostLongR, val));
  commands.emplace_back(Command(Command::VoxelLimits, vxl));

  // Unexpected behaviors
  commands.emplace_back(Command(Command::VoxelLimits, val));
  commands.emplace_back(Command(Command::SigStrength, vxl));
};

  /**
   * Unary predicate to test where a char is a digit or a minus sign.
   * @param c character
   * @return if != 0 character is a digit or minus sign
   */
  bool isNumber(int c){ return (isdigit(c) + (c == (int)'-')); }


  /**
   *  Tests if command message is well formed.
   *  Desired behavior: Command message is of lenght 12 Byte for singular commands and of 37 Byte length for
   *  dimensional commands. The message consists of a prefix byte, 5 byte to indicate the paramter to be changed
   *  by the command, and the following Bytes represented signed integeters.
   */
  TEST(CommandTest, generateWellFormed){

    std::list<Command> commands;

    generateAllCommands(commands, 2225);

    for(Command cmd : commands){
      EXPECT_TRUE((strlen(cmd.getBytes()) == 0) || (strlen(cmd.getBytes()) == 12)||
                  (strlen(cmd.getBytes()) == 37))
                  << "Command " << cmd.getBytes() << " has bad length!";

      EXPECT_TRUE(std::all_of(cmd.getBytes()+6,
          cmd.getBytes()+strlen(cmd.getBytes())-2,
          isNumber))
                  << "Command " << cmd.getBytes() << " has non-number characters encoded as values!";
    }
  }


  /**
   * Testing the behavior of generate for invalid Inputs
   * Desired behavior: Generated command bytes should consist of a fixed length and a valid param name.
   */
  TEST(CommandTest, generateInvalidInputs) {
    std::list<Command> commands;

    // only value between 0 and 9999 works for singular command
    // 5 bytes : 1 byte sign + 4 digits for the value
    generateAllCommands(commands, 9999);

    for (Command cmd : commands) {
      // length of the singular Command is 12 and for the dimensional one is 37, otherwise 0 if its empty
      EXPECT_TRUE((strlen(cmd.getBytes()) == 0) ||
                  (strlen(cmd.getBytes()) == 12) ||
                  (strlen(cmd.getBytes()) == 37))
                << "Value overflow in command " << cmd.getBytes();
    }
  }

} // namespace toposens_driver

int main(int argc, char **argv) {

  ros::init(argc, argv, "TsDriverCommandTestNode");
  testing::InitGoogleTest(&argc, argv);

  ros::start();

  auto res = RUN_ALL_TESTS();

  ros::shutdown();

  return res;

}