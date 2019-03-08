/** @file     command.cpp
 *  @author   Adi Singh, Christopher Lang
 *  @date     March 2019
 */

#include <toposens_driver/command.h>

namespace toposens_driver
{
  /** add relevant comment here. index refers to enum order/Cfg level 
  /** should retain order as it corresponds to enum */

/** Generates a singular command message for a one-dimensional value space.
 *
 *  Note that all desired values are transmitted as zero-padded 5-byte
 *  strings, and the first byte is always reserved for the arithmetic
 *  sign: 0 for positive values, - for negative values.
 *
 *  Singular format:
 *  @n - Starts with #kPrefix
 *  @n - 5 bytes defining firmware parameter to update
 *  @n - 5 bytes with desired parameter value
 *  @n - Terminating carriage return
 */
<<<<<<< toposens_driver/src/command.cpp
bool Command::generate(Parameter param, int value)
{
  std::string format = "%c%s%05d\r";
  return (std::sprintf( _bytes, format.c_str(), kPrefix, _getKey(param), value) > 0);
}

/** Generates a dimensional command message for a voxel update.
 *
 * All desired values are transmitted as zero-padded 5-byte strings.
 * The first byte is always reserved for the arithmetic sign: 0 for
 * positive values, and - for negative values.
 *
 * Dimensional format:
 *  @n - Starts with #kPrefix
 *  @n - 5 bytes defining firmware parameter to update
 *  @n - 5 bytes with cuboid's lower X-limit
 *  @n - 5 bytes with cuboid's upper X-limit
 *  @n - 5 bytes with cuboid's lower Y-limit
 *  @n - 5 bytes with cuboid's upper Y-limit
 *  @n - 5 bytes with cuboid's lower Z-limit
 *  @n - 5 bytes with cuboid's upper Z-limit
 *  @n - Terminating carriage return
 */
bool Command::generate(Parameter param, TsVoxel voxel)
{
  std::string format = "%c%s%05d%05d%05d%05d%05d%05d\r";
  return (std::sprintf(_bytes, format.c_str(), kPrefix, _getKey(param),
                        voxel.x_min * 10, voxel.x_min * 10,
                        voxel.y_min * 10, voxel.y_max * 10,
                        voxel.z_min * 10, voxel.z_max * 10
                      ) > 0);
}

/**
 * Looks up key defined by the TS firmware for every parameter
 */
char* Command::_getKey(Parameter param){
  if     (param == Parameter::SigStrength)return "nWave";
  else if(param == Parameter::FilterSize) return "filtr";
  else if(param == Parameter::NoiseThresh)return "dThre";
  else if(param == Parameter::BoostShortR)return "slop1";
  else if(param == Parameter::BoostMidR)  return "slop2";
  else if(param == Parameter::BoostLongR) return "slop3";
  else /*VoxelLimit*/ return "goLim";
}

} // namespace toposens_driver