/** @file     command.cpp
 *  @author   Adi Singh, Christopher Lang
 *  @date     March 2019
 */

#include <toposens_driver/command.h>

namespace toposens_driver
{
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

 @todo ... document parameters and return values
 */
  bool Command::generate(Parameter param, int value)
  {
    memset(&_bytes, '\0', sizeof(_bytes));
    std::string format = "%c%s%05d\r";

    if(param == VoxelLimits){
      ROS_ERROR("Dimensional command called with singular hand-over parameter. Skipping...");
      return false;
    }

    if((value < MIN_VALUE) || (value > MAX_VALUE)){
      ROS_WARN("Out of range value %i clipped to closest limit", value);
      value = (value < MIN_VALUE) ? MIN_VALUE : MAX_VALUE;
    }


    int n_bytes = std::sprintf( _bytes, format.c_str(), kPrefix, _getKey(param).c_str(), value);
    return (n_bytes > 0);
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

  @todo ... document parameters and return values
 */
  bool Command::generate(Parameter param, TsVoxel voxel)
  {
    memset(&_bytes, '\0', sizeof(_bytes));

    if (param != VoxelLimits) {
      ROS_ERROR("Singluar command called with dimensional hand-over parameter. Skipping...");
      return false;
    }

    if (   !_validate(voxel.x_min, voxel.x_max)
           || !_validate(voxel.y_min, voxel.y_max)
           || !_validate(voxel.z_min, voxel.z_max)
            ) return false;


    std::string format = "%c%s%05d%05d%05d%05d%05d%05d\r";
    int n_bytes = std::sprintf(_bytes, format.c_str(),kPrefix, _getKey(param).c_str(),
                               voxel.x_min * 10, voxel.x_max * 10,
                               voxel.y_min * 10, voxel.y_max * 10,
                               voxel.z_min * 10, voxel.z_max * 10
    );

    return (n_bytes > 0);
  }


  /** Command keys are 5-byte strings hard-coded in the TS device firmware. */
  std::string Command::_getKey(Parameter param)
  {
    if      (param == Parameter::SigStrength) return "nWave";
    else if (param == Parameter::FilterSize)  return "filtr";
    else if (param == Parameter::NoiseThresh) return "dThre";
    else if (param == Parameter::SNRBoostNear)return "slop1";
    else if (param == Parameter::SNRBoostMid) return "slop2";
    else if (param == Parameter::SNRBoostFar) return "slop3";
    else if (param == Parameter::CalibTemp)   return "DTemp";
    else /*VoxelLimit*/                       return "goLim";
  }

  /** Verifies that desired coordinate range values do not result in a digit overflow in the command bytes. */
  bool Command::_validate(int &lower_val, int &upper_val)
  {
    int upper_limit = MAX_VALUE / 10;
    int lower_limit = MIN_VALUE / 10;

    if (upper_val > MAX_VALUE) {
      upper_val = upper_limit;
      ROS_WARN("Invalid dimensional value changed from %i to %d", upper_val, upper_limit);
    }

    if (lower_val > MIN_VALUE) {
      lower_val = lower_limit;
      ROS_WARN("Invalid dimensional value changed from %i to %d", lower_val, lower_limit);
    }

    if (upper_val <= lower_val) {
      ROS_ERROR("Invalid relative magnitudes. Skipping command generation.");
      return false;
    }

    return true;
  }



} // namespace toposens_driver
