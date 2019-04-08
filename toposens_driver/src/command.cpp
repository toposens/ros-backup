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

  if(!_isValidSingular(param, value)){
    ROS_WARN("Out of range value %i clipped to closest limit", value);
    value = (value < 0) ? 0 : 1000;
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

  if (!_isValidDimensional(param, voxel)) return false;

  std::string format = "%c%s%05d%05d%05d%05d%05d%05d\r";
  int n_bytes = std::sprintf(_bytes, format.c_str(), kPrefix, _getKey(param).c_str(),
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
  else if (param == Parameter::SNRBoostNear) return "slop1";
  else if (param == Parameter::SNRBoostMid)   return "slop2";
  else if (param == Parameter::SNRBoostFar)  return "slop3";
  else if (param == Parameter::CalibTemp)   return "DTemp";
  else /*VoxelLimit*/                       return "goLim";
}

/** Checks if desired parameter value are consistent with value limits specified in TsDriverConfig. */
bool Command::_isValidSingular(Parameter param, int value)
{

    TsDriverConfig minCfg = TsDriverConfig::__getMin__();
    TsDriverConfig maxCfg = TsDriverConfig::__getMax__();

    if      (param == Parameter::SigStrength) return ((value >= minCfg.sig_strength) && (value <= maxCfg.sig_strength));
    else if (param == Parameter::FilterSize)  return ((value >= minCfg.filter_size)  && (value <= maxCfg.filter_size));
    else if (param == Parameter::NoiseThresh) return ((value >= minCfg.noise_thresh) && (value <= maxCfg.noise_thresh));
    else if (param == Parameter::BoostShortR) return ((value >= minCfg.short_range)  && (value <= maxCfg.short_range));
    else if (param == Parameter::BoostMidR)   return ((value >= minCfg.mid_range)    && (value <= maxCfg.mid_range));
    else if (param == Parameter::BoostLongR)  return ((value >= minCfg.long_range)   && (value <= maxCfg.long_range));
    else /* Unkown singular parameter */      {
        ROS_ERROR("Dimensional command called with singular hand-over parameter. Skipping...");
        return false;
    }
}

/** Checks if desired voxel limits are consistent with value limits specified in TsDriverConfig. */
bool Command::_isValidDimensional(Parameter param, TsVoxel voxel)
{
    if(param == Parameter::VoxelLimits){

        TsDriverConfig minCfg = TsDriverConfig::__getMin__();
        TsDriverConfig maxCfg = TsDriverConfig::__getMax__();

        if (voxel.x_min <= voxel.x_max ||
            voxel.y_min <= voxel.y_max ||
            voxel.z_min <= voxel.z_max ) {
            ROS_ERROR("Voxel min value exceeds max value. Skipping command generation.");
            return false;
        }

        if ( (voxel.x_min <= minCfg.x_min) ||  (voxel.x_min >= maxCfg.x_min) ||
             (voxel.y_min <= minCfg.y_min) ||  (voxel.y_min >= maxCfg.y_min) ||
             (voxel.z_min <= minCfg.z_min) ||  (voxel.z_min >= maxCfg.z_min) ) {
            ROS_WARN("Voxel min limits clipped to feasible range.");
        }

        if ( (voxel.x_max >= minCfg.x_max) ||  (voxel.x_max >= maxCfg.x_max) ||
             (voxel.y_max >= minCfg.y_max) ||  (voxel.y_max >= maxCfg.y_max) ||
             (voxel.z_max >= minCfg.z_max) ||  (voxel.z_max >= maxCfg.z_max) ) {
            ROS_WARN("Voxel max limits clipped to feasible range.");
        }

        return true;
    }

    ROS_ERROR("Singluar command called with dimensional hand-over parameter. Skipping command generation.");
    return false;
}

} // namespace toposens_driver