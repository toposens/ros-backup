/** @file     command.cpp
 *  @author   Adi Singh, Christopher Lang
 *  @date     March 2019
 */

#include <toposens_driver/command.h>

namespace toposens_driver
{

/** All desired values are transmitted as zero-padded 5-byte string.
 *  The first byte is always reserved for the arithmetic sign: 0 for
 *  positive values, - for negative values.
 *
 *  Command format:
 *  @n - Starts with #kPrefix
 *  @n - 5 bytes defining firmware parameter to update
 *  @n - 5 bytes with desired parameter value
 *  @n - Terminating carriage return

 @todo ... document parameters and return values
 */
Command::Command(Parameter param, int value)
{
  memset(&_bytes, '\0', sizeof(_bytes));
  std::string format = "%c%s%05d\r";
  
// Checks if desired parameter value are consistent with value limits
  // @todo refine comment
  if((value < MIN_VALUE) || (value > MAX_VALUE)){
    ROS_WARN("Out of range value %i clipped to closest limit", value);
    value = (value < MIN_VALUE) ? MIN_VALUE : MAX_VALUE;
  }

  std::sprintf( _bytes, format.c_str(), kPrefix, _getKey(param).c_str(), value);
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

} // namespace toposens_driver