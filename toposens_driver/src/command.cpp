#include <toposens_driver/command.h>

namespace toposens_driver
{

  const char Command::SigStrength[6]   = "nWave"; //<
  const char Command::FilterSize[6]    = "filtr"; //<
  const char Command::NoiseThresh[6]   = "dThre"; //<
  const char Command::VoxelLimits[6]   = "goLim"; //< min/max ranges for cuboid's limits
  const char Command::BoostShortR[6]   = "slop1"; //<
  const char Command::BoostMidR[6]     = "slop2"; //<
  const char Command::BoostLongR[6]    = "slop3"; //<

/** Generates a singular command message for a one-dimensional value space.
 *
 *  Note that all desired values are transmitted as zero-padded 5-byte
 *  strings, and the first byte is always reserved for the arithmetic
 *  sign: 0 for positive values, - for negative values.
 *
 *  Singular format:
 *  @n - Starts with char 'C'
 *  @n - 5 bytes defining firmware parameter to update
 *  @n - 5 bytes with desired parameter value
 *  @n - Terminating char '\r'
 *
 */
bool Command::generate(const char* param, int val)
{
  std::string format = "%c%s%05d\r";
  return (std::sprintf( _bytes, format.c_str(), _prefix, param, val) > 0);
}

/** Generates a dimensional command message for a voxel update.
  *
  * Dimensional parameter values should be transformed to the TS-frame
  * before transmission. The transform from TS- to ROS-frame is
  * given by T(rt) = [(0, 0, 1), (-1, 0, 0), (0, 1, 0)].
  *
  * Note that all desired values are transmitted as zero-padded 5-byte
  * strings, and the first byte is always reserved for the arithmetic
  * sign: 0 for positive values, - for negative values.

  * Dimensional format:
  *  @n - Starts with char 'C'
  *  @n - 5 bytes defining firmware parameter to update
  *  @n - 5 bytes with cuboid's lower X-limit
  *  @n - 5 bytes with cuboid's upper X-limit
  *  @n - 5 bytes with cuboid's lower Y-limit
  *  @n - 5 bytes with cuboid's upper Y-limit
  *  @n - 5 bytes with cuboid's lower Z-limit
  *  @n - 5 bytes with cuboid's upper Z-limit
  *  @n - Terminating char '\r'
  */
bool Command::generate(const char* param, TsVoxel vxl)
{
  std::string format = "%c%s%05d%05d%05d%05d%05d%05d\r";
  return (std::sprintf(_bytes, format.c_str(), _prefix, param,
                      vxl.y_max * -10, vxl.y_min * -10,
                      vxl.z_min * 10, vxl.z_max * 10,
                      vxl.x_min * 10, vxl.x_max * 10 ) > 0);   // ros --> ts :: x,y,z --> -y,z,x
}

/** Creates well-formed TS-defined settings command and returns pointer to it.
  */
char* Command::getBytes(){
  return _bytes;
}

} // namespace toposens_driver