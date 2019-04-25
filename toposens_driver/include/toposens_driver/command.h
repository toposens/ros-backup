/**  @file     command.h
  *  @author   Adi Singh, Christopher Lang
  *  @date     March 2019
  */

#ifndef COMMANDS_H
#define COMMANDS_H

#include <toposens_driver/TsDriverConfig.h>

namespace toposens_driver
{

/** @brief Generates firmware-compatible commands for tuning
 *  performance parameters.
 *
 *  @details Firmware defines commands in two formats, singular
 *  and dimensional. Singular commands expect one parameter value,
 *  dimensional commands expect XYZ limits defining a spatial cuboid.
 *  Currently, only the voxel filtering command is dimensionally formatted.
 *
 *  Note that all desired values are transmitted as zero-padded 5-byte
 *  strings, and the first byte is always reserved for the arithmetic
 *  sign: 0 for positive values, - for negative values.
 *
 *  Note that modifications on these parameters are done on the sensor chip, not in this driver
 */
class Command
{
  public:
    /** An arbitrary cuboid volume with points of interest and the sensor at its origin. */
    typedef TsDriverConfig::DEFAULT::VOXEL TsVoxel;

    /** Defined in TS firmware and exposed as public constants to
     *  be accessed by the #Command API. Ranges and default values
     *  are stated in the package's cfg file.     
     */
    enum Parameter
    {
      SigStrength  =0,  /** Number of waves emitted in every transmission cycle [0 to 20]. */
      FilterSize   =1,  /** Kernel size applied on ADC signals for edge detection [1 to 100]. */
      NoiseThresh  =2,  /** Minimum amplitude for an echo to be considered valid [0 to 20]. */
      VoxelLimits  =3,  /** 3D limits specifying boundaries of a volume of interest [0 to x-, y-, z-range]. */
      SNRBoostNear =4,  /** Short-range SNR booster for first third of x-range [0 to 1000]. */
      SNRBoostMid  =5,  /** Mid-range SNR booster for second third of x-range [0 to 1000]. */
      SNRBoostFar  =6,  /** Long-range SNR booster for last third of x-range [0 to 1000]. */
      CalibTemp    =7   /** Ambient temperature that sensor is calibrated to */
    };

    /** Empty constructor allowing subsequent manual generation of command. */
  //  Command(){};

    /** Builds a command message accepted by the TS firmware.
     *  @param param Setting name from the enumerated command list.
     *  @param value Desired integer value for sensor parameter.
     */
    Command(Parameter param, int value);

    /** Wrapper for composing a singular command message for a one-dimensional value space.
     *  @param param Setting name from the enumerated command list.
     *  @param value Desired integer value for sensor parameter
     */
  //  Command(Parameter param, int value) { generate(param, value); }

    /** Wrapper for composing a dimensional command message for a 3D voxel update.
     *  @param param Setting name from the enumerated command list.
     *  @param voxel Cuboidal limit ranges as [min, max] values for each dimension.
     */
  //  Command(Parameter param, TsVoxel voxel) { generate(param, voxel); }

    /** Creates a singular command message for a one-dimensional value space.
     *  Can be used to fill command data in an empty object or to manually edit
     *  parameter values of an existing object.
     *  @param param Setting name from the enumerated command list.
     *  @param value Desired integer value for sensor parameter.
     *  @returns True on successful command generation.
     */
  //  bool generate(Parameter param, int value);

    /** Creates a dimensional command message for a 3D voxel update.
     *  Can be used to fill command data in an empty object or to manually edit
     *  parameter values of an existing object.
     *  @param param Setting name from the enumerated command list.
     *  @param voxel Cuboidal limit ranges as [min, max] values for each dimension.
     *  @returns True on successful command generation.
     */
  //  bool generate(Parameter param, TsVoxel voxel);

    /** Returns the latest command message produced by generate().
     *  @returns Pointer to a char array containing command.
     */
    char* getBytes() { return _bytes; }


  private:
    const int MAX_VALUE =  9999;
    const int MIN_VALUE = -9999;

    char _bytes[50];   /**< Large enough buffer to hold a well-formed command.*/
    static const char kPrefix = 'C'; /**< Designates a string as a firmware command.*/

    /** Looks up command keys defined by the TS firmware corresponding to
     *  given setting parameters.
     *  @param param Setting name from the enumerated command list.
     *  @returns Corresponding firmware parameter key.
     */
    std::string _getKey(Parameter param);

    /**
     * Verifies that desired coordinate range values do not result in a digit overflow in the command bytes.
     * @param lower_val
     * @param upper_val
     * @return True if desired range is valid, i.e. does not result in an overflow.
     */
  //  bool _validate(int &lower_val, int &upper_val);

};

} // namespace toposens_driver

#endif
