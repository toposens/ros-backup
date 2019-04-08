## How to run the Tests 

```
catki_make --make-args run_tests 
```

## Description of the tests in the ros driver package

# 1. Unit Tests 

* Serial Test: 

Testing the behavior of the serial constructor with invalid port devices 
(null port device and non-existent port device) and with valid mock ports.

Testing the behavior of the getFrame method with invalid data frame format.

* Sensor Test: 
 
Testing the behavior of the parse method with valid, invalid and large 
data frames.

Testing the sensor shutdown method . 

* Command Test: 

Testing the behavior of the generate method with invalid Inputs ( invalid 
integer value for the singular command message and invalid voxel limit 
range values  for the dimensional command message )

# 2. Integration Tests

Testing that changes in sensor parameters (noise Threshold, Signal Strength
and VoxelLimits) are being applied.
