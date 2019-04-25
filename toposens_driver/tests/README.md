## How to run the Tests 

```
catkin_make run_tests
```

## Description of the tests in the ros driver package

* Serial Test: 

Testing the behavior of the serial port constructor with invalid mock port devices
(null port device and non-existent port device) and with valid ones.

Testing Serial::getFrame with well formatted TS data frame format.

Testing Serial::send using two interconnected mock ports.

* Sensor Test:
 
Testing Sensor::poll with valid, large and empty data frames.

Tests for handling invalid data frames.

Testing Sensor::shutdown : Shutting down the sensor should allow to start another sensor application.

Testing Sensor::poll using dumped data from a file.

To get new data in the dump file make sure that the sensor is plugged in.

> Run : 

```
catkin_make -DPLUGGED_IN=true run_tests
```

* Command Test: 

Testing the behavior of the Command constructor with valid and invalid parameters.

Testing if the generated command message is well formed.


* Reconfigure Test:

Testing that changes in sensor parameters are being applied.


* Hz Test:

Checks 10 Hz publishing rate of ts_driver_node. 


