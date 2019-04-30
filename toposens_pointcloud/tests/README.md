## How to run the Tests

```
catkin_make run_tests
```
 
## Description of the tests in the ros PointCloud package


* PointCloud Test:

Checks that each published TsScan is converted to a new PointCloud message of template XYZI.

Checks that an empty TsScan generates zero PointCloud messages.

* Hztest:


Checks 10 Hz publishing rate of ts_cloud_node.


* Publishtest:


Checks that ts_cloud topic is operative.


> The last two tests are using a bag file. That's why we don't need to plug the sensor anymore.


> If you want to run only the PointCloud test : 

```
catkin_make -DPLUGGED_IN=false run_tests
```