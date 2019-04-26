## How to run the Tests

```
catkin_make run_tests
```

## Description of the tests in the ros markers package


* Plotting sequence:


Testing Markers::_plot .

Checks that the published TsScans are plotted in the order they are received.

Checks that an empty TsScan don't generate any points.


* Hztest:


Checks 10 Hz publishing rate of ts_markers_node.


* Publishtest:


Checks that ts_markers topic is operative.

> The last two tests are using a bag file. That's why we don't need to plug the sensor anymore.

The bag file will be played back in ROS to the same topic it was recorded from (/ts_scans) allowing

the ts_markers_node to subscribe to it and publish on the ts_markers topic.

If you want to get new data in your bag file run :

in Terminal 1 :

```
roslaunch toposens_driver toposens_driver.launch
```

in Terminal 2 :

```
rosbag record -O <path to your bag file.bag> /ts_scans
```

