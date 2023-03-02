# SICK NanoScan3

Manual - [wiki](http://wiki.ros.org/sick_safetyscanners)

- 1651 measurements
- angle resolution: 0.002909 rad
- scan angle: 275°

## Install ROS support

```
sudo apt-get install ros-melodic-sick-safetyscanners
```

## Run the driver
```
roslaunch sick_safetyscanners sick_safetyscanners.launch sensor_ip:=<sensor ip> host_ip:=<host ip>
```
## Published topics

Topic: `/sick_safetyscanners/scan`

Message type: `sensor_msgs/LaserScan`
