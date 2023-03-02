# Packages

Packages are independent units, that can be re-used.

## New package

Go to `catkin_ws/src/` folder.

Basic syntax:
```
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

Create new package `rpi_test`:

```
cd ~/catkin_ws/src
catkin_create_pkg rpi_test rospy std_msgs
cd ..
catkin_make
```