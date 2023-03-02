# ROS setup

## roscore

- takes care of communicaiton between different ROS functionalities
- it can run only one at once
- connects different ROS system into one ROS network

```python
roscore
```

## catkin workspace

CATKIN is an official build system for ROS

```
cd
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
```

Alternative is to use `catkin build` (beforehand you need to remove devel and build folders).

Conect console with ROS variables
```
cd devel
source setup.bash
```

To do this automatically when the console is opened, add to `bashrc.sh`
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
