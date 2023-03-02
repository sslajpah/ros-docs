# Roslaunch

Roslaunch is a tool for easily launching multiple ROS nodes as well as setting parameters. Roslaunch takes in one or more XML configuration files (with the .launch extension) that specify the parameters to set and nodes to launch, as well as the machines that they should be run on.

Basic sytax is
```xml
<launch>
  <param name="/parameter_name" type="variable_type" value="variable_value"/>
  <node name="name_of_the_node" pkg="name_of_the_package" type="source_file.py" ns="namespace" args="optional arguments"/>
  <include file="included.launch">
    <arg name="arg_name" value="arg_val" />
  </include>
</launch>
```
### New .launch

Create new package `rpi_feros_bringup` used for launch files:
```
roscd
cd ..
cd src
catkin_create_pkg rpi_feros_bringup
```
and do `catkin_make`.

Go the new package and create folder `launch`
```
roscd rpi_feros_bringup
mkdir launch
cd launch
```

Inside the folder create new file `feros.launch`
```
touch rpi_led.launch
code rpi_led.launch
```

Copy the following lines
```xml linenums="1" title="rpi_led.launch"
<?xml version="1.0"?>

<launch>
    <node name="btn_pub" pkg="rpi_feros" type="button_publisher.py" output="screen"/>
    <node name="led_act" pkg="rpi_feros" type="led_actuator.py" output="screen"/>   
</launch>
```

To run the `rpi_led.launch` use `roslaunch` function
```
roslaunch rpi_feros_bringup rpi_led.launch
```

By running this .launch file three things are started

- `roscore` (automatically),
- `button_publisher.py`, and
- `led_actuator.py`.

