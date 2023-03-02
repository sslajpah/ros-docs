# Parameters

Parameter server: globally available dictionary within ROS master

ROS parameter: one variable within the parameter server 

Types:

- Boolean
- Int
- Double
- String
- Lists
- ...


## Example

Code examples:
```python linenums="1"
# set parameter
rospy.set_param('/publish_frequency', 2)

# get parameter
publish_freq = rospy.get_param('/publish_frequency')

# get list of parameters
try:
  rospy.get_param_names()
except ROSException:
  print("Could not get param names")
```

Set parameter in a launch file:
```xml
<param name="/parameter_name" type="parameter_type" value="value"/>
```

```xml linenums="1"
<launch>
  <param name="/publish_frequency" type="int" value="2"/>
</launch>
```

`<rosparam>` is used if you want to load a list of parameters from a .yaml file
```xml linenums="1"
<rosparam command="load" file="$(find <package_name>)/example.yaml" />
```
## DEBUG

- `rosparam set <param name> <value>` - to create new parameter
- `rosparam get <param name>` - get parameter value
- `rosparam list` - get list of parameters


## Exercise

Upgrade SimpleActionClient by setting number of runs as parameter `/number_of_runs`.

Upgrade SimpleActionServer by setting LED frequency as parameter `/led_frequency`.

Create `.launch` file for Action Server.
