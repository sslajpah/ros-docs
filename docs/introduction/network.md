# ROS network

- one ROS master in the entire network
- all nodes must use the same ROS master (`ROS_MASTER_URI`)
- two-way connection between devices
- each device must present itself with a name that other devices recognize

[http://wiki.ros.org/ROS/Tutorials/MultipleMachines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)

[http://wiki.ros.org/ROS/NetworkSetup](http://wiki.ros.org/ROS/NetworkSetup)


## Connection

`ping` the remote computer (hostname: IP or name)
```
ping 192.168.65.60
```

Use `ssh` to connect to remote computer and ping your computer
```
ssh student@192.168.65.60
ping 192.168.65.50
```

## ROS_MASTER_URI

ROS master is set with variable `ROS_MASTER_URI`. It needs to be set withing each terminal.
```
export ROS_MASTER_URI=http://[hostname]:11311
```
To check, use
```
echo $ROS_MASTER_URI
```

Set parameter `ROS_IP` for IP or `ROS_HOSTNAME`for hostname if you have multiple addresses for a computer and need to force ROS to a particular one.


To do this automatically, add command to `bashrc`
```
sudo nano ~/.bashrc
```
Add
```
export ROS_MASTER_URI=http://[hostname]:11311
```

ATTENTION!

This can cause a problem, it you would like to run ROS master on the local computer.

## Test connection

It is important to test connection in both ways:

- master (publisher) >>> others (subscriber)
- master (subscribers) <<< others (publisher) 

Publisher:
```
rostopic pub /test_connection std_msgs/Bool "data: True"
```

Subscriber:
```
rostopic list

rostopic echo /test_connection
```

## Exercise

Connect to master computer with Sick Nanoscan3 and connect to topic `/sick_safetyscanners/scan`.

