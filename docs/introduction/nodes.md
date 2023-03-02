# Node
Nodes are processes:

- used for calculations,
- that run inside robotic application,
- grouped into packages,
- that communicate with each other (topics, servers, actions, parameter servers). 

Why to use nodes:

- reduce code complexity,
- the code is more error-resistant,
- use of different programming languages.

## New node

Nodes are scripts that are located inside `scripts` folder.
```
roscd rpi_test
mkdir scripts
cd scripts
```
Create new script `my_first_node.py`:
```
touch my_first_node.py
```

Change the access permissions of the file to executable:
```
chmod +x my_first_node.py
```

Open script with Visual Studio Code:
```
code my_first_node.py
```

The minimal working example for node:

```python linenums="1" title="my_fist_node.py"
#!/usr/bin/env python

import rospy

if __name__ == '__main__':
  rospy.init_node('my_first_python_node')
  rospy.loginfo('This node has been started.')
  rospy.sleep(1)
  print('Exit now')
```

Only one node with specific name can be run at a time. If you want to run more instances of the same node, change:
```python linenums="6"
  rospy.init_node('my_first_python_node', anonymous=True)
```

To run, write
```
python my_first_node.py
```

## New node - class

```python linenums="1" title="my_fist_node_class.py"
#!/usr/bin/env python

import rospy

class hello_world():
    
  def __init__(self):
    # init variables
    self.delay = 5
    self.ctrl_c = False
    rospy.on_shutdown(self.shutdownhook)
        
  def shutdownhook(self):
    # works better than the rospy.is_shutdown()
    # this code is run at ctrl + c
    rospy.loginfo('This node has been terminated.')
    self.ctrl_c = True
  
  def test_node(self): 
    rospy.loginfo('This node has been started.')
    rospy.sleep(self.delay)
    print('Exit now')
            
if __name__ == '__main__':
  # initialise node
  rospy.init_node('my_first_python_node')
  # initialise class
  first_node = hello_world()
  try:
    first_node.test_node()
  except rospy.ROSInterruptException:
    pass
```


## DEBUG

- `rosrun <pkg name> <node name>` - run specific node
- `rosnode list` - list of all active nodes
- `rosnode info <node name>` - information about hte node
- `rosnode kill <node name>` - shut down node
- `rosnode ping <node name>` - ping node (check, if it is working)

