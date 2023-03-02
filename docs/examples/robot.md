# Robot control

Example python class for controlling serial robot via Cartesian velocity control (twist control) with included force/torque measurements (wrench).


## Code

```python linenums="1" title="robot_control.py"
#!/usr/bin/env python

# example class to control UR5e robot
#
# Sebastjan Slajpah @ Robolab, 2022
#

import rospy
from geometry_msgs.msg import Twist, Vector3, WrenchStamped


class RobotControl():

    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)

        cmd_vel_topic='/twist_controller/command'
        self._check_wrench_ready()

        # start the publisher
        self.vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.cmd = Twist()        

        self.wrench_subscriber = rospy.Subscriber('/wrench', WrenchStamped, self.wrench_callback)
        
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    
    def _check_wrench_ready(self):
        self.wrench_msg = None
        rospy.loginfo("Checking wrench ...")
        while self.wrench_msg is None and not rospy.is_shutdown():
            try:
                self.wrench_msg = rospy.wait_for_message("/wrench", WrenchStamped, timeout=1.0)
                rospy.logdebug("Current /wrench READY=>" + str(self.wrench_msg))

            except:
                rospy.logerr("Current /wrench not ready yet, retrying for getting scan")
        rospy.loginfo("Checking /wrench...DONE")
        return self.wrench_msg

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop_robot()
        self.ctrl_c = True

    def wrench_callback(self, msg):
        self.wrench_msg = msg

    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear = Vector3(0.0, 0.0, 0.0)
        self.cmd.angular = Vector3(0.0, 0.0, 0.0)
        self.publish_once_in_cmd_vel()

    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0.05
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "up":
            self.cmd.linear.z = speed
        elif motion == "down":
            self.cmd.linear.z = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()
            print(self.wrench_msg.forces.z)

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        return s


if __name__ == '__main__':
    
    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.move_straight_time('up',0.0, 3) # objekt robotcontrol_object.move_straight_time('up',0.0, 3) direction' up' speed 0.o  3=time (s)
        
    except rospy.ROSInterruptException:
        pass

```

File is available at [GitHub](https://github.com/sslajpah/ros-docs/blob/main/scripts/robot_control.py).