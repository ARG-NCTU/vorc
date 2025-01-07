#!/usr/bin/env python
# license removed for brevity

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class Node():
    def __init__(self,linear_scaling,angular_scaling,keyboard=False):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling
        self.left_pub = None
        self.right_pub = None
        self.left_msg =Float32()
        self.right_msg =Float32()
        self.keyboard = keyboard
        self.auto = 1
        
        # Publisher
        self.left_pub = rospy.Publisher("left_cmd",Float32,queue_size=10)
        self.right_pub = rospy.Publisher("right_cmd",Float32,queue_size=10)

        # Subscriber
        self.sub_cmd = rospy.Subscriber("cmd_vel", Twist, self.cb_cmd, queue_size=1)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.cbJoy, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_publish)

    def cb_publish(self, event):
        self.left_pub.publish(self.left_msg)
        self.right_pub.publish(self.right_msg)
    def cb_cmd(self, data):
        print(data)
        if self.auto:
            self.t2t(data.linear.x, data.angular.z)
            # self.left_msg.data = data.linear.x
            # self.right_msg.data = data.linear.x
            # self.left_ang_msg.data = data.angular.z
            # self.right_ang_msg.data = -1*data.angular.z
    
    def cbJoy(self,data):
        if(data.buttons[7]==1) and not self.auto:
            self.auto = 1
            rospy.loginfo("going auto")
        elif(data.buttons[6]==1) and self.auto:
            self.auto = 0
            rospy.loginfo("going manual")

        if not self.auto:
            self.t2t(data.axes[1], data.axes[3])
    
    def t2t(self, x, z):
        left_pos = np.array([-6.103, 0.911])
        right_pos = np.array([-6.103, -0.911])
        lever_arm = np.linalg.norm(left_pos - right_pos) / 2
        angular_contribution = z * lever_arm
        left_thrust = x - angular_contribution
        right_thrust = x + angular_contribution
        left_thrust = np.clip(x - angular_contribution, -1, 1)
        right_thrust = np.clip(x + angular_contribution, -1, 1)
        self.left_msg.data = left_thrust
        self.right_msg.data = right_thrust

if __name__ == '__main__':

    rospy.init_node('twist2drive', anonymous=True)

    # ROS Parameters
    # Scaling from Twist.linear.x to (left+right)
    linear_scaling = rospy.get_param('~linear_scaling',1)
    # Scaling from Twist.angular.z to (right-left)
    angular_scaling = rospy.get_param('~angular_scaling',1)

    rospy.loginfo("Linear scaling=%f, Angular scaling=%f"%(linear_scaling,angular_scaling))


    key = '--keyboard' in sys.argv
    node=Node(linear_scaling,angular_scaling,keyboard=key)


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
