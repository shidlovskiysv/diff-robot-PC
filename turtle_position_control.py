#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from gazebo_msgs.msg import ModelStates
import numpy as np
from tf.transformations import euler_from_quaternion

rospy.init_node("control_node")
rate = rospy.Rate(10) 
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

def callback(data):
    my_comand=Twist()
    Yd=15
    Xd=6
    i=1
    x=data.pose[i].position.x
    y=data.pose[i].position.y
    qx=data.pose[i].orientation.x
    qy=data.pose[i].orientation.y
    qz=data.pose[i].orientation.z
    qw=data.pose[i].orientation.w
    euler=euler_from_quaternion((qx,qy,qz,qw))
    yaw=euler[2]
    Ex=Xd-x
    Ey=Yd-y
    
    thettaD=np.arctan2(Ey,Ex)

    contr_a=0.5*(thettaD-yaw)
    contr_v=0.1*np.sqrt((Ex**2)+(Ey**2))
    my_comand.linear.x=contr_v
    my_comand.angular.z=contr_a
    pub.publish(my_comand)
    #print(contr_v)

sub = rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
rospy.spin()