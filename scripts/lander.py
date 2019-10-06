#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy, time
from math import atan2, sqrt
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

flag_land = False

kpx = 0.1
kpy = 0.1
kpz = 0.15
kp_psi = 0.8

kdx = 0.1
kdy = 0.1
kdz = 0.0
kd_psi = 0.1

vx = 0.0
vy = 0.0
vz = 0.0

rdius = 0.1

ctrl = Twist()
pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
pub_to = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)
pub_l = rospy.Publisher('bebop/land', Empty, queue_size=1)

def lander():
    
    rospy.init_node('lander', anonymous=True)
    time.sleep(2.0)
    pub_to.publish()
    time.sleep(5.0)
    rospy.Subscriber('bebop/odom', Odometry, callback_odom)
    rospy.Subscriber('aruco_single/pose', PoseStamped, callback)
    rate = rospy.Rate(20) # 10hz
    rospy.spin()

def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + '  x:%f, y:%f, z:%f', data.pose.position.x, data.pose.position.y, data.pose.position.z)
#    ctrl.linear.x = kpx * data.pose.position.x
#    ctrl.linear.y = kpy * data.pose.position.y
#    ctrl.linear.z = 0.0

#    q0 = data.pose.orientation.w
#    q1 = data.pose.orientation.x
#    q2 = data.pose.orientation.y
#    q3 = data.pose.orientation.z
#    psi = atan2(2*(q0*q3 + q1*q2), 1-2*(q2**2 + q3**2))
#    ctrl.angular.x = 0.0
#    ctrl.angular.y = 0.0
#    ctrl.angular.z = kp_psi * psi
    
    
    if(data.pose.position.y**2+data.pose.position.x**2 < rdius**2):
        pub_l.publish()

    ctrl.linear.x = -kpx * data.pose.position.y - kdx * vx
    ctrl.linear.y = -kpy * data.pose.position.x - kdx * vy
    ctrl.linear.z = 0.0

    q0 = data.pose.orientation.w
    q1 = data.pose.orientation.x
    q2 = data.pose.orientation.y
    q3 = data.pose.orientation.z
    psi = atan2(2*(q0*q3 + q1*q2), 1-2*(q2**2 + q3**2))

    rospy.loginfo('%f, %f, %f',data.pose.position.y,data.pose.position.x, psi*180.0/3.14)

    err_psi = (psi-1.57)

    ctrl.angular.x = 0.0
    ctrl.angular.y = 0.0
    ctrl.angular.z = - kp_psi * err_psi

    if(err_psi>0.017 or err_psi<-0.017):
    	ctrl.linear.x = 0.0
    	ctrl.linear.y = 0.0
    	ctrl.linear.z = 0.0

    if(flag_land==False):
     	pub.publish(ctrl)

def callback_odom(data):
    global vx, vy, vz
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z

def callback_land(data):
     global flag_land
     flag_land = True

if __name__ == '__main__':
    try:
        lander()
    except rospy.ROSInterruptException:
        pass
