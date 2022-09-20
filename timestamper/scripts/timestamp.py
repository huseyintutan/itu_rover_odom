#!/usr/bin/env python

# @author huseyintutan

import rospy
from std_msgs.msg import Float64MultiArray
import math 
from sensor_msgs.msg import Joy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import sin, cos

class SubWithTimer:
    def __init__(self):
        self.L = 0  # LEFT WHEEL RPM
        self.R = 0# RIGHT WHEEL RPM
        self.RR = 0 # RIGHT WHEEL VELOCITY
        self.LL = 0# LEFT WHELL VELOCITY
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.subA = rospy.Subscriber('drive_system_left_motors_feedbacks', Float64MultiArray, self.callbackA)
        self.subB = rospy.Subscriber('drive_system_right_motors_feedbacks', Float64MultiArray, self.callbackB)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback) # ROSBAG TOPIC HZ =5

    def callbackA(self, msg): # LEFT WHEEL CALLBACK
        self.L = msg.data
        self.LL=(((self.L[0]+self.L[1])/2)*2*(math.pi)*0.135)/60 # r = 135mm
        #print(self.LL)

    def callbackB(self, msg):   # RIGHT WHEEL CALLBACK
        self.R = msg.data
        self.RR=(((self.R[0]+self.R[1])/2)*2*(math.pi)*0.135)/60 # r = 135mm
        #print(self.RR)

    def timer_callback(self, event):
        
        odom_broadcaster = tf.TransformBroadcaster()

        pubTimer = rospy.Publisher('odom', Odometry, queue_size=10)

        # DIFF DRIVE CALCULATIONS

        L = 0.89 # WHEEL BASE 890mm

        vx = (self.RR + self.LL)/2
        vy = 0
        vth = (self.RR - self.LL)/L
        
        self.current_time = rospy.Time.now()


        dt = (self.current_time - self.last_time).to_sec()
        delta_x = (vx * cos(self.th) ) * dt
        delta_y = (vx * sin(self.th) ) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        print(odom_quat)

        odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )
                
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat)) # z = 0

        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        pubTimer.publish(odom)

        self.last_time = self.current_time

if __name__ == '__main__':
    rospy.init_node("timestamp", anonymous=True)
    SubWithTimer()
    rospy.spin()


  