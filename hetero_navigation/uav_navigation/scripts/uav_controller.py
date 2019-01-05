#!/usr/bin/env python
# -*- coding: utf-8 -*-



import rospy
import serial
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
 
z_current=0
twist=Twist()
odom=Odometry()
pub=rospy.Publisher("/uav/cmd_vel", Twist, queue_size=10)

def posecallback(data):
	global z_current, odom
	z_current = float(data.pose.position.z)
	odom.pose.pose=data.pose 
def stop():
	global twist, pub
	twist.linear.z=0
	twist.linear.x=0
	twist.angular.z=0
	pub.publish(twist)
	
def go_up():
	global twist, pub
	twist.linear.z=0.5
	twist.linear.x=0
	twist.angular.z=0
	pub.publish(twist)

def go_down():
	global twist, pub
	twist.linear.z=-0.5
	twist.linear.x=0
	twist.angular.z=0
	pub.publish(twist)

def letsSerial():
	rospy.init_node("uav_navigation_controller")
	rate = rospy.Rate(10) 
	do_once=0
	print_once=0
	global z_current,odom
	length=10
	pub_odom=rospy.Publisher("/uav/odometry_filtered", Odometry, queue_size=10)
	 

	while not rospy.is_shutdown():
	   odom.header.frame_id="world"
	   odom.header.stamp=rospy.Time.now()


	   rospy.Subscriber("/uav/ground_truth_to_tf/pose", PoseStamped, posecallback) 
	   if(do_once==0):
		   if(z_current >length and z_current<(length+0.5)):
			print("***Stopped!")
			do_once=1
			stop()
		   if(z_current<length):
			print("***Going up!")
			go_up()
		   if(z_current>(length+0.5)):
			print("***Going down!")
			go_down()    
	   elif(do_once==1):
		  if(print_once==0):
			print("***Congrats you can give point via move_base")
			print_once=1
		  elif(print_once==1):
			pub_odom.publish(odom)

	   rate.sleep()
	rospy.spin()




if __name__ == '__main__':
	try:
		letsSerial()
	except rospy.ROSInterruptException:
		pass