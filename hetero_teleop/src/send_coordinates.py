#!/usr/bin/env python
# 2018 URC subs  waypoint controller "manuel"
# ITU Rover Team
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

point =[ [41.1053558192,29.0235311762], [41.1053331777, 29.0237283253] , [41.1052168137,29.023746603],[41.1051709292,29.0235311272] ]
status=String()
x=0
def waypointSub(msg):
	global status
	status=msg.data

def wait_result():
	global x , status
	while(status !="succes"):
		a=5
		rospy.Subscriber('/ugv_gps/waypoint/succes',String, waypointSub)

	if(status == "succes"):
		x=x+1
		status=" "
		print(x)
		do_all()


def do_all():
	global x, point
	waypoint=NavSatFix()
	Pub = rospy.Publisher('/rover_gps/waypoint', NavSatFix, queue_size=10)
	lat=point[x][0]
	lon=point[x][1]
	waypoint.latitude=float(lon)
	waypoint.longitude=float(lat)
	Pub.publish(waypoint)
	wait_result()


def main():
	global point
	rospy.init_node('ugv_waypoint')
	waypoint=NavSatFix()
	Pub = rospy.Publisher('/rover_gps/waypoint', NavSatFix, queue_size=10)
	while not rospy.is_shutdown():
		rospy.Subscriber('/ugv_gps/waypoint/succes',String, waypointSub)
		print "enter waypoint w1 w2 w3 w4 full "
		wp=raw_input() 
		if (wp == '\x03'):
			break
		if (wp=='full'):
			do_all()

		print("sending to "+str(wp))
		if (wp =='w1'):
			lat=point[0][0]
			lon=point[0][1]
		if (wp =='w2'):
			lat=point[1][0]
			lon=point[1][1]
		if (wp =='w3'):
			lat=point[2][0]
			lon=point[2][1]
		if (wp =='w4'):
			lat=point[3][0]
			lon=point[3][1]
		
		waypoint.latitude=float(lon)
		waypoint.longitude=float(lat)
		Pub.publish(waypoint)
	rospy.spin()


if __name__ == '__main__':
	main()
	rospy.spin()
