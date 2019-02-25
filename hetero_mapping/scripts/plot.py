#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt 
import matplotlib.pyplot as plt2  
import matplotlib.pyplot as grid 
def talker():
 
	rospy.init_node('graph', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	text_file = open("/home/roblab/hetero_ws/src/hetero_mapping/src/distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/roblab/hetero_ws/src/hetero_mapping/src/occupancy_volume.txt", "r")
	lines1 = text_file.read().split(',')
	text_file.close()
	plot_one=1
	x=[]
	y=[]
	x2=[]
	y2=[]
	i=2
	for i in range(len(lines)-1):
	   x.append(0)
	   y.append(0)
	i=2
	for i in range(len(lines)-1):
	   x2.append(0)
	   y2.append(0)
	i=1
	for i in range(len(lines)-1):
	   x[i-1]=i-1
	   y[i]=str(float(lines[i]))
	   
	   i +=1

	i=2
	for i in range(len(lines1)-1):
	   x2[i-1]=i-1
	   y2[i]=str(float(lines1[i]))
	   
	   i +=1

	plt.grid(color='r', linestyle='--', linewidth=1 )
   
	# plotting the points 

	if(plot_one==1):
		plt.plot(x, y,'o-') 
		plt.xlabel('Sample Number ') 
		plt.ylabel('RMSD Values ') 
		plt.title('UAV Horizontal Cloud Performance Metrics') 
		#Hetero Team Cloud Performance Metrics
		plt.show() 
	else:
		plt2.plot(x2, y2,'o-') 
		plt2.xlabel('Sample Number ') 
		plt2.ylabel('Occupancy in Max Volume % ') 
		plt2.title('UAV Horizontal  Cloud  Occupancy Metrics')
		plt2.show() 

	  

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass