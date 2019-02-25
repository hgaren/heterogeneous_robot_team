#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt 
import matplotlib.pyplot as plt2  
import matplotlib.pyplot as plt3  

import matplotlib.pyplot as grid 
def talker():
 
	rospy.init_node('graph', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	text_file = open("/home/roblab/hetero_ws/src/hetero_mapping/src/total_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/roblab/hetero_ws/src/hetero_mapping/src/total_occupancy_volume.txt", "r")
	lines1 = text_file.read().split(',')
	text_file.close()
	plot_one=2

	x=[]
	y=[]
	x2=[]
	y2=[]
	i=1
	for i in range(len(lines)-2):
	   x.append(0)
	   y.append(0)
	i=1
	for i in range(len(lines)-2):
	   x2.append(0)
	   y2.append(0)
	i=1
	for i in range(len(lines)-1):
	   x[i-1]=i-1
	   y[i-1]=str(float(lines[i]))
	   
	   i +=1
	
	i=1
	for i in range(len(lines1)-1):
	   x2[i-1]=i-1
	   y2[i-1]=str(float(lines1[i]))
	   
	   i +=1
	text_file = open("/home/roblab/hetero_ws/src/hetero_mapping/src/uav_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/roblab/hetero_ws/src/hetero_mapping/src/uav_occupancy_volume.txt", "r")
	lines1 = text_file.read().split(',')
	text_file.close()
	plot_one=2

	x3=[]
	y3=[]
	x4=[]
	y4=[]
	i=1
	for i in range(len(lines)-2):
	   x3.append(0)
	   y3.append(0)
	i=1
	for i in range(len(lines)-2):
	   x4.append(0)
	   y4.append(0)
	i=1
	for i in range(len(lines)-1):
	   x3[i-1]=i-1
	   y3[i-1]=str(float(lines[i]))
	   
	   i +=1
	
	i=1
	for i in range(len(lines1)-1):
	   x4[i-1]=i-1
	   y4[i-1]=str(float(lines1[i]))
	   
	   i +=1

	text_file = open("/home/roblab/hetero_ws/src/hetero_mapping/src/ugv_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/roblab/hetero_ws/src/hetero_mapping/src/ugv_occupancy_volume.txt", "r")
	lines1 = text_file.read().split(',')
	text_file.close()
	plot_one=2

	x5=[]
	y5=[]
	x6=[]
	y6=[]
	i=1
	for i in range(len(lines)-2):
	   x5.append(0)
	   y5.append(0)
	i=1
	for i in range(len(lines)-2):
	   x6.append(0)
	   y6.append(0)
	i=1
	for i in range(len(lines)-1):
	   x5[i-1]=i-1
	   y5[i-1]=str(float(lines[i]))
	   
	   i +=1
	
	i=1
	for i in range(len(lines1)-1):
	   x6[i-1]=i-1
	   y6[i-1]=str(float(lines1[i]))
	   
	   i +=1

	'''
	if x[50] !=0:
		print ("P:50:" +str(y[55]))
	if x[100] !=0:
		print ("P:100:" +str(y[110]))
	if x[200] !=0:
		print ("P:200:" +str(y[210]))
	if x[300] !=0:
		print ("P:300:" +str(y[320]))
	if x2[50] !=0:
		print ("O:50:" +str(y2[55]))
	if x2[100] !=0:
		print ("O:100:" +str(y2[110]))
	if x2[200] !=0:
		print ("O:200:" +str(y2[210]))
	if x2[300] !=0:
		print ("O:300:" +str(y2[320]))
	'''
	plt.grid(color='r', linestyle='--', linewidth=1 )
	
	# plotting the points 
	string1 = "Total Orthogonal and Orthogonal "
	string2 = "UAV Orthogonal"
	string3 = "UAV Regular"
	string4 = "UGV Orthogonal"
	string5 = "UGV Regular"
	#Total 
	plt.subplot(2, 1, 1)
	plt.title(string1+' Configuration Performance  Metrics') 
	plt.plot(x, y,'o-') 
	plt.xlabel('Sample Number ') 
	plt.ylabel('RMS Values ') 
		
	plt.subplot(2, 1, 2)
	plt.title(string1+' Configuration  Occupancy Metrics') 
	plt.plot(x2, y2,'o-') 
	plt.xlabel('Sample Number ') 
	plt.ylabel('Occupancy Rate  % ') 
	plt.show() 

	
	
	#UAV  
	plt2.grid(color='r', linestyle='--', linewidth=1 )
	plt2.subplot(2, 1, 1)
	plt2.title(string2+' Configuration Performance  Metrics') 
	plt2.plot(x3, y3,'o-') 
	plt2.xlabel('Sample Number ') 
	plt2.ylabel('RMS Values ') 
		
	plt2.subplot(2, 1, 2)
	plt2.title(string2+' Configuration Performance  Metrics') 
	plt2.plot(x4, y4,'o-') 
	plt2.xlabel('Sample Number ') 
	plt2.ylabel('Occupancy Rate  % ') 
	plt2.show() 
	  
	#UGV  
	plt3.grid(color='r', linestyle='--', linewidth=1 )
	plt3.subplot(2, 1, 1)
	plt3.title(string4+' Configuration Performance  Metrics') 
	plt3.plot(x5, y5,'o-') 
	plt3.xlabel('Sample Number ') 
	plt3.ylabel('RMS Values ') 
		
	plt3.subplot(2, 1, 2)
	plt3.title(string4+' Configuration Performance  Metrics') 
	plt3.plot(x6, y6,'o-') 
	plt3.xlabel('Sample Number ') 
	plt3.ylabel('Occupancy Rate  % ') 
	plt3.show() 
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
