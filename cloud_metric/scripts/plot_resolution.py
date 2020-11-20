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
	string5="resolution_0_3"
	string4="resolution_0_2"
	string3="resolution_0_1"
	string2="resolution_0_0_5"
	string1="resolution_0_0_1"
	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string1+"/o_r/total_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string1+"/o_r/total_occupancy_volume.txt", "r")
	lines1 = text_file.read().split(',')
	text_file.close()
	plot_one=2
	bound=30
	x=[]
	y=[]
	x2=[]
	y2=[]
	i=1
	for i in range(1,(len(lines)-bound),20):
	   x.append(i-1)
	   y.append(str(float(lines[i])))   
	   i +=1	
	i=1
	for i in range(1,(len(lines1)-bound),20):
	   x2.append(i-1)
	   y2.append(str(float(lines1[i])))   
	   i +=1
	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string2+"/o_r/total_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string2+"/o_r/total_occupancy_volume.txt", "r")
	lines1 = text_file.read().split(',')
	text_file.close()
	plot_one=2

	x3=[]
	y3=[]
	x4=[]
	y4=[]

	i=1
	for i in range(1,(len(lines)-bound),20):
	   x3.append(i-1)
	   y3.append(str(float(lines[i])))   
	   i +=1	
	i=1
	for i in range(1,(len(lines1)-bound),20):
	   x4.append(i-1)
	   y4.append(str(float(lines1[i])))   
	   i +=1

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string3+"/o_r/total_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string3+"/o_r/total_occupancy_volume.txt", "r")
	lines1 = text_file.read().split(',')
	text_file.close()
	plot_one=2

	x5=[]
	y5=[]
	x6=[]
	y6=[]
	i=1
	i=1
	for i in range(1,(len(lines)-bound),20):
	   x5.append(i-1)
	   y5.append(str(float(lines[i])))   
	   i +=1	
	i=1
	for i in range(1,(len(lines1)-bound),20):
	   x6.append(i-1)
	   y6.append(str(float(lines1[i])))   
	   i +=1
	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string4+"/o_r/total_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string4+"/o_r/total_occupancy_volume.txt", "r")
	lines1 = text_file.read().split(',')
	text_file.close()
 
	x7=[]
	y7=[]
	x8=[]
	y8=[]
	i=1
	for i in range(1,(len(lines)-bound),20):
	   x7.append(i-1)
	   y7.append(str(float(lines[i])))   
	   i +=1	
	i=1
	for i in range(1,(len(lines1)-bound),20):
	   x8.append(i-1)
	   y8.append(str(float(lines1[i])))   
	   i +=1
	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string5+"/o_r/total_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string5+"/o_r/total_occupancy_volume.txt", "r")
	lines1 = text_file.read().split(',')
	text_file.close()
 
	x9=[]
	y9=[]
	x10=[]
	y10=[]
	i=1
	for i in range(1,(len(lines)-bound),20):
	   x9.append(i-1)
	   y9.append(str(float(lines[i])))   
	   i +=1	
	i=1
	for i in range(1,(len(lines1)-bound),20):
	   x10.append(i-1)
	   y10.append(str(float(lines1[i])))   
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
	string1 = "Resolutions in Orthogonal and Regular"
	string2 = "UAV Orthogonal"
	string3 = "UAV Regular"
	string4 = "UGV Orthogonal"
	string5 = "UGV Regular"
	#Total 
	plt.subplot(2, 1, 1)
	plt.title(string1+' Configuration Performance  Metrics ') 
	plt.plot(x, y,color='green',marker='x') 
	plt.plot(x3, y3,color='red',marker='^') 
	plt.plot(x5, y5,color='blue',marker='s') 
	plt.plot(x7, y7,color='orange',marker='o') 
	plt.plot(x9, y9,color='black',marker='<') 
	plt.xlabel('Sample Number ') 
	plt.ylabel('RMS Values ') 
	plt.legend(["0.01","0.05","0.1","0.2","0.3"])
	plt.subplot(2, 1, 2)
	plt.title(string1+' Configuration  Occupancy Metrics ') 
	plt.plot(x2, y2,color='green',marker='x') 
	plt.plot(x4, y4,color='red',marker='^') 
	plt.plot(x6, y6,color='blue',marker='s')  
	plt.plot(x8, y8,color='orange',marker='o') 
	plt.plot(x10, y10,color='black',marker='<') 
	plt.xlabel('Sample Number ') 
	plt.ylabel('Occupancy Rate  % ') 

	plt.show() 

	
	
	
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
