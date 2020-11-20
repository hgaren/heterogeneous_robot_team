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
	string="resolution_0_0_5"
	string1="resolution_0_0_1"

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string1+"/o_r/total_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string1+"/o_r/total_occupancy_volume.txt", "r")
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
	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string1+"/o_o/total_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string1+"/o_o/total_occupancy_volume.txt", "r")
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

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string1+"/r_r/total_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string1+"/r_r/total_occupancy_volume.txt", "r")
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
	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string1+"/r_o/total_distance_av.txt", "r")
	lines = text_file.read().split(',')
	text_file.close()

	text_file = open("/home/hgaren/hetero_ws/src/hetero_mapping/src/"+string1+"/r_o/total_occupancy_volume.txt", "r")
	lines1 = text_file.read().split(',')
	text_file.close()
 
	x7=[]
	y7=[]
	x8=[]
	y8=[]
	i=1
	for i in range(len(lines)-2):
	   x7.append(0)
	   y7.append(0)
	i=1
	for i in range(len(lines)-2):
	   x8.append(0)
	   y8.append(0)
	i=1
	for i in range(len(lines)-1):
	   x7[i-1]=i-1
	   y7[i-1]=str(float(lines[i]))
	   
	   i +=1
	
	i=1
	for i in range(len(lines1)-1):
	   x8[i-1]=i-1
	   y8[i-1]=str(float(lines1[i]))
	   
	   i +=1


        print (string1 )
	if x[300] !=0:
		print ("o_r P:300:" +str(y[320]))
	if x2[300] !=0:
		print ("O:300:" +str(y2[320]))
	if x3[300] !=0:
		print ("o_o P:300:" +str(y3[320]))
	if x4[300] !=0:
		print ("O:300:" +str(y4[320]))	
	if x5[300] !=0:
		print ("r_r P:300:" +str(y5[320]))
	if x6[300] !=0:
		print ("O:300:" +str(y6[320]))		
	if x7[300] !=0:
		print ("r_o P:300:" +str(y7[320]))
	if x8[300] !=0:
		print ("O:300:" +str(y8[320]))			
	
	plt.grid(color='r', linestyle='--', linewidth=1 )
	
	# plotting the points 
	string1 = "Total"
	string2 = "UAV Orthogonal"
	string3 = "UAV Regular"
	string4 = "UGV Orthogonal"
	string5 = "UGV Regular"
	#Total
	''' 
	plt.subplot(2, 1, 1)
	plt.title(string1+' Configuration Performance  Metrics (Resolution: 0.3)') 
	plt.plot(x, y,color='green',marker='x') 
	plt.plot(x3, y3,color='red',marker='^') 
	plt.plot(x5, y5,color='blue',marker='s') 
	plt.plot(x7, y7,color='orange',marker='o') 
	plt.xlabel('Sample Number ') 
	plt.ylabel('RMS Values ') 
	plt.legend(["o_r","o_o","r_r","r_o"])
	plt.subplot(2, 1, 2)
	plt.title(string1+' Configuration  Occupancy Metrics (Resolution: 0.3)') 
	plt.plot(x2, y2,color='green',marker='x') 
	plt.plot(x4, y4,color='red',marker='^') 
	plt.plot(x6, y6,color='blue',marker='s')  
	plt.plot(x8, y8,color='orange',marker='o') 
	plt.xlabel('Sample Number ') 
	plt.ylabel('Occupancy Rate  % ') 

	plt.show() 
	'''
	
	
	
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
