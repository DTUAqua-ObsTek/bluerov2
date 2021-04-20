#!/usr/bin/env python
import rosbag
bag = rosbag.Bag('waterlinked_data_0.bag')
t_arr=[]
x_arr=[]
y_arr=[]
first_loop = True
t0 = 0
file = open("data_export.csv","w")
file.write("t[s],x[m],y[m]\n")
for topic, msg, t in bag.read_messages(topics=['/waterlinked/position/acoustic/filtered']):
	if first_loop:
		t0 = t.to_sec()
		first_loop = False
	file.write(str(t.to_sec()-t0) + "," + str(msg.x) + "," + str(msg.y) + "\n")
file.close()
bag.close()