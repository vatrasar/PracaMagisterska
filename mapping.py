#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import time

class MappingNode():
    def __init__(self):
        self.node=rospy.init_node("laserConversionNode",anonymous=True)
        self.subLaser=rospy.Subscriber("coppeliaLaserInfo",LaserScan,self.get_laser_data)
        self.max_points=1024
        self.my_plot=None
        self.plot_fig=None
        self.is_first_datas=True
        self.rate=rospy.Rate(50)
        self.last_print_time=time.time()
        #fig = plt.figure()
        #self.ax = fig.add_subplot(1,1,1)
        rospy.loginfo("nowe dane test")
        rospy.spin()

    def get_laser_data(self,msg):
        
        

        
        laser_data=msg.ranges
        #rospy.loginfo("liczby: %d"%(len(laser_data)))
        conuter=0
        #points_list=self.get_points(old_data)
        x_array,y_array=self.get_x_y(laser_data)
        if(time.time()-self.last_print_time>0.01):
            self.show_rays(x_array,y_array)
            self.last_print_time=time.time()
        #rospy.loginfo("nowe dane test %f"%(self.last_data_publication_time))
        
        #self.test()
        #rospy.loginfo("nowe dane %f"%(time.time()-self.last_data_publication_time))

    def print_loop(self):
        pass

    def show_rays(self,x_array,y_array):
        #plt.figure(figsize=(6,10))
        
        
        if(self.is_first_datas):
            plt.show()
            plt.ion()
            
            self.is_first_datas=False

        plt.plot([y_array, np.zeros(np.size(y_array))],[x_array, np.zeros(np.size(x_array))], "ro-") # lines from 0,0 to the 

        #plt.axis("equal")
        #bottom, top = plt.ylim()  # return the current ylim
        #plt.ylim((top, bottom)) # rescale y axis, to match the grid orientation
        #plt.grid(True)

        

        
        
        plt.draw()
        plt.pause(0.00000001)
        plt.clf()
        
     



    def get_points(self,laser_data):
        points_list=[]
        current_point=[]
        for i,cordinate in enumerate(laser_data):
            
            if (i+1)%3==0:
                # current_point.append(cordinate)
                points_list.append(current_point)
                current_point=[]
            else:
                current_point.append(cordinate)
        return points_list


    def get_x_y(self,old_data):
        x=[]
        y=[]
        current_point=[]
        for i,cordinate in enumerate(old_data):
            
            if (i+1)%3==0:
                x.append(current_point[0])
                y.append(current_point[1])
                current_point=[]
            else:
                
                current_point.append(cordinate)

        x=np.array(x)
        y=np.array(y)
        return (x,y)

if __name__ == "__main__":

    newNode=MappingNode()    

# def create_map(laser_data,laser_step,laser_min):
#     """
#     Reading LIDAR laser beams (angles and corresponding distance data)
#     """
#     measures = [line.split(",") for line in open(f)]
#     angles = []
#     distances = []
#     for measure in measures:
#         angles.append(float(measure[0]))
#         distances.append(float(measure[1]))
#     angles = np.array(angles)
#     distances = np.array(distances)
#     return angles, distances

# def plot_test():


#     # Data for plotting
#     t = np.arange(0.0, 2.0, 0.01)
#     s = 1 + np.sin(2 * np.pi * t)

#     fig, ax = plt.subplots()
#     ax.plot(t, s)

#     ax.set(xlabel='time (s)', ylabel='voltage (mV)',
#         title='About as simple as it gets, folks')
#     ax.grid()

#     fig.savefig("test.png")
#     plt.show()