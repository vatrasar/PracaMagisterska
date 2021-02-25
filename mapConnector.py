import rospy
import math
from sensor_msgs.msg import LaserScan
import rospy
import numpy as np
import matplotlib
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
import numpy as np
import time
from tools import n_closest
from tools import rotate_2d_vector
from tfTools import get_2d_point_moved_using_vector
from tfTools import get_transform_vector_from_pose
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import warnings
from drone_control.msg import MyNumpy



class MappingConnector():
    def __init__(self):
        #self.node=rospy.init_node("laserConversionNode",anonymous=True)
        self.subMyMap=rospy.Subscriber("myMap",MyNumpy,self.sub_map_memmory)
        self.x_min=-5
        self.x_max=5
        self.y_min=-5
        self.y_max=5
        self.map_resolution=0.05
        self.dimension_x=int((self.x_max-self.x_min)/self.map_resolution)
        self.dimension_y=int((self.y_max-self.y_min)/self.map_resolution)
        self.map_memmory=np.zeros((self.dimension_y,self.dimension_x))#first for x secound for y
        self.max_points=1024
        self.my_plot=None
        self.plot_fig=None
        self.is_ready=False
        self.drone_pos=None
        self.theta=None
        self.last_print_time=time.time()
        self.buffor_range=10
        #fig = plt.figure()
        #self.ax = fig.add_subplot(1,1,1)

        warnings.filterwarnings("ignore")
        self.time_euler=time.time()
        self.time_pos=time.time()
        #rospy.spin()
        self.counter=0

    def get_map_memmory(self):
        return self.map_memmory

    def sub_map_memmory(self,msg):
        data=msg.data
        data=np.array(data)
        
        self.map_memmory=np.reshape(data,(self.dimension_y,self.dimension_x))
        #print(self.map_memmory.shape)
        #rospy.loginfo("jest")
        self.is_ready=True

    def isReady(self):
        return self.is_ready

  

    def is_target_reachable(self,target_point):
        x_i,y_i=self.get_point_on_map_index(target_point[0],target_point[1])
        if(self.map_memmory[y_i][x_i]!=0):
            return False
        else:
            return True




      

    def transfer_points(self,x_array,y_array,angle):
        for i,x_p in enumerate(x_array):
            x=x_array[i]
            y=y_array[i]
            
            #rospy.loginfo("angle %f"%(angle))
            
           # x,y=rotate_2d_vector([x,y],np.pi)
            x,y=rotate_2d_vector([x,y],angle)
            x,y=get_2d_point_moved_using_vector(self.drone_pos,(x,y))
            x_array[i]=x
            y_array[i]=y
        return (x_array,y_array)



    def get_point_on_map_index(self,x,y):

        x_i = int(round((x - self.x_min) / self.map_resolution))
        y_i= int(round((y - self.y_min) / self.map_resolution))
        return (x_i,y_i)

    def get_drone_x_y_arrays(self,x_array):
        x=np.zeros(np.size(x_array))
        y=np.zeros(np.size(x_array))
        y[:]=self.drone_pos[1]
        x[:]=self.drone_pos[0]
        return x,y
    
        



    def update_map_memory(self,x_array,y_array,map_resolution,x_min,y_min):
        for i, x in enumerate(x_array):
            x_i,y_i=self.get_point_on_map_index(x_array[i],y_array[i])
            self.map_memmory[y_i][x_i]=1
            neighbours=n_closest(self.map_memmory,(y_i,x_i),self.buffor_range)
            neighbours[neighbours!=1]=2

    def get_points_from_memory(self,value):
        points_indexs=np.where(self.map_memmory==value)
        x_index_array=points_indexs[1]
        y_index_array=points_indexs[0]
        x_array=x_index_array*self.map_resolution
        x_array=x_array+self.x_min

        y_array=y_index_array*self.map_resolution
        y_array=y_array+self.y_min


        return (x_array,y_array)
    


    def convert_index_to_point(self,x_i,y_i):
        x=x_i*self.map_resolution
        x=x+self.x_min

        y=y_i*self.map_resolution
        y=y+self.y_min
        return (x,y)
     



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