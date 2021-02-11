#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from tools import convert_to_360
from tools import rotate_2d_vector
import time
class LaserConversionNode():
    def __init__(self):
        self.node=rospy.init_node("laserConversionNode",anonymous=True)
        self.subLaser=rospy.Subscriber("coppeliaLaserInfo",LaserScan,self.get_laser_data)
        self.pubLaser=rospy.Publisher("myLaserInfo",LaserScan,queue_size=5)
        self.max_points=1024
        self.last_data_publication_time=time.time()
        rospy.spin()
    
    def get_laser_data(self,msg):
        
        
        self.last_data_publication_time=time.time()
        old_data=msg.ranges
        conuter=0
        points_list=self.get_points(old_data)
        #rospy.loginfo("points number:%d"%(len(points_list)))
        inf_array=np.full((self.max_points),np.inf)
    
        
        results=self.convert_point_to_ros_format(points_list,msg.angle_increment,inf_array)
        msg.ranges=results
        msg.header.stamp=rospy.Time.now()
        self.pubLaser.publish(msg)
        #rospy.loginfo("opubloikowanorrrr")



    def convert_point_to_ros_format(self,points_list,increment,inf_array):
        #rospy.loginfo("incement %f"%(increment))
        for point in points_list:
            
            rotated_point=rotate_2d_vector(point,np.pi)
            distance,angle=self.cart2pol(rotated_point[0],rotated_point[1])
            #rospy.loginfo("angle orginal %f"%(angle))
            orginal_angle=angle
            angle=convert_to_360(angle)
            #rospy.loginfo("angle after 360 %f"%(angle))
            #angle=(5.0/3)*np.pi-angle
            
            #rospy.loginfo("angle result %f"%(angle))
            i=int((angle)/increment)
     
            inf_array[i-1]=distance

        
        return inf_array.tolist()




    def get_points(self,old_data):
        points_list=[]
        current_point=[]
        for i,cordinate in enumerate(old_data):
            
            if (i+1)%3==0:
                # current_point.append(cordinate)
                points_list.append(current_point)
                current_point=[]
            else:
                current_point.append(cordinate)
        return points_list

    def cart2pol(self,x, y):
        rho = np.sqrt(x**2 + y**2)
        phi = np.arctan2(y, x)
        return(rho, phi)

    def get_distance_form_zero(self,point):
        x_target,y_target,z_target=point
        
        
        
        p2 = np.array([x_target,y_target])


        squared_dist = np.sum((p2)**2, axis=0)
        dist = np.sqrt(squared_dist)

        return dist



        


            
        
    

if __name__ == "__main__":

    newNode=LaserConversionNode()
    

