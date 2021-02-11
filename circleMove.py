#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from tools2 import get_2d_vector_from_polar
from tools2 import get_3d_point_moved_using_vector
from tools2 import get_3d_vector_from_polar
import keyboard
import time
import numpy as np
import math
import random
from droneControl import DroneController

class CircleMove():

    def __init__(self):
        self.droneController=DroneController()

    def circle_move(self,radius):

        points_on_circle_max_number=50
        #points_list=self.get_points_on_circle(radius,points_on_circle_max_number)
        current_point_on_circle_index=0
        rate=rospy.Rate(self.droneController.publication_rate)
        i=0
        while True:
            if(self.droneController.has_first_target_message()):
                i+=1
                rospy.loginfo("iter (%f )"%(i))
                points_list=self.get_points_on_circle(radius,points_on_circle_max_number)

                self.move_to_point(points_list[current_point_on_circle_index])
                
                if(current_point_on_circle_index>points_on_circle_max_number-2):
                    current_point_on_circle_index=0
                    i=0
                else:
                    current_point_on_circle_index+=1
            rate.sleep()


    #theta is usuing to adjust hight of drone
    def get_new_theta_step(self,theta,old_theta_step,max_theta_angle):
        if(theta+old_theta_step>max_theta_angle or theta+old_theta_step<0):
            return -old_theta_step
        else:
            return old_theta_step


    def circle_move_3d(self,radius):
        points_on_circle_max_number=50
        vertical_points=100
        max_theta_angle=(3.14*0.3)
        theta_step=max_theta_angle/vertical_points
        theta=0 #theta is usuing to adjust hight of drone
        #points_list=self.get_points_on_circle(radius,points_on_circle_max_number)
        current_point_on_circle_index=0
        rate=rospy.Rate(self.droneController.publication_rate)
       
        while True:
            if(self.droneController.has_first_target_message()):
                
                
                points_list=self.get_points_on_circle_3d(radius,points_on_circle_max_number,theta)
                theta_step=self.get_new_theta_step(theta,theta_step,max_theta_angle)
                theta+=theta_step
                rospy.loginfo("theta %f step:%f"%(theta,theta_step))
                self.droneController.move_to_point(points_list[current_point_on_circle_index])
                
                if(current_point_on_circle_index>points_on_circle_max_number-2):
                    current_point_on_circle_index=0
                    
                else:
                    current_point_on_circle_index+=1
            rate.sleep()



    def get_points_on_circle(self,radius,points_on_circle_max_number):
        step=2*3.14/(points_on_circle_max_number+1)
        angle=0
        points_list=[]
        for i in range(0,points_on_circle_max_number):
            vector=get_2d_vector_from_polar(angle,radius)
            result_point=get_3d_point_moved_using_vector(vector,self.droneController.get_target_pose())
            points_list.append(result_point)
            angle+=step
        
        # for point in points_list :
        #     rospy.loginfo("point number (%f %f %f)"%(point[0],point[1],point[2]))
        return points_list
    
    def get_points_on_circle_3d(self,radius,points_on_circle_max_number,theta):
        step=2*3.14/(points_on_circle_max_number+1)
        angle=0
        points_list=[]
        for i in range(0,points_on_circle_max_number):
            vector=get_3d_vector_from_polar(angle,theta,radius)
            result_point=get_3d_point_moved_using_vector(vector,self.droneController.get_target_pose())
            points_list.append(result_point)
            angle+=step
        
        # for point in points_list :
        #     rospy.loginfo("point number (%f %f %f)"%(point[0],point[1],point[2]))
        return points_list



if __name__ == '__main__':
    try:
        circleMove=CircleMove()

        circleMove.circle_move_3d(1)

       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")