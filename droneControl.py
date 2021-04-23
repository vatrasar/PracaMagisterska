#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from tools2 import get_2d_vector_from_polar
from tools2 import get_3d_point_moved_using_vector
from tools2 import get_3d_vector_from_polar
from tools2 import get_distance
from tools2 import get_angle_to_target
from tools2 import convert_to_360
from tools2 import get_vector_to_target
from tools2 import get_absolute_vector_to_target
from tools2 import get_vector_with_length_and_direction
from tools2 import get_ros_point
import time
import numpy as np
import math
import random
from dataColector import RosDroneComunicator

class DroneController():

    def __init__(self):

        self.distance_tolerance=0.1
        self.distance_tolerance_circle=0.3
        self.angle_tolerance=0.01
        self.static_rotation_speed=0.05
        self.plus_static_rotation_speed=self.static_rotation_speed
        self.minus_static_rotation_speed=-self.static_rotation_speed
        self.drone_move_max_speed=0.025
        self.publication_rate=100
        self.rosComunicator=RosDroneComunicator()

        




    def get_new_rotation_speed_direction(self,angle_target,current_angle):
        minus_angle_target=0
        plus_angle_target=0
        minus_angle_target=angle_target-2*3.14
        plus_angle_target=angle_target
        
           

        delta_plus=0
        delta_minus=0
        
        if(current_angle<angle_target):
            delta_plus=angle_target-current_angle
            delta_minus=abs(2*3.14-angle_target+current_angle)
        else:
            delta_plus=2*3.14-current_angle+angle_target
            delta_minus=abs(current_angle-angle_target)
        
        # rospy.loginfo("plus delta:%f"%(delta_plus))
        # rospy.loginfo("minus delta:%f"%(delta_minus))

        new_static_rotation_speed=0
        if(delta_plus>delta_minus):

            new_static_rotation_speed=self.minus_static_rotation_speed
        else:
            new_static_rotation_speed=self.plus_static_rotation_speed
        return new_static_rotation_speed


    def rotation_control(self,target_point):
        
        angle_target=get_angle_to_target(target_point,self.rosComunicator.drone_pos)
        #rospy.loginfo("target_angle:%f"%(angle_target))
        
        
        #orginally between pi and -pi
        current_angle=convert_to_360(self.rosComunicator.theta)
        
        #rospy.loginfo("current:%f"%(current_angle))
    
        current_vector_to_target=get_vector_to_target(self.rosComunicator.drone_pos,target_point,self.rosComunicator.theta)
    
        rotation_speed_to_publication=self.get_new_rotation_speed_direction(angle_target,current_angle)
        
        is_target_angle_reached=False

        #if your distance to target is lower than speed you will it will set speed to publication to distance to target
        if(abs(self.static_rotation_speed)>=abs(angle_target-current_angle)):
            rotation_speed_to_publication=angle_target-current_angle
            is_target_angle_reached=True


        #rospy.loginfo("rotation_speed_to_publication:%f"%(rotation_speed_to_publication))
        self.rosComunicator.pubDroneStatic.publish(rotation_speed_to_publication)
        

        return is_target_angle_reached




    def get_target_pose(self):
        return self.rosComunicator.target_pos


    def has_first_target_message(self):
        return self.rosComunicator.has_first_target_message
    


    def move_to_point(self,target_point,is_one_step):


        while get_distance(target_point,self.rosComunicator.drone_pos)>self.distance_tolerance:
            rate=rospy.Rate(self.publication_rate)
            rate.sleep()
            is_target_angle_reached=self.rotation_control(target_point)
            if(is_target_angle_reached):       
                direction_vector=get_absolute_vector_to_target(self.rosComunicator.drone_pos,target_point)
                move_vector=get_vector_with_length_and_direction(min(get_distance(target_point,self.rosComunicator.drone_pos),self.drone_move_max_speed),direction_vector)
                move_ros_pos=get_ros_point(move_vector)
                self.rosComunicator.pub_move_vector.publish(move_ros_pos)
                if(is_one_step):
                    break
