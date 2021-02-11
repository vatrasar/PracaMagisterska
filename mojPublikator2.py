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

class DroneController():

    def __init__(self):
        self.pub_move_vector=rospy.Publisher("droneMoveVector",Point,queue_size=1)

        self.pubDroneStatic=rospy.Publisher("droneStaticRotation",Float64,queue_size=1)

        
        self.node=rospy.init_node("keyControlNode",anonymous=True)
  
        self.target_pos=[10,10,10]
        self.drone_pos=[-1,-1,-1]
        self.distance_tolerance=0.01
        self.distance_tolerance_circle=0.3
        self.angle_tolerance=0.01
        self.subTargetPos=rospy.Subscriber("targetPosition",Point,self.get_target_position)
        self.subDronePos=rospy.Subscriber("dronePosition",Point,self.get_drone_position)
        self.subOrientation=rospy.Subscriber("eulerInfo",Float64,self.get_drone_orientation)
        self.has_first_target_message=False
        self.theta=0

        self.static_rotation_speed=0.5
        self.plus_static_rotation_speed=self.static_rotation_speed
        self.minus_static_rotation_speed=-self.static_rotation_speed
        self.drone_move_max_speed=0.2
        self.publication_rate=100



    

    def get_drone_position(self,msg):
        
        self.drone_pos=[msg.x,msg.y,msg.z]


    def get_drone_orientation(self,msg):
       
        self.theta=msg.data


    def get_target_position(self,msg):
        
        self.target_pos=[msg.x,msg.y,msg.z]
        self.has_first_target_message=True

    def get_angle_to_target(self,target_point):
        absolute_vector=self.get_absolute_vector_to_target(self.drone_pos,target_point)
        xy_vector=[absolute_vector[0],absolute_vector[1]]
        angle_target=self.convert_to_360(self.get_vector_angle(xy_vector))
        return angle_target


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
        
        angle_target=self.get_angle_to_target(target_point)
        #rospy.loginfo("target_angle:%f"%(angle_target))
        
        
        #orginally between pi and -pi
        current_angle=self.convert_to_360(self.theta)
        
        #rospy.loginfo("current:%f"%(current_angle))
    
        current_vector_to_target=self.get_vector_to_target(self.drone_pos,target_point,self.theta)
    
        rotation_speed_to_publication=self.get_new_rotation_speed_direction(angle_target,current_angle)
        
        is_target_angle_reached=False

        #if your distance to target is lower than speed you will it will set speed to publication to distance to target
        if(abs(self.static_rotation_speed)>=abs(angle_target-current_angle)):
            rotation_speed_to_publication=angle_target-current_angle
            is_target_angle_reached=True


        #rospy.loginfo("rotation_speed_to_publication:%f"%(rotation_speed_to_publication))
        self.pubDroneStatic.publish(rotation_speed_to_publication)
        

        return is_target_angle_reached

    def get_ros_point(self,vector):
        point=Point()
        point.x=vector[0]
        point.y=vector[1]
        point.z=vector[2]
        return point

    def convert_to_360(self,angle):
        if(angle<0):
            angle=2*3.14+angle
        return angle


    def get_vector_angle(self,vector):
        vector_2=vector
        vector_1=[1,0]
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        det_produtcst=np.cross(unit_vector_1,unit_vector_2)
        angle = np.arctan2(det_produtcst,dot_product)
        return angle
    

    def get_target_ros_pos(self,target_coordinates):
        point=Point()
        point.x=target_coordinates[0]
        point.y=target_coordinates[1]
        point.z=target_coordinates[1]
        return point




    def get_vector_with_length_and_direction(self,drone_move_max_speed,direction_vector):
        sum_of_squares=direction_vector[0]**2+direction_vector[1]**2+direction_vector[2]**2
        scale_factor=drone_move_max_speed/math.sqrt(sum_of_squares)
        result_vector=[direction_vector[0],direction_vector[1],direction_vector[2]]
        result_vector[0]=result_vector[0]*scale_factor
        result_vector[1]=result_vector[1]*scale_factor
        result_vector[2]=result_vector[2]*scale_factor
        return result_vector
    
    def get_vector_target_point(self,vector,current_pos):
        vector_target_point=[0,0,0]
        vector_target_point[0]=current_pos[0]+vector[0]
        vector_target_point[1]=current_pos[1]+vector[1]
        vector_target_point[2]=current_pos[2]+vector[2]
        return vector_target_point



    def move_arround_target(self):
        max_distance_form_target=1
        min_z=0.3
        rate=rospy.Rate(self.publication_rate)
        
        ros_point_directon_vector=Point()
        ros_point_directon_vector.x=0
        ros_point_directon_vector.y=0
        ros_point_directon_vector.z=0
        direction_point=self.target_pos
        input("enter do startu")
        while(not(rospy.is_shutdown())):
            

            distance=self.get_distance(self.drone_pos,self.target_pos)
            if(distance>max_distance_form_target or self.drone_pos[2]<min_z):
                rospy.loginfo("reached")
                
                
                direction_vector=self.get_reflection_vector(self.drone_pos,self.target_pos,max_distance_form_target,min_z)
                vector_target_point=self.get_vector_target_point(self.get_vector_with_length_and_direction(self.drone_move_max_speed*100,direction_vector),self.drone_pos)
                direction_vector=self.get_vector_with_length_and_direction(self.drone_move_max_speed,direction_vector)
                ros_point_directon_vector=self.get_ros_point(direction_vector)
                is_target_angle_reached=False
                while(not(is_target_angle_reached)):
                    is_target_angle_reached=self.rotation_control(vector_target_point)


                while(distance>max_distance_form_target):
                    rospy.loginfo("come back")
                    distance=self.get_distance(self.drone_pos,self.target_pos)
                    self.pub_move_vector.publish(ros_point_directon_vector)
                    rate.sleep()

            self.pub_move_vector.publish(ros_point_directon_vector)
            rate.sleep()

    def move_to_target(self):
        
        while not(rospy.is_shutdown()):

            
            input("enter do startu")
            
            while self.get_distance(self.target_pos,self.drone_pos)>self.distance_tolerance:
                rate=rospy.Rate(self.publication_rate)
                rate.sleep()
                is_target_angle_reached=self.rotation_control(self.target_pos)
                if(is_target_angle_reached):
                    
                    direction_vector=self.get_absolute_vector_to_target(self.drone_pos,self.target_pos)
                    move_vector=self.get_vector_with_length_and_direction(min(self.get_distance(self.target_pos,self.drone_pos),self.drone_move_max_speed),direction_vector)
                    move_ros_pos=self.get_ros_point(move_vector)
                    self.pub_move_vector.publish(move_ros_pos)


    def move_to_point(self,target_point):

        rospy.loginfo("random point (%f,%f,%f)"%(target_point[0],target_point[1],target_point[2]))
        while self.get_distance(target_point,self.drone_pos)>self.distance_tolerance_circle:
            rate=rospy.Rate(self.publication_rate)
            rate.sleep()
            is_target_angle_reached=self.rotation_control(target_point)
            if(is_target_angle_reached):       
                direction_vector=self.get_absolute_vector_to_target(self.drone_pos,target_point)
                move_vector=self.get_vector_with_length_and_direction(min(self.get_distance(target_point,self.drone_pos),self.drone_move_max_speed),direction_vector)
                move_ros_pos=self.get_ros_point(move_vector)
                self.pub_move_vector.publish(move_ros_pos)



    

    def get_reflection_vector(self,current_position,target_pos,max_distance,min_z):
        random_point=self.get_random_point_around_target(max_distance,target_pos,min_z)
        #rospy.loginfo("random point (%f,%f,%f)"%(x_target,y_target,z_target))
        direction_vector=self.get_absolute_vector_to_target(current_position,random_point)
        result_vector=self.get_vector_with_length_and_direction(max_distance,direction_vector)
        
        return result_vector


    def get_random_point_around_target(self,max_distance_form_target,target_pos,min_z):
        
        randomSign=(-1)**(random.randint(0,1))
        rospy.loginfo("randsign:%d"%(randomSign))
        x=random.random()*randomSign
        randomSign=(-1)**(random.randint(0,1))
        y=random.random()*randomSign
        z=random.random()
        result_pos=self.get_vector_with_length_and_direction(max_distance_form_target,(x,y,z))
        result_pos[0]=result_pos[0]+target_pos[0]
        result_pos[1]=result_pos[1]+target_pos[1]
        result_pos[2]=max(min_z,target_pos[2])+result_pos[2]
        return result_pos


    def get_distance(self,source,position):
        x_target,y_target,z_target=position
        x_source,y_source,z_source=source
        
        p1 = np.array([x_target,y_target,z_target])
        p2 = np.array([x_source,y_source,z_source])


        squared_dist = np.sum((p1-p2)**2, axis=0)
        dist = np.sqrt(squared_dist)

        return dist


    def get_absolute_vector_to_target(self,current_position,target_position):
        (x_current,y_current,z_current)=current_position
        
        (x_target,y_target,z_target)=target_position
        
        result=(x_target-x_current,y_target-y_current,z_target-z_current)
        return result
    
    def get_vector_to_target(self,current_position,target_position,theta):
        

        (x_current,y_current,z_current)=current_position

        (x_target,y_target,z_target)=target_position

        nonNormalizetVector=(x_target-x_current,y_target-y_current,z_target-z_current)

        resultVector=nonNormalizetVector

        angle_z=theta

        result_vector_after_rotation=self.rotate_vector((resultVector[0],resultVector[1],resultVector[2]),-angle_z)

        
        return result_vector_after_rotation
    
    def rotate_vector(self, vector,angle):
        new_x=math.cos(angle)*vector[0]-math.sin(angle)*vector[1]
        new_y=math.sin(angle)*vector[0]+math.cos(angle)*vector[1]
        return (new_x,new_y,vector[2])

    def circle_move(self,radius):
        points_on_circle_max_number=50
        #points_list=self.get_points_on_circle(radius,points_on_circle_max_number)
        current_point_on_circle_index=0
        rate=rospy.Rate(self.publication_rate)
        i=0
        while True:
            if(self.has_first_target_message):
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
        theta=0
        #points_list=self.get_points_on_circle(radius,points_on_circle_max_number)
        current_point_on_circle_index=0
        rate=rospy.Rate(self.publication_rate)
       
        while True:
            if(self.has_first_target_message):
                
                
                points_list=self.get_points_on_circle_3d(radius,points_on_circle_max_number,theta)
                theta_step=self.get_new_theta_step(theta,theta_step,max_theta_angle)
                theta+=theta_step
                rospy.loginfo("theta %f step:%f"%(theta,theta_step))
                self.move_to_point(points_list[current_point_on_circle_index])
                
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
            result_point=get_3d_point_moved_using_vector(vector,self.target_pos)
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
            result_point=get_3d_point_moved_using_vector(vector,self.target_pos)
            points_list.append(result_point)
            angle+=step
        
        # for point in points_list :
        #     rospy.loginfo("point number (%f %f %f)"%(point[0],point[1],point[2]))
        return points_list



if __name__ == '__main__':
    try:
        dronecontroller=DroneController()
        #dronecontroller.move_to_target()
        #dronecontroller.move_arround_target()
        dronecontroller.circle_move_3d(1)

       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")