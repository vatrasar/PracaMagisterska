import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from tools2 import get_2d_vector_from_polar
from tools2 import get_3d_point_moved_using_vector
from tools2 import get_3d_vector_from_polar
import time
import numpy as np
import math
import random
from droneControl import DroneController
import random

class CircleMove():

    def __init__(self):
        self.droneController=DroneController()
        self.current_point_on_circle_index=0
        myRandom=random.Random()


    def circle_move(self,radius):

        points_on_circle_max_number=50
        #points_list=self.get_points_on_circle(radius,points_on_circle_max_number)
        
        rate=rospy.Rate(100)
        i=0
        while True:
            if(self.droneController.has_first_target_message()):
                i+=1
                #rospy.loginfo("iter (%f )"%(i))
                points_list=self.get_points_on_circle(radius,points_on_circle_max_number)

                self.move_to_point(points_list[current_point_on_circle_index])
                
                if(current_point_on_circle_index>points_on_circle_max_number-2):
                    current_point_on_circle_index=0
                    i=0
                else:
                    current_point_on_circle_index+=1
            rate.sleep()


    #theta is usuing to adjust hight of drone
    # def get_new_theta_step(self,theta,old_theta_step,max_theta_angle,target_theta):

    
    #     if(theta+old_theta_step>max_theta_angle or theta+old_theta_step<0):
    #         return -old_theta_step
    #     else:
    #         return old_theta_step]


        #theta is usuing to adjust hight of drone
    def get_new_theta_step(self,theta,target_theta,old_step):
        if(theta<target_theta):
            return abs(old_step)
        else:
            return -abs(old_step)

    def get_new_target_theta(self,target_theta,step,current_theta,height_point_number):
       # rospy.loginfo("target%f"%(target_theta))
        if((step>0 and target_theta<current_theta+step) or (step<0 and target_theta>current_theta+step)):
            level=random.randint(0,height_point_number)
            rospy.loginfo("level %f"%(level))
            return level*abs(step)
        else:
            return target_theta
        

       


    def circle_move_3d(self,radius):
        points_on_circle_max_number=50
        vertical_points=100
        max_theta_angle=(3.14*0.3)
        theta_step=max_theta_angle/vertical_points
        theta_target=random.randint(0,vertical_points)*theta_step #theta is usuing to adjust hight of drone
       
        #rospy.loginfo("step:%f target%f"%(theta_step,theta_target))
        theta=0
        #points_list=self.get_points_on_circle(radius,points_on_circle_max_number)
        current_point_on_circle_index=0
        rate=rospy.Rate(self.droneController.publication_rate)
       
        while True:
            if(self.droneController.has_first_target_message()):
                
                
                points_list=self.get_points_on_circle_3d(radius,points_on_circle_max_number,theta)
                
                theta_target=self.get_new_target_theta(theta_target,theta_step,theta,vertical_points)
               
                theta_step=self.get_new_theta_step(theta,theta_target,theta_step)
                theta+=theta_step
                #rospy.loginfo("3theta %f step:%f target%f"%(theta,theta_step,theta_target))
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



# if __name__ == '__main__':
#     try:
#         circleMove=CircleMove()

#         circleMove.circle_move_3d(1)

       
#     except rospy.ROSInterruptException:
#         rospy.loginfo("node terminated.")