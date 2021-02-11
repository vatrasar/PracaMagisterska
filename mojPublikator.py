#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import keyboard
import time
import numpy as np
import math

class DroneController():

    def __init__(self):
        self.pubDroneDirection=rospy.Publisher("droneDirection",Point,queue_size=1)
        self.pubDroneRotation=rospy.Publisher("droneRotation",Float64,queue_size=1)
        self.speedZ=0.5
        self.speedY=0.2
        #bez dublowania nazwy noda
        self.node=rospy.init_node("keyControlNode",anonymous=True)
        self.rate=rospy.Rate(10)
        self.target_pos=[50,50,50]
        self.drone_pos=[-50,-50,-50]
        self.distance_tolerance=0.5
        self.angle_tolerance=0.001
        self.subTargetPos=rospy.Subscriber("targetPosition",Point,self.get_target_position)
        self.subDronePos=rospy.Subscriber("dronePosition",Point,self.get_drone_position)
        self.subOrientation=rospy.Subscriber("eulerInfo",Float64,self.get_drone_orientation)
       
        self.theta=0
        self.rotation_speed=0.1
        self.init_power=0.01
        self.stop_rotation_tolerance=0.1
        self.current_rotation_speed=0.0
        self.max_rotation_speed=0.1
        self.previous_teta=0
        self.last_rotation_prev_update=time.time()
        self.minus_rotation_speed=-self.rotation_speed
        self.plus_rotation_speed=self.rotation_speed
        self.previous_direction_vec=[0,0,0]

    

    def get_drone_position(self,msg):
        #rospy.loginfo("dane o pozycji")
        self.drone_pos=[msg.x,msg.y,msg.z]

    def get_drone_orientationSpeed(self,msg):
        
        previousTeta=msg.data
        rate=rospy.Rate(10)
        rate.sleep()
        current_teta=self.theta
        rospy.loginfo("theta:%f"%(self.theta))
        rospy.loginfo("theta_prev:%f"%(previousTeta))
        self.current_rotation_speed=(current_teta-previousTeta)*10
        rospy.loginfo("aktualna rotacja:%f"%(self.current_rotation_speed))




    def get_drone_orientation(self,msg):
        #rospy.loginfo("dane o orientacji zaktualizwoane")
        
        self.theta=msg.data
        if(time.time()-self.last_rotation_prev_update>1):
            current_teta=self.theta
            self.current_rotation_speed=current_teta-self.previous_teta
            self.previous_teta=self.theta
 #           rospy.loginfo("aktualna rotacja:%f"%(self.current_rotation_speed))


    def get_target_position(self,msg):
        #rospy.loginfo("dane o pozycji targeta")
        self.target_pos=[msg.x,msg.y,msg.z]

    def rotation_control(self,init_vector_to_target):
        current_vector_to_target=self.get_vector_to_target(self.drone_pos,self.target_pos,self.theta)
        delta=current_vector_to_target[0]-self.previous_direction_vec[0]
        self.previous_direction_vec=current_vector_to_target
        rospy.loginfo("delta:%f"%(delta))
        rospy.loginfo("vec_y:%f"%(current_vector_to_target[1]))
        rospy.loginfo("vec_x:%f"%(current_vector_to_target[0]))
        if(delta<0):
            rospy.loginfo("decyduje delta")
            if(self.current_rotation_speed>0):
                self.rotation_speed=self.plus_rotation_speed
            else:
                self.rotation_speed=self.minus_rotation_speed
        
        xy_vector_lenght=math.sqrt(current_vector_to_target[1]**2+current_vector_to_target[0]**2)
        #rospy.loginfo("xy_vector_lenght:%f"%(xy_vector_lenght))
        desired_speed=((abs(xy_vector_lenght-current_vector_to_target[0]))/xy_vector_lenght)*self.max_rotation_speed
        #rospy.loginfo("desired speed:%f"%(desired_speed))
        #rospy.loginfo("current speed:%f"%(self.current_rotation_speed))
        #rospy.loginfo("target_x:%f"%(current_vector_to_target[0]))

        if(abs(self.current_rotation_speed)>desired_speed):
            #rospy.loginfo("decyduje desired")
            if(self.current_rotation_speed<0):
                self.rotation_speed=self.minus_rotation_speed
            else:
                self.rotation_speed=self.plus_rotation_speed

       
        self.pubDroneRotation.publish(self.rotation_speed)




    def talker(self):
        
        while not(rospy.is_shutdown()):
            #duration=int(input("podaj czas ruchu: \n"))
            #start_time=time.time()
            #direction=input("podaj kierunek:\n")
            #rospy.loginfo("\nkierunek: %s \ntime: %d"%(direction,duration))
            target=(1,1,1)
            input("enter do startu")
            init_vector_to_target=self.get_vector_to_target(self.drone_pos,self.target_pos,self.theta)
            while self.get_distance(self.target_pos,self.drone_pos)>self.distance_tolerance:
                self.rate.sleep()
                self.rotation_control(init_vector_to_target)
                # vector_to_target=self.get_vector_to_target(self.drone_pos,self.target_pos,self.theta)
                
                #rospy.loginfo("rotation_speed:%f"%(self.current_rotation_speed))
                # rospy.loginfo("\nkierunek: %f"%(vector_to_target[1]))
                # if(abs(vector_to_target[1])<self.angle_tolerance):
                #     rospy.loginfo("osiagnieto cel")
                    
                    
                #     if(abs(self.current_rotation_speed)>self.stop_rotation_tolerance):
                #         if(self.current_rotation_speed<0):
                #             self.rotation_speed=-self.plus_rotation_speed
                #         else:
                #             self.rotation_speed=self.plus_rotation_speed
                #         rotation_speed=self.rotation_speed*(abs(self.current_rotation_speed)/0.01)
                #         self.pubDroneRotation.publish(rotation_speed)
                    
                # else:
                    
                #     rotation_speed=self.rotation_speed
                #     self.pubDroneRotation.publish(rotation_speed)

                
                
            rospy.loginfo("ruch zakonczony\n")


    def get_not_rotated_vector(self,current_position,target_position):
        (x_current,y_current,z_current)=current_position
        (x_target,y_target,z_target)=target_position
        no_rotated_vector=(x_target-x_current,y_target-y_current,z_target-z_current)
        return no_rotated_vector

    def get_distance(self,source,position):
        x_target,y_target,z_target=position
        x_source,y_source,z_source=source
        
        p1 = np.array([x_target,y_target,z_target])
        p2 = np.array([x_source,y_source,z_source])
        # rospy.loginfo("p1 "+str(z_target))
        # rospy.loginfo("p2 "+str(z_source))

        squared_dist = np.sum((p1-p2)**2, axis=0)
        dist = np.sqrt(squared_dist)
        #rospy.loginfo(dist)
        return dist
    
    def get_vector_to_target(self,current_position,target_position,theta):
        

        (x_current,y_current,z_current)=current_position
        #rospy.loginfo("current_vector (%f,%f,%f)"%(x_current,y_current,z_current))
        (x_target,y_target,z_target)=target_position
        #rospy.loginfo("target_vector (%f,%f,%f)"%(x_target,y_target,z_target))
        nonNormalizetVector=(x_target-x_current,y_target-y_current,z_target-z_current)
       # rospy.loginfo("result_vector (%f,%f,%f)"%(nonNormalizetVector[0],nonNormalizetVector[1],nonNormalizetVector[2]))
  #      rospy.loginfo("target_x:%f  x_current: %f"%(x_target,x_current))
        #normalization
        #an_array=np.array(nonNormalizetVector)
        #norm = np.linalg.norm(an_array)
        #normal_vector = an_array/norm
        #resultVector=normal_vector*self.max_speed
        resultVector=nonNormalizetVector
#        rospy.loginfo("result (%f,%f,%f)"%(resultVector[0],resultVector[1],resultVector[2]))
#        rospy.loginfo("current (%f,%f,%f)"%(x_current,y_current,z_current))
        angle_z=theta
        
#        rospy.loginfo("current (%f,%f,%f)"%(x_current,y_current,z_current))
        result_vector_after_rotation=self.rotate_vector((resultVector[0],resultVector[1],resultVector[2]),-angle_z)
        rospy.loginfo("result_vector after rot (%f,%f,%f)"%(result_vector_after_rotation[0],result_vector_after_rotation[1],result_vector_after_rotation[2]))
 #       rospy.loginfo("angle_z %f"%(angle_z))
 #       rospy.loginfo("result after angle (%f,%f,%f)"%(result_vector_after_rotation[0],result_vector_after_rotation[1],result_vector_after_rotation[2]))
        
        return result_vector_after_rotation
    
    def rotate_vector(self, vector,angle):
        #rospy.loginfo("resultVector (%f,%f,%f)"%(vector[0],vector[1],vector[2]))
        #rospy.loginfo("angle %f"%(angle))
        new_x=math.cos(angle)*vector[0]-math.sin(angle)*vector[1]
        #rospy.loginfo("new x %f"%(new_x))
        #rospy.loginfo("sin angle %f"%(math.sin(angle)))
        #rospy.loginfo("cos angle %f"%(math.cos(angle)))
        new_y=math.sin(angle)*vector[0]+math.cos(angle)*vector[1]
        return (new_x,new_y,vector[2])



if __name__ == '__main__':
    try:
        dronecontroller=DroneController()
        dronecontroller.talker()

       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")