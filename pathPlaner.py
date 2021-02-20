#!/usr/bin/env python3
from circleMove import CircleMove
from moveToTarget import MoveToTarget
from droneControl import DroneController
import rospy
from mapping import MappingNode
from geometry_msgs.msg import Point
from tools2 import get_absolute_vector_to_target
from tools2 import get_vector_with_length_and_direction
from tools2 import get_distance
import time
class PathPlaner():

    def __init__(self):

        self.droneController=DroneController()
        #self.targetProvider=CircleMove()
        #self.circleMove.circle_move_3d(1)
        self.targetProvider=MoveToTarget()
        self.map=MappingNode()
        self.move_loop()

    def move_loop(self):
        rate=rospy.Rate(100)
        rate2=rospy.Rate(1)
        rate2.sleep()
        while(True):
            if(self.targetProvider.isReady() and self.map.is_ready()):
                target_point=self.targetProvider.get_next_target()
                
                
                if(self.map.is_target_reachable(target_point)):
                    
                    self.droneController.move_to_point(target_point,True)
                else:
                    rospy.loginfo("nieosiagalny")
                rate.sleep()

                #     path,is_target_reachable=self.get_path_to_target(target_point)
                # if(is_target_reachable):
                #     self.droneController.move_to_point(target_point)

    # def get_step_to_target(self,target_point):
    #     direction_vector=get_absolute_vector_to_target(self.droneController.rosComunicator.drone_pos,target_point)
    #     rospy.loginfo(target_point)
    #     move_vector=get_vector_with_length_and_direction(min(get_distance(target_point,self.droneController.rosComunicator.drone_pos),self.droneController.drone_move_max_speed),direction_vector)
    #     return move_vector      



if __name__ == '__main__':
    try:
        node=PathPlaner()
        

       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")


    
    








if __name__ == "__main__":

    newNode=PathPlaner()  