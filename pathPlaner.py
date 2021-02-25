#!/usr/bin/env python3
from circleMove import CircleMove
from moveToTarget import MoveToTarget
from droneControl import DroneController
import rospy
from mapConnector import MappingConnector
from geometry_msgs.msg import Point
from tools2 import get_absolute_vector_to_target
from tools2 import get_vector_with_length_and_direction
from tools2 import get_distance
from tools2 import get_2d_distance
from tools2 import get_3d_point_moved_using_vector
from tools import n_closest
import time
import numpy as np
from tools import is_points_equal

class Cell():
    def __init__(self,x,y,parent_cell):
        self.x=x 
        self.y=y
        self.parent_cell=parent_cell
        self.value=0
    def get_pose(self):
        return (self.x,self.y)

class PathPlaner():

    def __init__(self):

        self.droneController=DroneController()
        #self.targetProvider=CircleMove()
        #self.circleMove.circle_move_3d(1)
        self.targetProvider=MoveToTarget()
        self.map=MappingConnector()
        self.i=0
        self.move_loop()
        


    def move_loop(self):
        rate=rospy.Rate(100)
        rate2=rospy.Rate(1)
        rate2.sleep()
        while(True):
            if(self.targetProvider.isReady() and self.map.is_ready):
                target_point=self.targetProvider.get_next_target()
                
                
                if(self.map.is_target_reachable(target_point)):
                    if(self.is_straight_path_free_from_obstacles(target_point)):
                        # rospy.loginfo("wyznaczono trase,step %d"%(1))
                        self.droneController.move_to_point(target_point,True)
                        
                    else:
                        try:
                            # rospy.loginfo("wyznaczono trase,step %d"%(2))
                            path=self.move_with_a_star(target_point)
                            step=int(self.droneController.distance_tolerance/self.map.map_resolution)+1
                            step_point=(path[step][0],path[step][1],target_point[2])
                        except IndexError:
                            # rospy.loginfo("wyznaczono trase,step %d"%(3))
                            step_point=self.get_free_point_near_drone()
                            
                            #rospy.loginfo("%f %f"%(step_point[0],step_point[1]))
                            step_point=(step_point[0],step_point[1],target_point[2])
                            
                        
                        
                        # rospy.loginfo("wyznaczono trase,step %d"%(5))
                        self.droneController.move_to_point(step_point,False)
                       

                    
                    
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
    def get_free_point_near_drone(self):
        current_pose=self.droneController.rosComunicator.drone_pos
        start_pose=self.map.get_point_on_map_index(current_pose[0],current_pose[1])
        d=1
        
        while(True):
            closest=n_closest(self.map.get_map_memmory(),start_pose,d)
            if(closest[closest==0].size!=0):
                closest[closest==0]=5
                points_indexs=np.where(self.map.get_map_memmory()==5)
                closest[closest==0]=0
                x_index_array=points_indexs[1]
                y_index_array=points_indexs[0]
                #rospy.loginfo("wyznaczono trase,step %d"%(d))
                for i,x in enumerate(x_index_array):
                    point=self.map.convert_index_to_point(x_index_array[i],y_index_array[i])
                    if(self.is_straight_path_free_from_obstacles((point[0],point[1],0))):
                        return point

            else:
                d=d+1


    def is_straight_path_free_from_obstacles(self,target):
        source=self.droneController.rosComunicator.drone_pos
        direction_vector=get_absolute_vector_to_target(source,target)
        move_vector=get_vector_with_length_and_direction(min(get_distance(target,source),self.droneController.drone_move_max_speed),direction_vector)
        # move_ros_pos=get_ros_point(move_vector)
        current_point=source
        #rospy.loginfo(move_vector)
        while(get_distance(target,current_point)>self.droneController.distance_tolerance):
            if(self.map.is_target_reachable(current_point)):
                #rospy.loginfo(get_distance(target,source))
                current_point=get_3d_point_moved_using_vector(move_vector,current_point)
            else:
                return False
        return True

    
    def move_with_a_star(self,target_point):
        cell_state_map=np.zeros(self.map.get_map_memmory().shape)#0 not on list 1 on list 2 visited
        potential_cells_list=[]
        closed_nodes=[]

        current_pose=self.droneController.rosComunicator.drone_pos
        start_pose=self.map.get_point_on_map_index(current_pose[0],current_pose[1])
        current_parrent=None
        
        target_pose=self.map.get_point_on_map_index(target_point[0],target_point[1])
        new_current_cell=Cell(start_pose[0],start_pose[1],current_parrent)
        
        while not(is_points_equal(new_current_cell.get_pose(),target_pose)):
            
            
            closed_nodes.append(new_current_cell)
            poses_arround_target=self.get_cells_arround_target(new_current_cell.x,new_current_cell.y,cell_state_map)
            cells_arround_target_list=self.get_cells_from_poses(poses_arround_target,new_current_cell)

            self.set_cells_values(cells_arround_target_list,target_pose,start_pose)
            potential_cells_list.extend(cells_arround_target_list)
            min_cell,min_cell_index=self.get_cell_with_min_value(potential_cells_list)
            potential_cells_list.pop(min_cell_index)
            
            

            
            new_current_cell=min_cell
            self.update_state_map(cell_state_map,new_current_cell,cells_arround_target_list)
            # if(self.i<500):
            #     rospy.loginfo("result %f"%(min_cell.value))
            #     self.i=self.i+1
        #rospy.loginfo("wyznaczono trase")
        return self.get_point_on_path_list(new_current_cell)

    def get_point_on_path_list(self,last_cell):
        current_cell=last_cell
        path=[]
        while current_cell!=None:
            path.append(self.map.convert_index_to_point(current_cell.x,current_cell.y))
            current_cell=current_cell.parent_cell
            

        path.reverse()
        return path
    def update_state_map(self,state_map,new_current_point,cells_arround_target_list):
        for cell in cells_arround_target_list:
            state_map[cell.y][cell.x]=1
        state_map[new_current_point.y][new_current_point.x]=2

    def get_cell_with_min_value(self,cells_list):
        min_cell=cells_list[0]
        min_index=0
        for index,cell in enumerate(cells_list) :
            if(cell.value<min_cell.value):
                min_cell=cell
                min_index=index
        #     if(self.i<2):
        #         rospy.loginfo("value %f"%(cell.value))
        #         #self.i=self.i+1
        # # rospy.loginfo("value yyy")
        # if(self.i<2):
        #         rospy.loginfo("min %f"%(min_cell.value))
        # self.i=self.i+1
        return (min_cell,min_index)

    def get_cells_from_poses(self,poses_arround_target,parent):
        cells_list=[]
        for pose in poses_arround_target:
            new_cell=Cell(pose[0],pose[1],parent)
            cells_list.append(new_cell)

        return cells_list

    def set_cells_values(self,cells_list,target,start):
        
        for cell in cells_list:
            value=self.get_cell_value(target,start,cell)
            
            cell.value=value

    def get_cell_value(self,target,start_pose,cell):
        
        return get_2d_distance(cell.get_pose(),target)+get_2d_distance(cell.get_pose(),start_pose)



    def get_cells_arround_target(self,current_x_i,current_y_i,cell_state_map):
        x=0
        cells_index_list=[]
        if(current_x_i>0):
            if(self.is_cell_avaiable(current_x_i-1,current_y_i,cell_state_map)):
                cells_index_list.append((current_x_i-1,current_y_i))
            if(current_y_i>0 and self.is_cell_avaiable(current_x_i-1,current_y_i-1,cell_state_map)):
                cells_index_list.append((current_x_i-1,current_y_i-1))
            
        if(current_y_i>0 and self.is_cell_avaiable(current_x_i,current_y_i-1,cell_state_map)):
            cells_index_list.append((current_x_i,current_y_i-1))

        if(current_y_i>0 and current_x_i+1<cell_state_map.shape[1] and self.is_cell_avaiable(current_x_i+1,current_y_i-1,cell_state_map)):
            cells_index_list.append((current_x_i+1,current_y_i-1))
        
        if(current_x_i+1<cell_state_map.shape[1] and self.is_cell_avaiable(current_x_i+1,current_y_i,cell_state_map)):
            cells_index_list.append((current_x_i+1,current_y_i))
        if(current_x_i+1<cell_state_map.shape[1] and current_y_i+1<cell_state_map.shape[0] and self.is_cell_avaiable(current_x_i+1,current_y_i+1,cell_state_map)):
            cells_index_list.append((current_x_i+1,current_y_i+1))
        
        if(current_y_i+1<cell_state_map.shape[0] and self.is_cell_avaiable(current_x_i,current_y_i+1,cell_state_map)):
            cells_index_list.append((current_x_i,current_y_i+1))

        if(current_y_i+1<cell_state_map.shape[0] and current_x_i>0 and self.is_cell_avaiable(current_x_i,current_y_i+1,cell_state_map)):
            cells_index_list.append((current_x_i-1,current_y_i+1))

        #rospy.loginfo("%d"%(len(cells_index_list)))
        
        return cells_index_list



    def is_cell_avaiable(self,current_x_i,current_y_i,cell_state_map):
        return cell_state_map[current_y_i][current_x_i]==0 and self.map.get_map_memmory()[current_y_i][current_x_i]!=1 and self.map.get_map_memmory()[current_y_i][current_x_i]!=2
        


if __name__ == '__main__':
    try:
        node=PathPlaner()
        

       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")

