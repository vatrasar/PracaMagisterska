import math
import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

def get_3d_point_moved_using_vector(vector,point):
    result_point=(point[0]+vector[0],point[1]+vector[1],point[2]+vector[2])
    return result_point

def get_transform_between_orientations(source,target):
    
    source_euler=tf.transformations.euler_from_quaternion(source)
    target_euler=tf.transformations.euler_from_quaternion(target)
    source_z=convert_to_360(source_euler[2])
    target_z=convert_to_360(target_euler[2])
    target_euler=list(target_euler)
    target_euler[2]=convert_to_180(target_z-source_z)
    return tf.transformations.quaternion_from_euler(target_euler[0],target_euler[1],target_euler[2])

def get_2d_vector_from_polar(angle,distance):
	x=distance*np.cos(angle)
	y=distance*np.sin(angle)
	return (x,y,0)

def get_3d_vector_from_polar(angle,theta,distance):
    
    z=distance*math.sin(theta)
    x=distance*math.cos(theta)*math.cos(angle)
    y=distance*math.cos(theta)*math.sin(angle)
    return (x,y,z)


def get_distance(source,position):
    x_target,y_target,z_target=position
    x_source,y_source,z_source=source
    
    p1 = np.array([x_target,y_target,z_target])
    p2 = np.array([x_source,y_source,z_source])


    squared_dist = np.sum((p1-p2)**2, axis=0)
    dist = np.sqrt(squared_dist)

    return dist

def get_2d_distance(source,position):
    x_target,y_target=position
    x_source,y_source=source
    
    p1 = np.array([x_target,y_target])
    p2 = np.array([x_source,y_source])


    squared_dist = np.sum((p1-p2)**2, axis=0)
    dist = np.sqrt(squared_dist)

    return dist

def get_target_ros_pos(target_coordinates):
    point=Point()
    point.x=target_coordinates[0]
    point.y=target_coordinates[1]
    point.z=target_coordinates[1]
    return point


def convert_to_360(angle):
    if(angle<0):
        angle=2*3.14+angle
    return angle

def get_vector_angle(vector):
    vector_2=vector
    vector_1=[1,0]
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    det_produtcst=np.cross(unit_vector_1,unit_vector_2)
    angle = np.arctan2(det_produtcst,dot_product)
    return angle


def get_ros_point(vector):
    point=Point()
    point.x=vector[0]
    point.y=vector[1]
    point.z=vector[2]
    return point

def rotate_vector(vector,angle):
    new_x=math.cos(angle)*vector[0]-math.sin(angle)*vector[1]
    new_y=math.sin(angle)*vector[0]+math.cos(angle)*vector[1]
    return (new_x,new_y,vector[2])

def get_angle_to_target(target_point,drone_pos):
    absolute_vector=get_absolute_vector_to_target(drone_pos,target_point)
    xy_vector=[absolute_vector[0],absolute_vector[1]]
    angle_target=convert_to_360(get_vector_angle(xy_vector))
    return angle_target


def get_absolute_vector_to_target(current_position,target_position):
    (x_current,y_current,z_current)=current_position
    
    (x_target,y_target,z_target)=target_position
    
    result=(x_target-x_current,y_target-y_current,z_target-z_current)
    return result

def get_absolute_vector_to_target(current_position,target_position):
    (x_current,y_current,z_current)=current_position
    
    (x_target,y_target,z_target)=target_position
    
    result=(x_target-x_current,y_target-y_current,z_target-z_current)
    return result


def get_vector_to_target(current_position,target_position,theta):
    

    (x_current,y_current,z_current)=current_position
    (x_target,y_target,z_target)=target_position
    nonNormalizetVector=(x_target-x_current,y_target-y_current,z_target-z_current)
    resultVector=nonNormalizetVector
    angle_z=theta
    result_vector_after_rotation=rotate_vector((resultVector[0],resultVector[1],resultVector[2]),-angle_z)
    return result_vector_after_rotation


def get_vector_with_length_and_direction(drone_move_max_speed,direction_vector):
    sum_of_squares=direction_vector[0]**2+direction_vector[1]**2+direction_vector[2]**2
    scale_factor=drone_move_max_speed/math.sqrt(sum_of_squares)
    result_vector=[direction_vector[0],direction_vector[1],direction_vector[2]]
    result_vector[0]=result_vector[0]*scale_factor
    result_vector[1]=result_vector[1]*scale_factor
    result_vector[2]=result_vector[2]*scale_factor
    return result_vector



