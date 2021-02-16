import math
import tf
import numpy as np

def convert_to_360(angle):
    if(angle<0):
        angle=2*3.14+angle
    return angle

def convert_to_180(angle):
    result_anlge=angle
    if(angle>3.14):
        result_anlge=-(3.14*2-angle)
    if(angle<-3.14):
        result_anlge=(3.14*2+angle)
    return result_anlge

    
def rotate_2d_vector(vector,angle):
    new_x=math.cos(angle)*vector[0]-math.sin(angle)*vector[1]
    new_y=math.sin(angle)*vector[0]+math.cos(angle)*vector[1]
    return (new_x,new_y)

def get_transform_between_points(source,target):
    transform=(target[0]-source[0],target[1]-source[1],target[2]-source[2])
    
    return transform









