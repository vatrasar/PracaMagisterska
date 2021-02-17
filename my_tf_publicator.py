#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import numpy as np
import tf
from tools import get_transform_between_points
from tools import get_transform_between_orientations
from tools import rotate_2d_vector
from tfTools import get_orientation_from_pose
from tfTools import get_transform_vector_from_pose
import time
class TFPublicatorNode():
    def __init__(self):
        self.node=rospy.init_node("my_tf_publicator",anonymous=True)
        self.subTfBase_link=rospy.Subscriber("TfFootPrint",Pose,self.get_foot_print_data)
        self.subTf_sensor_link=rospy.Subscriber("TfSensor",Pose,self.get_sensor_link_data)
        #self.pubLaser=rospy.Publisher("myLaserInfo",LaserScan,queue_size=1)
        self.transform_broadcaster=tf.TransformBroadcaster()
        self.foot_print=None
        self.sensor_data=None
        self.rate=rospy.Rate(2)
        self.init_odom_pose=None
        self.publication()
        
        #rospy.spin()

    def get_foot_print_data(self,msg):
        self.foot_print=msg
        #rospy.loginfo("odebrane")
        if(self.init_odom_pose==None):
            self.init_odom_pose=msg

    def get_sensor_link_data(self,msg):
        self.sensor_data=msg
        #rospy.loginfo("odebrane")
        

    def publication(self):
        #rospy.loginfo("przedCzasem")
        #time.sleep(2)

        #rospy.loginfo("przedPetla")
        while(not rospy.is_shutdown()):
            #rospy.loginfo("pocz")
            if self.foot_print!=None and self.sensor_data!=None and self.init_odom_pose!=None:
                # eurel_rotation=tf.transformations.euler_from_quaternion((self.foot_print.orientation.x,self.foot_print.orientation.y,self.foot_print.orientation.z,self.foot_print.orientation.w))
                # rotation_quaterion=tf.transformations.quaternion_from_euler(0.0,0.0,eurel_rotation[2])
                # translation_vector=(self.foot_print.position.x,self.foot_print.position.y,self.foot_print.position.z)
                odom_to_current_translation_vector,odom_to_current_rotation_quaterion=self.get_odom_to_current_tf()
                current_time=rospy.Time.now()
                #rospy.loginfo("time:%s"%(current_time))
                map_to_odom_translation_vector=get_transform_vector_from_pose(self.init_odom_pose)
                map_to_odom_rotation_quaterion=get_orientation_from_pose(self.init_odom_pose)

                sensor_to_base_link_pose=get_transform_vector_from_pose(self.sensor_data)
                sensor_to_base_link_orientation=get_orientation_from_pose(self.sensor_data)
                #orginal_translation_vector=self.get_transform_vector_from_pose(self.foot_print)
                #orginal_rotation_quaterion=self.get_orientation_from_pose(self.foot_print)
                #rospy.loginfo(odom_to_current_rotation_quaterion)
                #rospy.loginfo("orginal footprint:"+str(orginal_translation_vector))
                #rospy.loginfo("init odom:"+str(map_to_odom_translation_vector))
                #rospy.loginfo("reult:"+str(odom_to_current_translation_vector))
                self.transform_broadcaster.sendTransform(odom_to_current_translation_vector,odom_to_current_rotation_quaterion,current_time,"base_link","odom")
                #self.transform_broadcaster.sendTransform((0,0,0),(0,0,0,0),current_time,"laser","base_link")
                #self.transform_broadcaster.sendTransform(map_to_odom_translation_vector,map_to_odom_rotation_quaterion,current_time,"odom","map")

                self.transform_broadcaster.sendTransform(sensor_to_base_link_pose,sensor_to_base_link_orientation,current_time,"sensor_link","base_link")
                #rospy.loginfo("publikacja")
                time.sleep(1/100.0)
                #rospy.loginfo("close")
        rospy.loginfo("close")




    def get_odom_to_current_tf(self):
        init_transform=self.get_transform_vector_from_pose(self.init_odom_pose)
        init_orientation=self.get_orientation_from_pose(self.init_odom_pose)

        init_orientation_euler=tf.transformations.euler_from_quaternion(init_orientation)


        current_transform=self.get_transform_vector_from_pose(self.foot_print)
        current_orientation=self.get_orientation_from_pose(self.foot_print)
        current_transform=current_transform
        transform_vector_from_init_to_current=get_transform_between_points(init_transform,current_transform)
        transform_orientation_from_init_to_current=get_transform_between_orientations(init_orientation,current_orientation)
        transform_vector_from_init_to_current=rotate_2d_vector(transform_vector_from_init_to_current,-init_orientation_euler[2])
        transform_vector_from_init_to_current=(transform_vector_from_init_to_current[0],transform_vector_from_init_to_current[1],0)
        return (transform_vector_from_init_to_current,transform_orientation_from_init_to_current)

    


if __name__ == "__main__":
    #rospy.loginfo("main")
    newNode=TFPublicatorNode()
    
    
    