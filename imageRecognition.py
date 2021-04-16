#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from PIL import Image as Image2
import numpy as np
from numpy import savetxt

class ImageRecognition():
    def __init__(self):
        self.node=rospy.init_node("imageRecognitionNode",anonymous=True)
        self.subTargetPos=rospy.Subscriber("frontCameraData",Image,self.get_image)
        self.is_one=False
        rospy.spin()
    
    def get_image(self,image_msg:Image):
        img_data=image_msg.data
        # rospy.loginfo("tablica %d, height %d, width %d"%(len(img_data),image_msg.height,image_msg.width))
        result_img=self.convert_to_numpy_array(img_data,image_msg.width)
        
        result_img=Image2.fromarray(result_img, 'RGB')
        if not(self.is_one):
            self.is_one=True
            #self.convert_csv(img_data,image_msg.width)
            result_img.save("/home/szymon/test.png")

    def convert_csv(self,arrray,row_size):
        result=[]
        row_list=[]
        for channel in arrray:
            row_list.append(channel)
            if(len(row_list)%(row_size*3)==0):
                result.append(row_list)
                row_list=[]

        
        result=np.array(result)
        result=result.astype(int)
        rospy.loginfo(result)
        savetxt("/home/szymon/test.csv",result,delimiter=';',fmt="%f")
        rospy.loginfo(result.shape)

        return result

    def convert_to_numpy_array(self, array,row_size):
        chanells_list=[]

        pixels_list=[]
        for color in array:
            chanells_list.append(color)
            if len(chanells_list)%3==0 and len(chanells_list)>0:
                pixels_list.append(chanells_list)
                chanells_list=[]
        
        result_list=[]
        row_list=[]
        for pixel in pixels_list:
            row_list.append(pixel)
            if(len(row_list)%row_size==0):
                result_list.append(row_list)
                row_list=[]


        result=np.uint8(np.array(result_list))
        
        rospy.loginfo(result.shape)

        return result
        
            




if __name__ == '__main__':
    try:
        imageRecognition=ImageRecognition()
        

        

       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")