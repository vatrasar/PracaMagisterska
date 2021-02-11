#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

class MoveToPlace():

    def __init__(self):
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        position_topic = "/turtle1/pose"
        self.pose_subscriber = rospy.Subscriber(position_topic, Pose, self.poseCallback) 

    def poseCallback(self,pose_message):
       
        self.x= pose_message.x
        self.y= pose_message.y
        self.theta = pose_message.theta

    def move(self,speed,distance, is_foraward):

        velocity_message=Twist()
        if(is_foraward):
            velocity_message.linear.x=abs(speed)
        else:
            velocity_message.linear.x=-abs(speed)

        x0=self.x
        y0=self.y
        distance_moved=0.0
        loop_rate=rospy.Rate(10)
        while(True):
            self.velocity_publisher.publish(velocity_message)
            loop_rate.sleep()
            distance_moved = abs(0.5 * math.sqrt(((self.x-x0) ** 2) + ((self.y-y0) ** 2)))
            if(not (distance_moved<distance)):
                rospy.loginfo("reached")
                break

        #finally, stop the robot when the distance is moved
        velocity_message.linear.x =0
        self.velocity_publisher.publish(velocity_message)

    def go_to_goal(x_goal, y_goal):
       

        velocity_message = Twist()
        cmd_vel_topic='/turtle1/cmd_vel'

        while (True):
            K_linear = 0.5 
            distance = abs(math.sqrt(((x_goal-self.x) ** 2) + ((y_goal-self.y) ** 2)))

            linear_speed = distance * K_linear


            K_angular = 4.0
            desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x)
            angular_speed = (desired_angle_goal-yaw)*K_angular

            velocity_message.linear.x = linear_speed
            velocity_message.angular.z = angular_speed

            velocity_publisher.publish(velocity_message)
            
            #print velocity_message.linear.x
            #print velocity_message.angular.z
            print 'x=', x, 'y=',y


            if (distance <0.01):
                break


    def rotate (self,angular_speed_degree, relative_angle_degree, clockwise):
    
        
        velocity_message = Twist()
        velocity_message.linear.x=0
        velocity_message.linear.y=0
        velocity_message.linear.z=0
        velocity_message.angular.x=0
        velocity_message.angular.y=0
        velocity_message.angular.z=0

        #get current location 
        theta0=self.theta
        angular_speed=math.radians(abs(angular_speed_degree))

        if (clockwise):
            velocity_message.angular.z =-abs(angular_speed)
        else:
            velocity_message.angular.z =abs(angular_speed)

        angle_moved = 0.0
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
        cmd_vel_topic='/turtle1/cmd_vel'
        

        t0 = rospy.Time.now().to_sec()

        while True :
            rospy.loginfo("Turtlesim rotates")
            self.velocity_publisher.publish(velocity_message)

            t1 = rospy.Time.now().to_sec()
            current_angle_degree = (t1-t0)*angular_speed_degree
            loop_rate.sleep()


                        
            if  (current_angle_degree>relative_angle_degree):
                rospy.loginfo("reached")
                break

        #finally, stop the robot when the distance is moved
        velocity_message.angular.z =0
        self.velocity_publisher.publish(velocity_message)
    
    def spiralClean(self):
        vel_msg = Twist()
        loop_rate = rospy.Rate(1)
        wk = 4
        rk = 0
    
        while((self.x<10.5) and (self.y<10.5)):
            rk=rk+1
            vel_msg.linear.x =rk
            vel_msg.linear.y =0
            vel_msg.linear.z =0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z =wk
            self.velocity_publisher.publish(vel_msg)
            loop_rate.sleep()
    
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
    

    





if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #declare velocity publisher
        moveToPlace=MoveToPlace()

        time.sleep(2)
        #moveToPlace.move(1,3.0,True)
        #move(1.0, 2.0, False)
        #moveToPlace.rotate(30, 56, True)
        moveToPlace.spiralClean()
        #setDesiredOrientation(math.radians(90))
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")