import rospy
from geometry_msgs.msg import Point

class MoveToTarget():
    def __init__(self):
        self.subTargetPos=rospy.Subscriber("targetPosition",Point,self.get_target_position)
        self.target_pos=None
        self.has_first_target_message=False
    def isReady(self):
        return self.has_first_target_message
    def get_target_position(self,msg):
        self.target_pos=[msg.x,msg.y,msg.z]
        self.has_first_target_message=True
    def get_next_target(self):
        return self.target_pos