import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64


class RosDroneComunicator():

    def __init__(self):
        self.pub_move_vector=rospy.Publisher("droneMoveVector",Point,queue_size=1)

        self.pubDroneStatic=rospy.Publisher("droneStaticRotation",Float64,queue_size=1)

        
        self.node=rospy.init_node("keyControlNode",anonymous=True)
        self.target_pos=[10,10,10]
        self.drone_pos=[-1,-1,-1]

        self.subTargetPos=rospy.Subscriber("targetPosition",Point,self.get_target_position)
        self.subDronePos=rospy.Subscriber("dronePosition",Point,self.get_drone_position)
        self.subOrientation=rospy.Subscriber("eulerInfo",Float64,self.get_drone_orientation)
        self.has_first_target_message=False
        self.theta=0

        self.static_rotation_speed=0.5
        self.plus_static_rotation_speed=self.static_rotation_speed
        self.minus_static_rotation_speed=-self.static_rotation_speed
        self.drone_move_max_speed=0.2
        



    

    def get_drone_position(self,msg):
        
        self.drone_pos=[msg.x,msg.y,msg.z]


    def get_drone_orientation(self,msg):
       
        self.theta=msg.data


    def get_target_position(self,msg):
        
        self.target_pos=[msg.x,msg.y,msg.z]
        self.has_first_target_message=True