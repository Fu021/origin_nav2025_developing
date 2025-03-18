#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,Float32
from referee_msg.msg import Referee

class NavNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.sub_referee = self.create_subscription(Referee,"/Referee",self.ref_callback,10)
        self.sub_cmd = self.create_subscription(Twist,"/cmd_vel_chassis",self.cmd_callback,10)
        self.pub_cmd = self.create_publisher(Twist,"/nav_cmd",qos_profile=rclpy.qos.qos_profile_sensor_data)
        self.sub_running = self.create_subscription(Bool,"/running_state",self.running_callback,10)
        self.running = True
        self.ref = Referee()
        #self.timer = self.create_timer(0.1,self.rot_callback)
        #self.pub_rot = self.create_publisher(Bool,"/nav_rotate",10)

    def cmd_callback(self,msg:Twist):
        # if self.ref.game_progress != 4:
        #     new_cmd = Twist()
        #     new_cmd.linear.x = 0.0
        #     new_cmd.linear.y = 0.0
        #     new_cmd.angular.z = 0.0
        #     self.pub_cmd.publish(new_cmd)
        #     return
        # if self.running:
        msg.linear.x = msg.linear.x
        msg.linear.y = msg.linear.y
        msg.angular.z = msg.angular.z
        msg.angular.z *= 1000
        self.pub_cmd.publish(msg)

    def running_callback(self,msg:Bool):
        if self.ref.game_progress != 4:
            return

        self.running = msg.data
        if self.running == False:
            new_cmd = Twist()
            new_cmd.angular.z = 2.0*1000
            self.pub_cmd.publish(new_cmd)

    def rot_callback(self):
        if self.ref.game_progress != 4:
            return

        # new_cmd = Twist()
        # new_cmd.angular.z = 3.0 * 1000
        # self.pub_cmd.publish(new_cmd)
        new_rot = Bool()
        new_rot.data = True
        self.pub_rot.publish(new_rot)

    def ref_callback(self,msg:Referee):
        self.ref = msg

def main(args = None):
    rclpy.init(args=args)
    node = NavNode(name="nav_node")
    rclpy.spin(node=node)