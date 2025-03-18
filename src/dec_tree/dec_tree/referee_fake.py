import rclpy
from referee_msg.msg import Referee
from rclpy.node import Node

class RefereePub(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub_referee = self.create_publisher(Referee,"/Referee",10)
        self.ref_timer = self.create_timer(0.1,self.ref_callback)

    def ref_callback(self):
        ref = Referee()
        ref.game_progress = 4
        ref.red_outpost_hp = 1000
        ref.blue_outpost_hp = 0
        self.pub_referee.publish(ref)

def main(args=None):
    rclpy.init(args=args)
    node = RefereePub("referee_fake")
    rclpy.spin(node)
    rclpy.shutdown()
