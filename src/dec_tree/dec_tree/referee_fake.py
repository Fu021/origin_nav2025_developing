import threading
import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from referee_msg.msg import Referee

msg = """
Reading from the keyboard and Publishing to Referee
---------------------------
w : Increase HP
s : Decrease HP

CTRL-C to quit
"""

keyBindings = {
    'w': 50,
    's': -50
}

class RefereePub(Node):
    def __init__(self):
        super().__init__('referee_fake')
        self.publisher = self.create_publisher(Referee, '/referee', 10)
        self.remain_hp = 600
        self.min_hp = 0
        self.max_hp = 600
        self.game_progress = 4
        self.settings = termios.tcgetattr(sys.stdin)
        self.getKeyThread= threading.Thread(target=self.run).start()
        self.publish_refereeThread = threading.Thread(target=self.publish_referee).start()

    def getKey(self, timeout):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def update_hp(self, change):
        new_hp = self.remain_hp + change
        if self.min_hp <= new_hp <= self.max_hp:
            self.remain_hp = new_hp
        print(f'Now HP: {self.remain_hp}')
    
    def publish_referee(self):
        while True:
            msg = Referee()
            msg.remain_hp = self.remain_hp
            msg.game_progress = self.game_progress
            msg.bullet_remaining_num_17mm = 100
            self.publisher.publish(msg)
            time.sleep(0.1)
    
    def run(self):
        try:
            while (1):
                key = self.getKey(0.5)
                if key in keyBindings:
                    self.update_hp(keyBindings[key])
                elif key == '\x03':
                    break
        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    print(msg)
    rclpy.init(args=args)
    node = RefereePub()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
