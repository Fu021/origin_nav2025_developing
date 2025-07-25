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

q : Increase 10s
a : Decrease 10s

e : Increase red outpost HP
d : Decrease red outpost HP

r : Increase blue outpost HP
s : Decrease blue outpost HP

CTRL-C to quit
"""

keyBindings = {
    'w': 50,
    's': -50
}

time_key_bind = {
    'q': 10,
    'a': -10
}

red_outpost_key_bind = {
    'e' : 100,
    'd' : -100
}

blue_outpost_key_bind = {
    'r' : 100,
    'f' : -100
}

class RefereePub(Node):
    def __init__(self):
        super().__init__('referee_fake')
        self.publisher = self.create_publisher(Referee, '/referee', 10)
        self.remain_hp = 600
        self.min_hp = 0
        self.max_hp = 600
        self.game_progress = 4
        self.max_outpost_hp = 1500
        self.min_outpost_hp = 0
        self.red_outpost_hp = 1500
        self.blue_outpost_hp = 1500
        self.min_time = 9
        self.max_time = 419
        self.stage_remain_time = 419
        self.stop = threading.Event()
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
    def update_time(self, change):
        new_time = self.stage_remain_time + change
        if self.min_time <= new_time <= self.max_time:
            self.stage_remain_time = new_time
        print(f'Now Time: {self.stage_remain_time}')
    def update_red_outpost_hp(self, change):
        new_outpost_hp = self.red_outpost_hp + change
        if self.min_outpost_hp <= new_outpost_hp <= self.max_outpost_hp:
            self.red_outpost_hp = new_outpost_hp
        print(f'Now red outpost HP: {self.red_outpost_hp}')
    def update_blue_outpost_hp(self, change):
        new_outpost_hp = self.blue_outpost_hp + change
        if self.min_outpost_hp <= new_outpost_hp <= self.max_outpost_hp:
            self.blue_outpost_hp = new_outpost_hp
        print(f'Now blue outpost HP: {self.blue_outpost_hp}')

    def publish_referee(self):
        while not self.stop.is_set():

            msg = Referee()
            msg.remain_hp = self.remain_hp
            msg.game_progress = self.game_progress
            msg.bullet_remaining_num_17mm = 100
            msg.stage_remain_time = self.stage_remain_time
            msg.red_outpost_hp = self.red_outpost_hp
            msg.blue_outpost_hp = self.blue_outpost_hp
            self.publisher.publish(msg)
            time.sleep(0.1)
        
    
    def run(self):
        while not self.stop.is_set():

            try:
                    key = self.getKey(0.5)
                    if key in keyBindings:
                        self.update_hp(keyBindings[key])
                    elif key in time_key_bind:
                        self.update_time(time_key_bind[key])
                    elif key in red_outpost_key_bind:
                        self.update_red_outpost_hp(red_outpost_key_bind[key])
                    elif key in blue_outpost_key_bind:
                        self.update_blue_outpost_hp(blue_outpost_key_bind[key])
                    elif key == '\x03':
                        self.stop.set()
                        rclpy.shutdown()
                        break

            except Exception as e:
                print(e)
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
def main(args=None):
    print(msg)
    rclpy.init(args=args)
    node = RefereePub()
    rclpy.spin(node)
    # rclpy.shutdown()

if __name__ == "__main__":
    main()
