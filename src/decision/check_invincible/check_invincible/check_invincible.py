import rclpy
from rclpy.node import Node
from enum import Enum
import rclpy.qos
from std_msgs.msg import Bool
from referee_msg.msg import Referee
from rm_interfaces.msg import Target

class EnemyHpStatus(Enum):
    NORMAL = 0
    DEAD = 1
    REVIVING = 2
    COIN_REVIVING = 3


class Invicible(Node):
    def __init__(self,name:str):
        super().__init__(node_name=name)
        self.sub_referee = self.create_subscription(Referee, '/Referee', self.referee_callback,rclpy.qos.qos_profile_system_default)
        self.sub_target = self.create_subscription(Target, '/front/armor_solver/target', self.target_callback,rclpy.qos.qos_profile_sensor_data)
        self.pub_inv=self.create_publisher(Bool,"skip_aim",10)
        self.time_period= 0.05
        self.tiemr=self.create_timer(self.time_period,self.time_callback)
        self.our_color = "blue"
        self.their_color = "red" if self.our_color == "blue" else "blue"
        self.skip_aim = False
        self.Referee_data = Referee()
        self.front_camera_aim = Target()
        self.hp_list = {
            "1" : ["1_hp",EnemyHpStatus.NORMAL,0,100],
            "2" : ["2_hp",EnemyHpStatus.NORMAL,0,100],
            "3" : ["3_hp",EnemyHpStatus.NORMAL,0,100],
            "4" : ["4_hp",EnemyHpStatus.NORMAL,0,100],
            "sentry" : ["7_hp",EnemyHpStatus.NORMAL,0,100],
        }
    def time_callback(self):
        msg=Bool()
        msg.data= self.skip_aim
        self.pub_inv.publish(msg)
    def update_enemy_hp_status(self,enemy_name, hp_type , reviving_time, last_hp, invicible_time):
        enemy_hp = getattr(self.Referee_data,self.their_color + "_" + enemy_name) 
        if self.their_color + "_" + enemy_name == 'red_1_hp':
            return [enemy_name,EnemyHpStatus.NORMAL,0,enemy_hp]
        match(hp_type):
            case EnemyHpStatus.NORMAL:
                if enemy_hp == 0:
                    return [enemy_name,EnemyHpStatus.DEAD,0,enemy_hp]
                return [enemy_name,EnemyHpStatus.NORMAL,0,enemy_hp]
            case EnemyHpStatus.DEAD:
                if 0< enemy_hp < 100:
                    return [enemy_name,EnemyHpStatus.REVIVING,self.Referee_data.stage_remain_time-invicible_time,enemy_hp]
                elif enemy_hp >= 100:
                    return [enemy_name,EnemyHpStatus.COIN_REVIVING,self.Referee_data.stage_remain_time-3,enemy_hp]
                return [enemy_name,EnemyHpStatus.DEAD,0,enemy_hp]
            case EnemyHpStatus.REVIVING:
                if self.Referee_data.stage_remain_time <= reviving_time or enemy_hp > last_hp: # 可以击打
                    return [enemy_name,EnemyHpStatus.NORMAL,0,enemy_hp] 
                return [enemy_name,EnemyHpStatus.REVIVING, reviving_time,enemy_hp]
            case EnemyHpStatus.COIN_REVIVING:
                if self.Referee_data.stage_remain_time <= reviving_time: # 可以击打
                    return [enemy_name,EnemyHpStatus.NORMAL,0,enemy_hp] 
                return [enemy_name,EnemyHpStatus.REVIVING, reviving_time,enemy_hp]
            
    def target_callback(self,msg:Target):
        self.front_camera_aim = msg
        return
    
    def referee_callback(self,msg:Referee):
        try:
            self.Referee_data = msg
            if self.Referee_data.game_progress != 4 or self.Referee_data.stage_remain_time >415 or self.Referee_data.stage_remain_time  == 0: # 比赛并未开始
                return
            for key, value in self.hp_list.items():
                if key == "sentry":
                    self.hp_list[key] = self.update_enemy_hp_status(value[0],value[1],value[2],value[3],60)
                else:
                    self.hp_list[key] = self.update_enemy_hp_status(value[0],value[1],value[2],value[3],10)
            if self.front_camera_aim.id == '' or self.front_camera_aim.id not in list(self.hp_list.keys()):
                self.skip_aim=False
            elif self.hp_list[self.front_camera_aim.id][1] == EnemyHpStatus.REVIVING or \
                self.hp_list[self.front_camera_aim.id][1] == EnemyHpStatus.COIN_REVIVING: # 无敌不能打弹
                self.skip_aim= True
            else :
                self.skip_aim=False
        except:
            self.skip_aim=False
        return

def main(args=None):
    rclpy.init(args=args)
    node=Invicible("check_invicible")
    rclpy.spin(node)
    rclpy.shutdown()