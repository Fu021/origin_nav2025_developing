import py_trees
from ament_index_python.packages import get_package_share_directory
import yaml
from rclpy.node import Node
from py_trees.common import Status
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
from std_msgs.msg import Bool,Int32,Float32
import random

class GetDataFromYaml(py_trees.behaviour.Behaviour):
    '''
        从指定的yaml文件中读取所有的信息\n
        存放在namespace为yaml的黑板上\n
        仅运行一次
    '''
    def __init__(self, name: str, yaml_name: str, node: Node):
        super().__init__(name)
        self.yaml_name = yaml_name
        self.open_yaml = False
        self.blackboard = self.attach_blackboard_client(namespace='yaml')
        self.node = node

    def update(self):
        if not self.open_yaml:
            try:
                path = get_package_share_directory("dec_tree") + "/config/" + self.yaml_name + ".yaml"
                with open(path, 'r') as file:
                    yaml_file = yaml.safe_load(file)
                self.node.get_logger().info("yaml文件导入成功: %s" %self.yaml_name)
            except:
                self.node.get_logger().info("无法读取yaml文件")
                return Status.FAILURE
            
            self.open_yaml = True
            
            for i in yaml_file:
                self.blackboard.register_key("%s"%i,py_trees.common.Access.WRITE)
                self.blackboard.__setattr__(i, yaml_file[i])

            return Status.SUCCESS
        else:
            return Status.SUCCESS
        
class PubGoal(py_trees.behaviour.Behaviour):
    '''
        发布目标点
    '''
    def __init__(self, name: str, nav: BasicNavigator):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.blackboard.register_key("Referee",py_trees.common.Access.READ)
        self.blackboard.register_key("goal",py_trees.common.Access.READ)
        self.blackboard.register_key("running",py_trees.common.Access.WRITE)
        self.blackboard.register_key("running",py_trees.common.Access.READ)
        self.nav = nav
        self.bt_path = get_package_share_directory("dec_tree") + "/bt/"

    def update(self):
        # if self.blackboard.Referee.game_progress != 4:
        #     return Status.FAILURE
        
        if self.blackboard.running.data == True:
            return Status.FAILURE
        
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.orientation.w = 1.0
        print("正在前往目标点")
        goal_msg.pose.position.x = float(self.blackboard.goal['x'])
        goal_msg.pose.position.y = float(self.blackboard.goal['y'])

        if self.nav.goToPose(goal_msg):
            self.blackboard.running = Bool()
            self.blackboard.running.data = True

        return Status.SUCCESS

class UnpackReferee(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key('Referee',py_trees.common.Access.READ)
        self.blackboard.register_key('home_occupy',py_trees.common.Access.WRITE)
    
    def update(self):
        self.blackboard.home_occupy = (self.blackboard.Referee.rfid_status>>19)&1
        print("home_occupy: %d"%self.blackboard.home_occupy)
        return Status.SUCCESS

class Patrol(py_trees.behaviour.Behaviour):
    '''
        巡逻\n
        name不能重复\n
        points_name指定在yaml内的名称\n
        wait指定到达目标点后等待时间\n
        referee_condition额外附加裁判条件 为1后将条件通过字典传入\n
        interrupt为1可以打断running状态强制发送点位
    '''
    def __init__(self, name: str, points_name, node:Node, nav: BasicNavigator, condition_func, random=True):
        super().__init__(name)
        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.yaml.register_key(points_name,py_trees.common.Access.READ)
        self.yaml.register_key('blood_limit',py_trees.common.Access.READ)
        self.yaml.register_key('blood_limit',py_trees.common.Access.WRITE)

        # self.yaml.register_key("our_outpost",py_trees.common.Access.READ)
        self.yaml.register_key("our_color",py_trees.common.Access.READ)
        # self.yaml.register_key("their_outpost",py_trees.common.Access.READ)

        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("goal",py_trees.common.Access.WRITE)
        self.blackboard.register_key("dec_now",py_trees.common.Access.WRITE)
        self.blackboard.register_key("running",py_trees.common.Access.READ)
        self.blackboard.register_key("reach_goal",py_trees.common.Access.READ)
        self.blackboard.register_key("Referee",py_trees.common.Access.READ)
        self.blackboard.register_key('home_occupy',py_trees.common.Access.READ)
        self.points = []
        self.name = name
        self.points_name = points_name
        self.lens = 0
        self.point_now = 0
        self.condition_func = condition_func
        self.node = node
        self.blackboard.dec_now = None
        self.nav = nav
        self.bullet_remain_last=0
        self.got_bullet = False
        self.got_bullet_in_final_minute=False
        self.is_in_12s_home_wait=False         #如果回家以后距离下一次弹丸发放不足10s,就触发12s等待，拿到弹丸后打破等待
        self.home_wait_start_time = 0  # 记录进入12秒等待期的时间戳
        self.waiting_for = None       # 等待的原因（如 'normal' 或 'home_phase_12s'）
        self.wait_until = 0           # 等待截止时间
        self.random =random
    def condition(self):
        if self.condition_func(self):
            return True
        return False
        # if self.blackboard.Referee.game_progress != 4:
        #     return False
        # is_hp_full = (self.blackboard.Referee.remain_hp >= 400)
        # is_hp_low = (self.blackboard.Referee.remain_hp < self.yaml.blood_limit)
        # is_bullet_low = (self.blackboard.Referee.bullet_remaining_num_17mm < 75)
        # is_bullet_empty = (self.blackboard.Referee.bullet_remaining_num_17mm <= 0)
        # is_final_minute = (self.blackboard.Referee.stage_remain_time <=62)
        # self.got_bullet = ((self.blackboard.Referee.bullet_remaining_num_17mm - self.bullet_remain_last > 50) and self.blackboard.home_occupy != 0) #在家里这一刻拿到弹了
        # print(f"got_bullet_in_final_minute:{self.got_bullet_in_final_minute},{self.got_bullet}")
        # if self.referee_condition == 'home':  # True： 需要回家
        #     '''
        #     需要回家需要满足的条件：
        #     case1： 比赛前六分钟没血或没弹就回家，直到血量满且子弹足

        #     case2： 进入最后一分钟，没有在最后一分钟拿到弹，且血低或弹尽，直到血回满并且拿到弹再走

        #     case3： 最后一分钟并且在最后一分钟拿到过弹后，仅血量不足回家,血回满再走

        #     都需要进行的：判断当前在补给区，并且距离下一波发弹的时间小于10s,则等待12s


        #     '''


        #     if is_final_minute:
        #         if self.got_bullet:
        #             self.got_bullet_in_final_minute  = True
                
        #     # Case 1: 进入最后一分钟且未拿到过弹，且血低或弹尽，直到血回满并且拿到弹再走
        #         if not self.got_bullet_in_final_minute :
        #             # 若 血量低且弹量为空继续发点
        #             if is_hp_low or is_bullet_empty :
        #                 return True
        #             # 若 血量没有回满且弹量low继续发点
        #             elif ((not is_hp_full) or is_bullet_low) and self.blackboard.dec_now == 'goto_home': 
        #                 return True
        #     # Case 2: 最后一分钟并且拿到过弹后，仅血量不足回家,血回满再走
        #         else :
        #             if is_hp_low:
        #                 return True
        #             elif (not is_hp_full) and self.blackboard.dec_now == 'goto_home':
        #                 return True
                
            
        #     else: #比赛前六分钟

        #         if is_hp_low or is_bullet_empty: #没血或者没弹
        #             return True
        #         elif self.waiting_for == "home_phase_12s":
        #             return True
        #         elif ((not is_hp_full) or is_bullet_low) and self.blackboard.dec_now == 'goto_home': #血量没回满或者子弹不足，继续在家呆着
        #             return True
            
        # elif self.referee_condition == 'outpost':
        #     their_color = 'red'
        #     if self.yaml.our_color == 'red':
        #         their_color = 'blue'
        #     if getattr(self.blackboard.Referee,their_color + '_outpost_hp') >= 0 and self.blackboard.Referee.stage_remain_time<=360:
        #         return True
            
        # elif self.referee_condition == 'peek':
        #     # True： 需要去peek
        #     their_color = 'red'
        #     if self.yaml.our_color == 'red':
        #         their_color = 'blue'
        #     if getattr(self.blackboard.Referee,their_color + '_outpost_hp') <= 0:
        #         return True
            
        
        # elif self.referee_condition == 'mid':
        #     #需要去mid
        #     their_color = 'red'
        #     if self.yaml.our_color == 'red':
        #         their_color = 'blue'
        #     if self.blackboard.Referee.stage_remain_time>=360:
        #         return True
            

        # return False 


        # if self.referee_condition == 1:
        #     if self.blackboard.Referee.bullet_remaining_num_17mm <= 0:
        #         if self.points == []:
        #             self.points = self.yaml.__getattr__(self.points_name)

        #         self.len = len(self.points)

        #         if self.blackboard.Referee.game_progress!=4:
        #             return False

        #         return True

        #     if getattr(self.blackboard.Referee,self.yaml.our_color + '_outpost_hp') > self.yaml.our_outpost:
        #         return False
        # elif self.referee_condition == 2:
        
        # if self.points == []:
        #     self.points = self.yaml.__getattr__(self.points_name)

        # self.len = len(self.points)

        # if self.blackboard.Referee.game_progress!=4:
        #     return False

        # return True
    
    def init_dec(self):
        self.blackboard.dec_now = self.name
        self.point_now = self.points[0]
        self.wait_begin = False
        self.end_time = 0
        random.seed(time.time())
        while not self.nav.isTaskComplete():
            self.nav.cancelTask()

    def go_to_next(self):
        if self.len == 1:
            return
        if self.random:
            tmp = random.choice(self.points)
            while tmp == self.point_now:
                tmp = random.choice(self.points)
            self.point_now = tmp
        else :
            self.point_now=self.points[(self.points.index(self.point_now)+1)%self.len]

    def update(self):
        
        
        # 初始条件
        condition= self.condition()
        self.bullet_remain_last=self.blackboard.Referee.bullet_remaining_num_17mm
        if not condition:
            return Status.FAILURE
                   
        self.points = self.yaml.__getattr__(self.points_name)
        self.len = len(self.points)
        
        # 初始化决策
        print(self.blackboard.dec_now,self.name)
        if self.blackboard.dec_now != self.name:
            self.init_dec()
            self.blackboard.goal = self.point_now
            self.node.get_logger().info("%s: send goal x:%f y:%f"%(self.name,self.point_now['x'],self.point_now['y']))
            if self.blackboard.dec_now == 'goto_outpost' or self.blackboard.dec_now == 'goto_mid':
                self.yaml.blood_limit = 200
            else :
                self.yaml.blood_limit = 150


            return Status.SUCCESS
        
        if self.blackboard.running.data == True:
            self.node.get_logger().info("running")
            return Status.SUCCESS
        print(self.blackboard.Referee.stage_remain_time % 60)
        # 分成4种情况
        # 1. 到达点位，reach_goal为true，则开启wait_begin;
        # 2. 未到达，继续发点
        # 3. wait_begin已经开启，时间未达到，直接继续发送当前点位
        # 4. wait_begin已经开启，时间达到，进入go_to_next尝试发送下一点位

        # 特殊情况
        # 当前在补给区，并且距离下一波发弹的时间小于10s,则等待12s

        #开启等待后检查等待是否结束
        if self.waiting_for is not None:        
            if time.time() > self.wait_until:
                if self.waiting_for == "home_phase_12s":
                    self.is_in_12s_home_wait = False
                self.waiting_for = None
                self.go_to_next()
                self.blackboard.goal = self.point_now
                self.node.get_logger().info("%s: send goal x:%f y:%f" % (self.name, self.point_now['x'], self.point_now['y']))
                return Status.SUCCESS
            else:
                self.node.get_logger().info("waiting for %s..." % self.waiting_for)
                return Status.SUCCESS
        
        # 检查是否需要进入12秒强制等待阶段
        elif self.blackboard.Referee.stage_remain_time % 60 <= 10 and \
            self.blackboard.home_occupy != 0 and \
            not self.is_in_12s_home_wait:

            self.is_in_12s_home_wait = True
            self.waiting_for = "home_phase_12s"
            self.wait_until = time.time() + 12
            return Status.SUCCESS
        
        # 正常到达目标后等待
        elif self.blackboard.reach_goal:
            self.waiting_for = "normal"
            tmp = 7.0
            self.wait_until = time.time() + tmp
    
        # 默认情况：发送当前目标点
        else:
            self.blackboard.goal = self.point_now
            self.node.get_logger().info("%s: send goal x:%f y:%f" % (self.name, self.point_now['x'], self.point_now['y']))

        return Status.SUCCESS
    
class CheckNavState(py_trees.behaviour.Behaviour):
    '''
        检查导航状态
    '''
    def __init__(self, name: str, nav: BasicNavigator, node: Node):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("running",py_trees.common.Access.READ)
        self.blackboard.register_key("running",py_trees.common.Access.WRITE)
        self.blackboard.register_key("reach_goal",py_trees.common.Access.WRITE)
        self.blackboard.register_key("dec_now",py_trees.common.Access.READ)


        self.nav = nav
        self.blackboard.running = Bool()
        self.blackboard.running.data = False
        self.node = node
        self.blackboard.reach_goal = False


    def update(self):
        if not self.blackboard.running.data:
            return Status.SUCCESS
        
        if self.nav.isTaskComplete():
            self.blackboard.running.data = False

            if self.nav.getResult() == TaskResult.SUCCEEDED:
                self.node.get_logger().info("success")
                self.blackboard.reach_goal = True
            else:
                self.node.get_logger().info("fail")
                self.nav.clearAllCostmaps()
                self.blackboard.reach_goal = False
        else:
            pass
            #print(self.nav.getFeedback())

        return Status.SUCCESS
    
class YawDec(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.yaml.register_key("our_color",py_trees.common.Access.READ)
        self.blackboard.register_key("Referee",py_trees.common.Access.READ)
        self.blackboard.register_key("yaw",py_trees.common.Access.WRITE)
        self.blackboard.register_key('cmd_vel',py_trees.common.Access.READ)
        self.blackboard.register_key("running",py_trees.common.Access.READ)

        self.blackboard.yaw = Float32()
        self.blackboard.yaw.data = 0.0

    def update(self):
        self.blackboard.yaw.data = 1.5

        return Status.SUCCESS
    
class PitchDec(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("dec_now",py_trees.common.Access.READ)
        self.blackboard.register_key("pitch",py_trees.common.Access.WRITE)
        self.blackboard.register_key("reach_goal",py_trees.common.Access.READ)
        self.blackboard.register_key("outpost_attack",py_trees.common.Access.READ)
        self.blackboard.register_key("Referee",py_trees.common.Access.READ)


        self.blackboard.pitch = Bool()
        self.blackboard.pitch.data = False
        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.yaml.register_key("our_color",py_trees.common.Access.READ)


    def update(self):
        their_color = 'red'
        if self.yaml.our_color == 'red':
            their_color = 'blue'
        if self.blackboard.dec_now == 'goto_mid' and self.blackboard.reach_goal and getattr(self.blackboard.Referee,their_color + '_outpost_hp') > 0:
            self.blackboard.pitch.data = True
        else:
            self.blackboard.pitch.data = False

        return Status.SUCCESS
    
class OutpostAttackDec(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("outpost_attack",py_trees.common.Access.WRITE)
        self.blackboard.register_key("Referee",py_trees.common.Access.READ)

        self.blackboard.outpost_attack = Bool()
    
    def update(self):
        # if self.blackboard.Refere <= 360:
        self.blackboard.outpost_attack.data = True
        return Status.SUCCESS