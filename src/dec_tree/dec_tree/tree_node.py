import py_trees
from ament_index_python.packages import get_package_share_directory
import yaml
from rclpy.node import Node
from py_trees.common import Status
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
from std_msgs.msg import Bool
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
                self.node.get_logger().info("yaml文件导入成功")
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
        self.blackboard.register_key("controller",py_trees.common.Access.READ)
        self.nav = nav
        self.bt_path = get_package_share_directory("dec_tree") + "/bt/"
        self.wait_time = 0

    def update(self):
        if self.blackboard.Referee.game_progress != 4:
            return Status.FAILURE
        
        if self.blackboard.running.data == True:
            return Status.FAILURE
        
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.orientation.w = 1.0
        print("正在前往目标点")
        goal_msg.pose.position.x = float(self.blackboard.goal['x'])
        goal_msg.pose.position.y = float(self.blackboard.goal['y'])

        if time.time() < self.wait_time:
            return Status.FAILURE

        if self.nav.goToPose(goal_msg):
            self.blackboard.running = Bool()
            self.blackboard.running.data = True
            self.wait_time = time.time() + 0.5

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
    def __init__(self, name: str, points_name, node:Node, nav: BasicNavigator, controller, referee_condition=0):
        super().__init__(name)
        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.yaml.register_key(points_name,py_trees.common.Access.READ)
        self.yaml.register_key("our_outpost",py_trees.common.Access.READ)
        self.yaml.register_key("our_color",py_trees.common.Access.READ)
        self.yaml.register_key("their_outpost",py_trees.common.Access.READ)

        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("goal",py_trees.common.Access.WRITE)
        self.blackboard.register_key("dec_now",py_trees.common.Access.WRITE)
        self.blackboard.register_key("dec_now",py_trees.common.Access.READ)
        self.blackboard.register_key("running",py_trees.common.Access.READ)
        self.blackboard.register_key("reach_goal",py_trees.common.Access.READ)
        self.blackboard.register_key("controller",py_trees.common.Access.WRITE)
        self.blackboard.register_key("Referee",py_trees.common.Access.READ)

        self.points = []
        self.name = name
        self.points_name = points_name
        self.lens = 0
        self.point_now = 0
        self.referee_condition = referee_condition
        self.node = node
        self.blackboard.dec_now = None
        self.controller = controller
        self.wait_begin = False
        self.nav = nav
        self.end_time = 0

    def condition(self):
        if self.referee_condition == 1:
            # if self.blackboard.Referee.bullet_remaining_num_17mm <= 0:
            #     if self.points == []:
            #         self.points = self.yaml.__getattr__(self.points_name)

            #     self.len = len(self.points)

            #     if self.blackboard.Referee.game_progress!=4:
            #         return False

            #     return True

            if getattr(self.blackboard.Referee,self.yaml.our_color + '_outpost_hp') > self.yaml.our_outpost:
                return False
        elif self.referee_condition == 2:
            their_color = 'red'
            if self.yaml.our_color == 'red':
                their_color = 'blue'
            if getattr(self.blackboard.Referee,their_color + '_outpost_hp') > self.yaml.their_outpost:
                return False
            if getattr(self.blackboard.Referee,self.yaml.our_color + '_outpost_hp') <= self.yaml.our_outpost:
                return False
        
        if self.points == []:
            self.points = self.yaml.__getattr__(self.points_name)

        self.len = len(self.points)

        if self.blackboard.Referee.game_progress!=4:
            return False

        return True
    
    def init_dec(self):
        self.blackboard.dec_now = self.name
        self.blackboard.controller = self.controller
        self.point_now = self.points[0]
        self.wait_begin = False
        self.end_time = 0
        random.seed(time.time())
        while not self.nav.isTaskComplete():
            self.nav.cancelTask()

    def go_to_next(self):
        if self.len == 1:
            return
        tmp = random.choice(self.points)
        while tmp == self.point_now:
            tmp = random.choice(self.points)
        self.point_now = tmp

    def update(self):
        # 初始条件
        if not self.condition():
            return Status.FAILURE
        
        # 初始化决策
        if self.blackboard.dec_now != self.name:
            self.init_dec()
            self.blackboard.goal = self.point_now
            self.node.get_logger().info("%s: send goal x:%f y:%f"%(self.name,self.point_now['x'],self.point_now['y']))
            return Status.SUCCESS
        
        if self.blackboard.running.data == True:
            self.node.get_logger().info("running")
            return Status.SUCCESS
        
        # 分成4种情况
        # 1. 到达点位，reach_goal为true，则开启wait_begin;
        # 2. 未到达，继续发点
        # 3. wait_begin已经开启，时间未达到，直接继续发送当前点位
        # 4. wait_begin已经开启，时间达到，进入go_to_next尝试发送下一点位
        if self.wait_begin:
            if time.time() > self.end_time:
                self.wait_begin = False
                self.go_to_next()
                self.blackboard.goal = self.point_now
                self.node.get_logger().info("%s: send goal x:%f y:%f"%(self.name,self.point_now['x'],self.point_now['y']))
                return Status.SUCCESS
            else:
                self.node.get_logger().info("waitting")
                return Status.SUCCESS
        
        if self.blackboard.reach_goal:
            self.wait_begin = True
            self.time_ = time.time()
            tmp = 0
            if self.blackboard.Referee.bullet_remaining_num_17mm > 0:
                tmp = random.uniform(3,4)
                self.node.get_logger().info("have bullet!!!")
            self.end_time = tmp + time.time()
            self.node.get_logger().info("time: %f"%(tmp))
        else:
            self.blackboard.goal = self.point_now
            self.node.get_logger().info("%s: send goal x:%f y:%f"%(self.name,self.point_now['x'],self.point_now['y']))
        
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
        self.blackboard.register_key("priority",py_trees.common.Access.WRITE)

        self.nav = nav
        self.blackboard.running = Bool()
        self.blackboard.running.data = False
        self.node = node
        self.blackboard.reach_goal = False
        self.blackboard.priority = Bool()
        self.blackboard.priority.data = False

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
    
class PriorityDec(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node:Node):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("priority",py_trees.common.Access.WRITE)
        self.blackboard.register_key("running",py_trees.common.Access.READ)
        self.blackboard.register_key("auto_aim",py_trees.common.Access.READ)
        self.blackboard.register_key("dec_now",py_trees.common.Access.READ)

        self.node = node

    def update(self):
        if self.blackboard.running.data:
            self.blackboard.priority.data = True
            self.node.get_logger().info("priority: nav")
        else:
            if self.blackboard.dec_now == "outpost":
                if self.blackboard.auto_aim.id != 'outpost':
                    self.blackboard.priority.data = True
                    self.node.get_logger().info("priority: nav")
                else:
                    self.blackboard.priority.data = False
                    self.node.get_logger().info("priority: auto_aim id: %s"%self.blackboard.auto_aim.id)
            else:
                self.blackboard.priority.data = False
                self.node.get_logger().info("priority: auto_aim id: %s"%self.blackboard.auto_aim.id)

        return Status.SUCCESS
    
class RotDec(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.yaml.register_key("our_color",py_trees.common.Access.READ)
        self.blackboard.register_key("Referee",py_trees.common.Access.READ)
        self.blackboard.register_key("rot",py_trees.common.Access.WRITE)

        self.blackboard.rot = Bool()
        self.blackboard.rot.data = False

    def update(self):
        if getattr(self.blackboard.Referee,self.yaml.our_color + '_outpost_hp') <= 0.1:
            self.blackboard.rot.data = True
        else:
            self.blackboard.rot.data = False

        return Status.SUCCESS
    
class PitchDec(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("dec_now",py_trees.common.Access.READ)
        self.blackboard.register_key("pitch",py_trees.common.Access.WRITE)

        self.blackboard.pitch = Bool()
        self.blackboard.pitch.data = False

    def update(self):
        if self.blackboard.dec_now == 'outpost':
            self.blackboard.pitch.data = False
        else:
            self.blackboard.pitch.data = True

        return Status.SUCCESS