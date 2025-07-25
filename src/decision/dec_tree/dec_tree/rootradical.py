import rclpy
from rclpy.node import Node
import py_trees_ros
import py_trees
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.qos import QoSProfile
from .tree_node import GetDataFromYaml,PubGoal,CheckNavState,Patrol,YawDec,PitchDec,UnpackReferee,OutpostAttackDec
from referee_msg.msg import Referee
from std_msgs.msg import Bool, Int32, Float32
from geometry_msgs.msg import Twist

def create_get_data(node,qos_profile,nav):
    get_data = py_trees.composites.Parallel(
        name="get_data",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )

    get_data_from_yaml = GetDataFromYaml(
        name="get_data_from_yaml",
        yaml_name="rmuc_radical",
        node=node
    )

    check_nav_state = CheckNavState(
        name="check_nav_state",
        nav=nav,
        node=node
    )

    referee_list = py_trees.composites.Sequence(
        name='referee_list',
        memory=False
    )
    save_Referee = py_trees_ros.subscribers.ToBlackboard(
        name="save_Referee",
        topic_name="/Referee",
        topic_type=Referee,
        blackboard_variables="Referee",
        initialise_variables=Referee(),
        qos_profile=qos_profile
    )
    save_Referee.setup(node=node)

    unpack_referee = UnpackReferee(name='unpack_referee')

    referee_list.add_children(
        [save_Referee,unpack_referee]
    )

    # save_auto_aim = py_trees_ros.subscribers.ToBlackboard(
    #     name="save_auto_aim",
    #     topic_name="/armor_solver/target",
    #     topic_type=Target,
    #     blackboard_variables="auto_aim",
    #     initialise_variables=Target(),
    #     qos_profile=rclpy.qos.qos_profile_sensor_data
    # )
    # save_auto_aim.setup(node=node)

    get_data.add_children(
        [get_data_from_yaml,referee_list,check_nav_state]
    )

    return get_data

def create_dec(node,nav,qos_profile):
    dec = py_trees.composites.Sequence(
        name="dec",
        memory=False
    )

    dec_selector = py_trees.composites.Selector(
        name="dec_selector",
        memory=False,
    )

    pub_goal = PubGoal(
        name="pub_goal",
        nav=nav
    )

    def condition_outpost(patrol):
        is_hp_full = (patrol.blackboard.Referee.remain_hp >= 400)
        is_hp_low = (patrol.blackboard.Referee.remain_hp < patrol.yaml.blood_limit)
        is_bullet_low = (patrol.blackboard.Referee.bullet_remaining_num_17mm < 75)
        is_bullet_empty = (patrol.blackboard.Referee.bullet_remaining_num_17mm <= 0)
        is_final_minute = (patrol.blackboard.Referee.stage_remain_time <=62)
        patrol.got_bullet = ((patrol.blackboard.Referee.bullet_remaining_num_17mm - patrol.bullet_remain_last > 50) and patrol.blackboard.home_occupy != 0) #在家里这一刻拿到弹了
        print(f"got_bullet_in_final_minute:{patrol.got_bullet_in_final_minute},{patrol.got_bullet}")
        their_color = 'red'
        if patrol.yaml.our_color == 'red':
            their_color = 'blue'
        if getattr(patrol.blackboard.Referee,their_color + '_outpost_hp') >= 0 and patrol.blackboard.Referee.stage_remain_time<=360:
            return True
        return False
        

    goto_outpost = Patrol(
        name="goto_outpost",
        points_name="outpost",
        node=node,
        nav=nav,
        condition_func=condition_outpost
    )

    def condition_peek(patrol):
        is_hp_full = (patrol.blackboard.Referee.remain_hp >= 400)
        is_hp_low = (patrol.blackboard.Referee.remain_hp < patrol.yaml.blood_limit)
        is_bullet_low = (patrol.blackboard.Referee.bullet_remaining_num_17mm < 75)
        is_bullet_empty = (patrol.blackboard.Referee.bullet_remaining_num_17mm <= 0)
        is_final_minute = (patrol.blackboard.Referee.stage_remain_time <=62)
        patrol.got_bullet = ((patrol.blackboard.Referee.bullet_remaining_num_17mm - patrol.bullet_remain_last > 50) and patrol.blackboard.home_occupy != 0) #在家里这一刻拿到弹了
        print(f"got_bullet_in_final_minute:{patrol.got_bullet_in_final_minute},{patrol.got_bullet}")
        their_color = 'red'
        if patrol.yaml.our_color == 'red':
            their_color = 'blue'
        if getattr(patrol.blackboard.Referee,their_color + '_outpost_hp') <= 0:
            return True
        return False

    goto_peek = Patrol(
        name='goto_peek',
        points_name='peek',
        node=node,
        nav=nav,
        condition_func=condition_peek
    )

    def condition_home(patrol):
        is_hp_full = (patrol.blackboard.Referee.remain_hp >= 400)
        is_hp_low = (patrol.blackboard.Referee.remain_hp < patrol.yaml.blood_limit)
        is_bullet_low = (patrol.blackboard.Referee.bullet_remaining_num_17mm < 75)
        is_bullet_empty = (patrol.blackboard.Referee.bullet_remaining_num_17mm <= 0)
        is_final_minute = (patrol.blackboard.Referee.stage_remain_time <=62)
        patrol.got_bullet = ((patrol.blackboard.Referee.bullet_remaining_num_17mm - patrol.bullet_remain_last > 50) and patrol.blackboard.home_occupy != 0) #在家里这一刻拿到弹了
        print(f"got_bullet_in_final_minute:{patrol.got_bullet_in_final_minute},{patrol.got_bullet}")
        '''
        #     需要回家需要满足的条件：
        #     case1： 比赛前六分钟没血或没弹就回家，直到血量满且子弹足

        #     case2： 进入最后一分钟，没有在最后一分钟拿到弹，且血低或弹尽，直到血回满并且拿到弹再走

        #     case3： 最后一分钟并且在最后一分钟拿到过弹后，仅血量不足回家,血回满再走

        #     都需要进行的：判断当前在补给区，并且距离下一波发弹的时间小于10s,则等待12s


        #     '''


        if is_final_minute:
            if patrol.got_bullet:
                patrol.got_bullet_in_final_minute  = True
            
        # Case 1: 进入最后一分钟且未拿到过弹，且血低或弹尽，直到血回满并且拿到弹再走
            if not patrol.got_bullet_in_final_minute :
                # 若 血量低且弹量为空继续发点
                if is_hp_low or is_bullet_empty :
                    return True
                # 若 血量没有回满且弹量low继续发点
                elif ((not is_hp_full) or is_bullet_low) and patrol.blackboard.dec_now == 'goto_home': 
                    return True
        # Case 2: 最后一分钟并且拿到过弹后，仅血量不足回家,血回满再走
            else :
                if is_hp_low:
                    return True
                elif (not is_hp_full) and patrol.blackboard.dec_now == 'goto_home':
                    return True
            
        
        else: #比赛前六分钟

            if is_hp_low or is_bullet_empty: #没血或者没弹
                return True
            elif patrol.waiting_for == "home_phase_12s":
                return True
            elif ((not is_hp_full) or is_bullet_low) and patrol.blackboard.dec_now == 'goto_home': #血量没回满或者子弹不足，继续在家呆着
                return True
        return False

    goto_home = Patrol(
        name="goto_home",
        points_name="home",
        node=node,
        nav=nav,
        condition_func=condition_home
    )

    def condition_mid(patrol):
        is_hp_full = (patrol.blackboard.Referee.remain_hp >= 400)
        is_hp_low = (patrol.blackboard.Referee.remain_hp < patrol.yaml.blood_limit)
        is_bullet_low = (patrol.blackboard.Referee.bullet_remaining_num_17mm < 75)
        is_bullet_empty = (patrol.blackboard.Referee.bullet_remaining_num_17mm <= 0)
        is_final_minute = (patrol.blackboard.Referee.stage_remain_time <=62)
        patrol.got_bullet = ((patrol.blackboard.Referee.bullet_remaining_num_17mm - patrol.bullet_remain_last > 50) and patrol.blackboard.home_occupy != 0) #在家里这一刻拿到弹了
        print(f"got_bullet_in_final_minute:{patrol.got_bullet_in_final_minute},{patrol.got_bullet}")

        # their_color = 'red'
        # if patrol.yaml.our_color == 'red':
        #     their_color = 'blue'
        # if patrol.blackboard.Referee.stage_remain_time>=360:
        return True

    goto_mid = Patrol(
        name="goto_mid",
        points_name="mid",
        node=node,
        nav=nav,
        condition_func=condition_mid
    )

    def condition_front(patrol):
        is_hp_full = (patrol.blackboard.Referee.remain_hp >= 400)
        is_hp_low = (patrol.blackboard.Referee.remain_hp < patrol.yaml.blood_limit)
        is_bullet_low = (patrol.blackboard.Referee.bullet_remaining_num_17mm < 75)
        is_bullet_empty = (patrol.blackboard.Referee.bullet_remaining_num_17mm <= 0)
        is_final_minute = (patrol.blackboard.Referee.stage_remain_time <=62)
        print(f"got_bullet_in_final_minute:{patrol.got_bullet_in_final_minute},{patrol.got_bullet}")

        their_color = 'red'
        if patrol.yaml.our_color == 'red':
            their_color = 'blue'
        if getattr(patrol.blackboard.Referee,their_color + '_outpost_hp') <= 0 and patrol.blackboard.Referee.stage_remain_time>=300:
            return True
        return False


    goto_front = Patrol(
        name="goto_front",
        points_name="front",
        node=node,
        nav=nav,
        condition_func=condition_front
    )


    # yaw = py_trees.composites.Sequence(
    #     name = 'yaw',
    #     memory=False
    # )

    # yaw_dec = YawDec(
    #     name='yaw_dec'
    # )
    # send_yaw = py_trees_ros.publishers.FromBlackboard(
    #     name="send_yaw",
    #     topic_name="nav_yaw",
    #     topic_type=Float32,
    #     qos_profile=qos_profile,
    #     blackboard_variable="yaw"
    # )
    # send_yaw.setup(node=node)

    # yaw.add_children(
    #     [yaw_dec,send_yaw]
    # )


    pitch = py_trees.composites.Sequence(
        name = 'pitch',
        memory=False
    )
    pitch_dec = PitchDec(
        name='pitch_dec'
    )

    send_pitch = py_trees_ros.publishers.FromBlackboard(
        name="send_pitch",
        topic_name="nav_pitch",
        topic_type=Bool,
        qos_profile=qos_profile,
        blackboard_variable="pitch"
    )
    send_pitch.setup(node=node)

    pitch.add_children(
        [pitch_dec,send_pitch]
    )

    outpost_attack_list = py_trees.composites.Sequence(
        name="outpost_attack_list",
        memory=False
    )

    outpost_attack_dec = OutpostAttackDec(
        name="outpost_attack_dec"
    )

    send_outpost_attack = py_trees_ros.publishers.FromBlackboard(
        name="send_outpost_attack",
        topic_name="outpost_attack",
        topic_type=Bool,
        qos_profile=qos_profile,
        blackboard_variable="outpost_attack"
    )
    send_outpost_attack.setup(node=node)

    outpost_attack_list.add_children(
        [outpost_attack_dec,send_outpost_attack]
    )

    dec_selector.add_children(
        [goto_home,goto_front,goto_peek,goto_outpost,goto_mid]
    )

    dec.add_children(
        [dec_selector,pitch,outpost_attack_list,pub_goal]
    )

    return dec

def create_tree(node):
    qos_profile = QoSProfile(depth=10)
    nav = BasicNavigator()

    root = py_trees.composites.Sequence(
        name="root",
        memory=False
    )

    get_data = create_get_data(node,qos_profile,nav)

    dec = create_dec(node,nav,qos_profile)

    root.add_children(
        [get_data,dec]
    )

    return root

def main(args = None):
    rclpy.init(args=args)
    node = Node("tree_node")
    period_ms = 100
    root = create_tree(node)
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node=node)
    tree.tick_tock(period_ms=period_ms)
    rclpy.spin(node)
    rclpy.shutdown()