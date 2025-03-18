import rclpy
from rclpy.node import Node
import py_trees_ros
import py_trees
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.qos import QoSProfile
from .tree_node import GetDataFromYaml,PubGoal,CheckNavState,Patrol,PriorityDec,RotDec,PitchDec
from referee_msg.msg import Referee
from std_msgs.msg import Bool
from rm_interfaces.msg import Target,GimbalCmd,Gimbal

def create_get_data(node,qos_profile,nav):
    get_data = py_trees.composites.Parallel(
        name="get_data",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )

    get_data_from_yaml = GetDataFromYaml(
        name="get_data_from_yaml",
        yaml_name="A",
        node=node
    )

    check_nav_state = CheckNavState(
        name="check_nav_state",
        nav=nav,
        node=node
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

    save_auto_aim = py_trees_ros.subscribers.ToBlackboard(
        name="save_auto_aim",
        topic_name="/armor_solver/target",
        topic_type=Target,
        blackboard_variables="auto_aim",
        initialise_variables=Target(),
        qos_profile=rclpy.qos.qos_profile_sensor_data
    )
    save_auto_aim.setup(node=node)

    get_data.add_children(
        [get_data_from_yaml,save_Referee,check_nav_state,save_auto_aim]
    )

    return get_data

def create_send_data(node,qos_profile):
    send_data = py_trees.composites.Parallel(
        name="send_data",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )

    send_running_state = py_trees_ros.publishers.FromBlackboard(
        name="send_running_state",
        topic_name="running_state",
        topic_type=Bool,
        qos_profile=qos_profile,
        blackboard_variable="running"
    )
    send_running_state.setup(node=node)

    send_rot = py_trees_ros.publishers.FromBlackboard(
        name="send_rot",
        topic_name="nav_rotate",
        topic_type=Bool,
        qos_profile=qos_profile,
        blackboard_variable="rot"
    )
    send_rot.setup(node=node)

    send_pitch = py_trees_ros.publishers.FromBlackboard(
        name="send_pitch",
        topic_name="nav_pitch",
        topic_type=Bool,
        qos_profile=qos_profile,
        blackboard_variable="pitch"
    )

    send_data.add_children(
        [send_running_state,
        send_rot,send_pitch]
    )

    return send_data

def create_dec(node,nav):
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

    home = Patrol(
        name="home",
        points_name="home",
        node=node,
        controller="FollowPath",
        referee_condition=1,
        nav=nav
    )

    outpost_1 = Patrol(
        name="outpost_1",
        points_name="outpost_1",
        node=node,
        controller="FollowPath",
        nav=nav,
        referee_condition=2
    )

    outpost = Patrol(
        name="outpost",
        points_name="outpost",
        node=node,
        controller="FollowPath",
        nav=nav
    )

    test = Patrol(
        name="outpost",
        points_name="test",
        node=node,
        nav=nav,
        controller="FollowPath"
    )

    priority_dec = PriorityDec(
        name="priority_dec",
        node=node
    )

    rot_dec = RotDec(
        name="rot_dec"
    )

    pitch_dec = PitchDec(
        name="pitch_dec"
    )

    dec_selector.add_children(
        [home,outpost_1,outpost]
        #[test]
    )

    dec.add_children(
        [rot_dec,pitch_dec,dec_selector,pub_goal]
    )

    return dec

def create_tree(node,period_ms):
    qos_profile = QoSProfile(depth=10)
    nav = BasicNavigator()

    root = py_trees.composites.Sequence(
        name="root",
        memory=False
    )

    get_data = create_get_data(node,qos_profile,nav)

    send_data = create_send_data(node,qos_profile)

    dec = create_dec(node,nav)

    root.add_children(
        [get_data,send_data,dec]
    )

    return root

def main(args = None):
    rclpy.init(args=args)
    node = Node("tree_node")
    period_ms = 100
    root = create_tree(node,period_ms)
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node=node)
    tree.tick_tock(period_ms=period_ms)
    rclpy.spin(node)
    rclpy.shutdown()