import rclpy
from rclpy.node import Node
import py_trees_ros
import py_trees
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.qos import QoSProfile
from .tree_node import GetDataFromYaml,PubGoal,CheckNavState,Patrol,RotDec,YawDec,UnpackReferee
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
        yaml_name="new_test",
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
        topic_name="/referee",
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

    goto_mid = Patrol(
        name="goto_mid",
        points_name="mid",
        node=node,
        nav=nav,
        referee_condition='mid'
    )

    goto_peek = Patrol(
        name='goto_peek',
        points_name='peek',
        node=node,
        nav=nav,
        referee_condition='peek'
    )

    goto_home = Patrol(
        name="goto_home",
        points_name="home",
        node=node,
        nav=nav,
        referee_condition='home'
    )

    rot = py_trees.composites.Sequence(
        name = 'rot',
        memory=False
    )


    rot_dec = RotDec(
        name="rot_dec"
    )
    send_rot = py_trees_ros.publishers.FromBlackboard(
        name="send_rot",
        topic_name="nav_rotate",
        topic_type=Int32,
        qos_profile=qos_profile,
        blackboard_variable="rot"
    )
    send_rot.setup(node=node)

    rot.add_children(
        [rot_dec,send_rot]
    )

    yaw = py_trees.composites.Sequence(
        name = 'yaw',
        memory=False
    )

    yaw_dec = YawDec(
        name='yaw_dec'
    )
    send_yaw = py_trees_ros.publishers.FromBlackboard(
        name="send_yaw",
        topic_name="nav_yaw",
        topic_type=Float32,
        qos_profile=qos_profile,
        blackboard_variable="yaw"
    )
    send_yaw.setup(node=node)

    yaw.add_children(
        [yaw_dec,send_yaw]
    )

    dec_selector.add_children(
        [goto_home,goto_peek,goto_mid]
    )

    dec.add_children(
        [dec_selector,rot,yaw,pub_goal]
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