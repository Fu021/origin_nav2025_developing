import struct
import serial
import time
import rclpy
from rclpy.node import Node, Publisher
from rm_interfaces.msg import Target,GimbalCmd,Gimbal
import rclpy.publisher
from std_msgs.msg import String,Bool,Float32,Int32
from threading import Thread
import tf_transformations
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import pi
from geometry_msgs.msg import Twist
from referee_msg.msg import Referee
import numpy
import glob
from .c_board_data_unpack import parse_message,build_all_message

gimbal_msg = Gimbal()
ser=0
# 定义结构体
class All_Data_Rx:
    def __init__(self, yaw_aim, pitch_aim,fire_or_not,track,vx, vy, rotate, yaw_speed_nav, pitch_mode_nav,super_cap_mode):
        self.yaw_aim = yaw_aim
        self.pitch_aim= pitch_aim
        self.fire_or_not= fire_or_not
        self.track= track
        self.vx = vx
        self.vy = vy
        self.rotate =rotate
        self.yaw_speed_nav = yaw_speed_nav
        self.pitch_mode_nav = pitch_mode_nav
        self.super_cap_mode = super_cap_mode

# 连接串口
def find_usb_devices():
    global ser
    usb_devices = glob.glob('/dev/ttyACM*')
    for device in usb_devices:
        ser = serial.Serial(device, 921600)
        if ser.is_open:
            #print("Trying to connect to:", device)
            time.sleep(0.1)
            while ser.in_waiting>0:
                data = ser.read()
                if data:
                    print('device found:%s',ser)
                    return 1
            ser.close()
            continue
    print("No device found")
    return 0
             
def send_all(yaw_aim,pitch_aim,fire_or_not,track,x_speed, y_speed, rotate,yaw_speed_nav,pitch_mode,super_cap_mode):
    data= All_Data_Rx(yaw_aim, pitch_aim,fire_or_not,track,x_speed, y_speed, rotate,1.0, pitch_mode, super_cap_mode)
    message= build_all_message(data)
    ser.write(message)


class SPNode(Node):
    def __init__(self):
        super().__init__("subscriber_publisher_node")
        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Target, '/armor_solver/target', self.target_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)  # CHANGE
        self.subscription = self.create_subscription(GimbalCmd, '/armor_solver/cmd_gimbal', self.all_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)  # CHANGE
        self.publish_gimbal = self.create_publisher(Gimbal,"gimbal_status",10)
        self.publish_referee:Publisher = self.create_publisher(Referee,"Referee",10)
        self.publisher_timer = self.create_timer(0.0067,self.publish_message)
        #导航
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel_chassis', self.nav_callback,rclpy.qos.qos_profile_sensor_data)
        self.sub_yaw = self.create_subscription(Float32,'nav_yaw',self.yaw_callback,10)
        self.sub_dip_angle = self.create_subscription(Float32,"/dip_angle",self.dip_angle_callback,10)

        self.sub_pitch = self.create_subscription(Bool,'nav_pitch',self.pitch_callback,10)

        self.clock_timer = self.create_timer(6,self.clock_callback)

        self.rotate = 22000 #小陀螺逻辑由电控控制，导航只需要传一个定值
        self.pitch = 0
        self.vx = 0.0
        self.vy = 0.0
        self.v_yaw = 0.0

        self.yaw_aim = 0.0
        self.msg_fire = 0
        self.track = 0.0
        self.pitch_aim = 0.0
        self.super_cap_mode = 1

        self.clock = 1
        
    def pitch_callback(self,msg:Bool):
        if msg.data == True:
            self.pitch = 1
        else:
            self.pitch = 0

    def dip_angle_callback(self, msg:Float32):
        degree = msg.data
        if degree < 8.0:
            self.super_cap_mode = 0
        else:
            self.super_cap_mode = 2
    def clock_callback(self):
        if self.clock == 1:
            self.clock = -1
        else:
            self.clock = 1

    def yaw_callback(self,msg:Float32):
        if self.vx == 0.0 and self.vy == 0.0 and msg.data == 1.5: #msg.data是跑动时候的转速
            self.v_yaw = self.clock*msg.data*2.0
        else:
            self.v_yaw = self.clock*msg.data
        
    def target_callback(self,msg:Target):
        self.track = msg.tracking

    def nav_callback(self, msg:Twist):
         self.vx=msg.linear.x
         self.vy=msg.linear.y

    def all_callback(self,msg):
        self.msg_fire=msg.fire_advice
        self.yaw_aim=msg.yaw
        self.pitch_aim=msg.pitch

    def publish_message(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'gimbal_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler((pi/180)*gimbal_msg.roll, (pi/180)*gimbal_msg.pitch, (pi/180)*gimbal_msg.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)
        # print(t.transform.rotation)
        self.publish_gimbal.publish(gimbal_msg)
        # print(self.yaw_aim,self.pitch_aim)

        send_all(self.yaw_aim,self.pitch_aim,self.msg_fire,self.track,self.vx,self.vy,self.rotate,self.v_yaw,self.pitch,self.super_cap_mode)

def receive_message(node:SPNode):
    global parsed_data
    while 1:
        head = ser.read()
        #print(head.hex())
        if head.hex() == 'aa':
            rx_buffer = []
            #print('receive_message')
            rx_buffer.append(head.hex())
            length = ser.read()
            rx_buffer.append(length.hex())
            while len(rx_buffer) < int.from_bytes(length, byteorder='big'):  # 至少包含帧头、帧长度和CRC校验位的长度
                rx_buffer.append(ser.read().hex())
            # print(rx_buffer)
            try:
                referee_data, result = parse_message(rx_buffer)
                if result != None:
                    gimbal_msg.yaw = result[0]
                    gimbal_msg.roll  = result[1]
                    gimbal_msg.pitch= result[2]
                    gimbal_msg.mode= 1 #0 打红 1打蓝
                if referee_data != None:
                    node.publish_referee.publish(referee_data)
                    # 发布消息
                #print(gimbal_msg.yaw,gimbal_msg.roll,gimbal_msg.pitch)
                
            except Exception as e:
                print("Error:",str(e))
                
def main(args=None):
    rclpy.init(args=args)
    find_usb_devices()
    subscriber_publisher_node = SPNode()
    p_message = Thread(target=receive_message,args=(subscriber_publisher_node,))
    p_message.start()
    rclpy.spin(subscriber_publisher_node)
    subscriber_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()