import struct
from referee_msg.msg import Referee

# CRC-8 校验表
CRC08_Table = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
]

# 定义帧头和命令字
HEADER = 0xAA
CMD_ID_AUTOAIM_DATA_RX= 0x81
rOTATE = 1
# 打包结构体为字节流
def pack_all(data):
    return struct.pack('<ff??fffffB?', data.yaw_aim, data.pitch_aim,data.fire_or_not,data.track,data.vx, data.vy, data.rotate, data.yaw_speed_nav, data.pitch_mode_nav,data.super_cap_mode,data.back)

# 计算CRC校验位
def crc8(data):
    crc = 0xff
    for byte in data:
        crc = CRC08_Table[crc ^ byte]
    return crc

# 构建消息
def build_all_message(data):
    data_bytes = pack_all(data)
    length = len(data_bytes) + 4  # 包含帧头、帧长度、命令字和校验位
    #print(length)
    crc = crc8(struct.pack('<BBB', HEADER, length, CMD_ID_AUTOAIM_DATA_RX) + data_bytes)
    return struct.pack('<BBB', HEADER, length, CMD_ID_AUTOAIM_DATA_RX) + data_bytes + struct.pack('<B', crc)

def parse_message(message):
    # 检查帧头4
    if message[0] != 'aa':
        raise ValueError("Invalid header")
    # 解析帧长度
    length = message[1]
    if len(message) != int(length,16):
        raise ValueError("Invalid message length")
    # 检查CRC校验位
    # crc = crc8(message[3:-1])
    # if crc != message[-1]:
    #     raise ValueError("CRC check failed")
    # 解析命令字
    cmd_id = message[2]
    referee_result = []
    referee_data = None
    if cmd_id == '18':
        referee_result.append(
            struct.unpack('<H',bytes.fromhex(''.join(message[3:5])))[0])#剩余血量 uint16
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[5:7]))        #总血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<B',
                bytes.fromhex(''.join(message[7]))          #比赛阶段  uint8
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[8:10]))          #比赛阶段剩余时间  uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[10:12]))          #剩余经济 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[12:14]))          #剩余子弹 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[14:16]))          #红1血量 uint16
                                )[0]
            )
        
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[16:18]))          #红2血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[18:20]))          #红3血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[20:22]))          #红4血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[22:24]))          #红7血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[24:26]))          #红前哨血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[26:28]))          #红基地血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[28:30]))          #蓝1血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[30:32]))          #蓝2血量 uint16
                                )[0]
            )
        
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[32:34]))          #蓝3血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[34:36]))          #蓝4血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[36:38]))          #蓝7血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[38:40]))          #蓝前哨血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<H',
                bytes.fromhex(''.join(message[40:42]))          #蓝基地血量 uint16
                                )[0]
            )
        referee_result.append(
            struct.unpack('<I',
                bytes.fromhex(''.join(message[42:46]))          # rfid uint32
                                )[0]
            )
        referee_result.append(
            struct.unpack('<I',
                bytes.fromhex(''.join(message[46:50]))          #增益点占领状态 uint32
                                )[0]
            )
        referee_result.append(
            struct.unpack('<B',
                bytes.fromhex(''.join(message[50]))          #受伤原因 uint8
                                )[0]
            )
        
        referee_result.append(
            struct.unpack('<B',
                bytes.fromhex(''.join(message[51]))          #敌方英雄位置 uint8
                                )[0]
            )
        referee_data = Referee()
        referee_data.remain_hp = referee_result[0]
        referee_data.max_hp = referee_result[1]
        referee_data.game_progress = referee_result[2]
        referee_data.stage_remain_time = referee_result[3]
        referee_data.coin_remaining_num = referee_result[4]
        referee_data.bullet_remaining_num_17mm = referee_result[5]
        referee_data.red_1_hp = referee_result[6]
        referee_data.red_2_hp = referee_result[7]
        referee_data.red_3_hp = referee_result[8]
        referee_data.red_4_hp = referee_result[9]
        referee_data.red_7_hp = referee_result[10]
        referee_data.red_outpost_hp = referee_result[11]
        referee_data.red_base_hp = referee_result[12]
        referee_data.blue_1_hp = referee_result[13]
        referee_data.blue_2_hp = referee_result[14]
        referee_data.blue_3_hp = referee_result[15]
        referee_data.blue_4_hp = referee_result[16]
        referee_data.blue_7_hp = referee_result[17]
        referee_data.blue_outpost_hp = referee_result[18]
        referee_data.blue_base_hp = referee_result[19]
        referee_data.rfid_status = referee_result[20]
        referee_data.event_type = referee_result[21]
        referee_data.hurt_type = referee_result[22]
        referee_data.enemy_hero_pos = referee_result[23]


        #print(referee_data)
    result = None
    if cmd_id == '14':
        result = []
        #print("Invalid command ID")
        for i in range(3, len(message)-2, 4):
            string = ''.join(message[i:i + 4])  # 每四个十六进制数组合成一个字符串
            bytes_data = bytes.fromhex(string)
            float_value = struct.unpack('<f', bytes_data)[0]
            result.append(float_value)
        result.append(struct.unpack('<B', bytes.fromhex(message[-2]))[0])
    
        #print(result)
    return referee_data, result