#include <rm_serial_cpp/Serial.hpp>

#include <thread>

#include <dirent.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace rm_serial_cpp
{
    Serial::Serial(rm_serial_cpp::RmSerialCpp& node)
    : node_ref(node)
    {
        std::thread connect_thread(std::bind(&Serial::connect, this));
        std::thread receive_thread(std::bind(&Serial::receive, this));

        serial = std::make_shared<boost::asio::serial_port>(io);
    }

    void Serial::connect()
    {
        while (1)
        {
            if (!is_connect.load())
            {
                DIR* dir = opendir("/dev/");
                struct dirent* entry;
                while ((entry = readdir(dir)) != nullptr) {
                    if (strncmp(entry->d_name, "ttyACM", 6) == 0) {
                        try
                        {
                            serial->open("/dev/" + std::string(entry->d_name));
                            serial->set_option(boost::asio::serial_port_base::baud_rate(921600));
                            serial->set_option(boost::asio::serial_port_base::character_size(8));
                            serial->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                            serial->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                            serial->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
                            
                            char data;
                            boost::system::error_code ec;
                            size_t bytes_read = boost::asio::read(*serial, boost::asio::buffer(&data, 1), ec);
                            if (ec)
                            {
                                serial->close();
                                continue;
                            }
                            if (bytes_read > 0) {
                                is_connect.store(true);
                                RCLCPP_INFO(node_ref.get_logger(),"connect success");
                                break;
                            }
                        }  
                        catch (const std::exception& e) {serial->close();}
                    }
                }
                closedir(dir);
                if (!is_connect.load())
                    RCLCPP_INFO(node_ref.get_logger(),"no device found");
            }
        }
    }

    void Serial::receive()
    {
        while (1)
        {
            if (is_connect.load())
            {
                try
                {
                    uint8_t head;
                    boost::asio::read(*serial, boost::asio::buffer(&head, 1));
                    if (head == 0xaa) {
                        std::vector<uint8_t> rx_buffer;
                        rx_buffer.push_back(head);

                        uint8_t length_byte;
                        boost::asio::read(*serial, boost::asio::buffer(&length_byte, 1));
                        rx_buffer.push_back(length_byte);

                        size_t expected_length = static_cast<size_t>(length_byte);
                        while (rx_buffer.size() < expected_length) {
                            uint8_t byte;
                            boost::asio::read(*serial, boost::asio::buffer(&byte, 1));
                            rx_buffer.push_back(byte);
                        }

                        int result = parse_message(rx_buffer);
                        switch (result)
                        {
                        case 0:
                            break;
                        case 1:
                            node_ref.referee_pub_publish(referee);
                            break;
                        case 2:
                            node_ref.autoaim_gimbal_pub_publish(gimbal);
                            publish_tf();
                            break;
                        default:
                            break;
                        }
                    }
                }
                catch(const std::exception& e)
                {
                    is_connect.store(false);
                    RCLCPP_INFO(node_ref.get_logger(),"no connect");
                }
            }
        }
    }

    int Serial::parse_message(const std::vector<uint8_t>& rx_buffer)
    {
        size_t expected_length = static_cast<size_t>(rx_buffer[1]);
        if (rx_buffer[0] != 0xaa || rx_buffer.size() != expected_length)
            return 0;

        if (rx_buffer[2] == 0x18)
        {
            std::memcpy(&referee.remain_hp,&rx_buffer[3],sizeof(uint16_t));
            std::memcpy(&referee.max_hp,&rx_buffer[5],sizeof(uint16_t));
            std::memcpy(&referee.game_progress,&rx_buffer[7],sizeof(uint8_t));
            std::memcpy(&referee.stage_remain_time,&rx_buffer[8],sizeof(uint16_t));
            std::memcpy(&referee.coin_remaining_num,&rx_buffer[10],sizeof(uint16_t));
            std::memcpy(&referee.bullet_remaining_num_17mm,&rx_buffer[12],sizeof(uint16_t));
            std::memcpy(&referee.red_1_hp,&rx_buffer[14],sizeof(uint16_t));
            std::memcpy(&referee.red_2_hp,&rx_buffer[16],sizeof(uint16_t));
            std::memcpy(&referee.red_3_hp,&rx_buffer[18],sizeof(uint16_t));
            std::memcpy(&referee.red_4_hp,&rx_buffer[20],sizeof(uint16_t));
            std::memcpy(&referee.red_7_hp,&rx_buffer[22],sizeof(uint16_t));
            std::memcpy(&referee.red_outpost_hp,&rx_buffer[24],sizeof(uint16_t));
            std::memcpy(&referee.red_base_hp,&rx_buffer[26],sizeof(uint16_t));
            std::memcpy(&referee.blue_1_hp,&rx_buffer[28],sizeof(uint16_t));
            std::memcpy(&referee.blue_2_hp,&rx_buffer[30],sizeof(uint16_t));
            std::memcpy(&referee.blue_3_hp,&rx_buffer[32],sizeof(uint16_t));
            std::memcpy(&referee.blue_4_hp,&rx_buffer[34],sizeof(uint16_t));
            std::memcpy(&referee.blue_7_hp,&rx_buffer[36],sizeof(uint16_t));
            std::memcpy(&referee.blue_outpost_hp,&rx_buffer[38],sizeof(uint16_t));
            std::memcpy(&referee.blue_base_hp,&rx_buffer[40],sizeof(uint16_t));
            std::memcpy(&referee.rfid_status,&rx_buffer[42],sizeof(uint32_t));
            std::memcpy(&referee.event_type,&rx_buffer[46],sizeof(uint32_t));
            std::memcpy(&referee.hurt_type,&rx_buffer[50],sizeof(uint8_t));
            return 1;
        }
        else if (rx_buffer[2] == 0x14)
        {
            std::memcpy(&gimbal.yaw,&rx_buffer[3],sizeof(float));
            std::memcpy(&gimbal.roll,&rx_buffer[7],sizeof(float));
            std::memcpy(&gimbal.pitch,&rx_buffer[11],sizeof(float));
            return 2;
        }
        else
            return 0;
    }

    void Serial::publish_tf()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = rclcpp::Clock().now();
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "gimbal_link";

        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setEuler((M_PI/180.0)*gimbal.yaw, (M_PI/180.0)*gimbal.pitch, (M_PI/180.0)*gimbal.roll);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        node_ref.tf_broadcaster->sendTransform(transform_stamped);
    }

    void Serial::send_all(const rm_serial_cpp::AllData& all_data)
    {
        if (is_connect.load())
        {
            std::vector<uint8_t> buffer;
            auto append = [&buffer](const void* ptr, size_t size) {
                const uint8_t* raw = reinterpret_cast<const uint8_t*>(ptr);
                buffer.insert(buffer.end(), raw, raw + size);
            };
            append(&all_data.autoaim_yaw, sizeof(all_data.autoaim_yaw));
            append(&all_data.autoaim_pitch, sizeof(all_data.autoaim_pitch));
            append(&all_data.autoaim_fire_advice, sizeof(all_data.autoaim_fire_advice));
            append(&all_data.autoaim_tracking, sizeof(all_data.autoaim_tracking));
            append(&all_data.nav_vx, sizeof(all_data.nav_vx));
            append(&all_data.nav_vy, sizeof(all_data.nav_vy));
            append(&all_data.nav_rot, sizeof(all_data.nav_rot));
            append(&all_data.nav_yaw, sizeof(all_data.nav_yaw));
            append(&all_data.nav_pitch, sizeof(all_data.nav_pitch));

            uint8_t length = buffer.size() + 4;
            uint8_t crc = crc8(buffer,sizeof(buffer));

            std::vector<uint8_t> message;
            message.push_back(0xaa);
            message.push_back(length);
            message.push_back(0x81);
            message.insert(message.end(), buffer.begin(), buffer.end());
            message.push_back(crc);

            try
            {
                serial->write_some(boost::asio::buffer(message));
            }
            catch(const std::exception& e)
            {
                is_connect.store(false);
                RCLCPP_INFO(node_ref.get_logger(),"no connect");
            }
        }
    }

    uint8_t Serial::crc8(const std::vector<uint8_t> data, size_t length)
    {
        uint8_t crc = 0xff;
        for (size_t i = 0; i < length; i++) 
        {
            crc = CRC08_Table[crc ^ data[i]];
        }
        return crc;
    }
}