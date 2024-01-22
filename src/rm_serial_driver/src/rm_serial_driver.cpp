// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  serial_pub_ = this->create_publisher<auto_aim_interfaces::msg::ReceiveSerial>("/angle/init", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  result_sub_ = this->create_subscription<auto_aim_interfaces::msg::SendSerial>(
    "/trajectory/result", 10,
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}
  // while (rclcpp::ok()) {
  //   try {
  //     // 这一行从串行端口接收一个字节的数据，将其存储在 header 向量中
  //     serial_driver_->port()->receive(header);
  //     if (header[0] == 0x5A) {
  //       // std::cout << "ture" << std::endl;
  //       data.resize(sizeof(ReceivePacket) - 1);
  //       serial_driver_->port()->receive(data);

  //       data.insert(data.begin(), header[0]);
  //       ReceivePacket packet = fromVector(data);
  //       // 验证数据的CRC校验和
  //       bool crc_ok =
  //         crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  //       if (crc_ok) {
  //         if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
  //           setParam(rclcpp::Parameter("detect_color", packet.detect_color));
  //           previous_receive_color_ = packet.detect_color;
  //         }

  //         if (packet.reset_tracker) {
  //           resetTracker();
  //         }
  //         // 创建一个名为 t 的 TransformStamped 消息，设置时间戳和坐标变换信息，然后发布坐标变换消息
  //         geometry_msgs::msg::TransformStamped t;
  //         timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
  //         t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
  //         t.header.frame_id = "odom";
  //         t.child_frame_id = "gimbal_link";
  //         tf2::Quaternion q;
  //         q.setRPY(packet.roll, packet.pitch, packet.yaw);
  //         t.transform.rotation = tf2::toMsg(q);
  //         tf_broadcaster_->sendTransform(t);

  //         if (abs(packet.aim_x) > 0.01) {
  //           aiming_point_.header.stamp = this->now();
  //           aiming_point_.pose.position.x = packet.aim_x;
  //           aiming_point_.pose.position.y = packet.aim_y;
  //           aiming_point_.pose.position.z = packet.aim_z;
  //           marker_pub_->publish(aiming_point_);
  //         }
  //       } else {
  //         RCLCPP_ERROR(get_logger(), "CRC error!");
  //       }
  //     } else {
  //       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
  //     }
  //   } catch (const std::exception & ex) {
  //     RCLCPP_ERROR_THROTTLE(
  //       get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
  //     reopenPort();
  //   }
  // }

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));
  bool receiving_data = false;  // 用于跟踪是否正在接收数据
  std::vector<uint8_t> data_buffer;  // 用于存储接收的数据

  while (rclcpp::ok()) {
      try {
          // 这一行从串行端口接收一个字节的数据，将其存储在 header 向量中
          serial_driver_->port()->receive(header);
     
          if (receiving_data) {
              // 如果正在接收数据，将数据添加到缓冲区
              data_buffer.push_back(header[0]);
              // std::cout << "header[0]" << static_cast<int>(header[0]) << std::endl;
              if (header[0] == 0xAA) {
                  // 如果检测到结束标识符（0xAAA），则停止接收数据并处理
                  receiving_data = false;
                  // for(int i = 0; i < static_cast<int>(data_buffer.size()); i++)
                  // {
                  //     //int a = int(data_buffer[i])；
                  //     std::cout << "data_buffer[" << i << "]:" << data_buffer[i] << std::endl;
                  //     //std::cout << "data_buffer[" << i << "]:" << std::hex <<std::uppercase <<a <<std::endl;
                  // }
                  // 处理接收到的数据
                  //if (data_buffer.size() == sizeof(ReceivePacket) - 1) {
                      ReceivePacket packet = fromVector(data_buffer);

                      // 执行您的操作，例如设置参数、发布消息等
                      if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
                          setParam(rclcpp::Parameter("detect_color", packet.detect_color));
                          previous_receive_color_ = packet.detect_color;
                      }

                      if (packet.reset_tracker) {
                          resetTracker();
                      }

                      // 创建坐标变换消息和发布
                      geometry_msgs::msg::TransformStamped t;
                      timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
                      t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
                      t.header.frame_id = "odom";
                      t.child_frame_id = "gimbal_link";
                      tf2::Quaternion q;
                      q.setRPY(packet.roll / 57.3f, packet.pitch / 57.3f, packet.yaw / 57.3f);
                      t.transform.rotation = tf2::toMsg(q);
                      tf_broadcaster_->sendTransform(t);

                      receive_serial_msg_.header.frame_id = "odom";
                      receive_serial_msg_.header.stamp = this->now();
                      receive_serial_msg_.is_shoot = packet.is_shoot;
                      receive_serial_msg_.pitch = packet.pitch;
                      receive_serial_msg_.yaw = packet.yaw;
                      serial_pub_->publish(receive_serial_msg_);
                  //}

                  // 清空数据缓冲区
                  data_buffer.clear();
              }
          } else if (header[0] == 0x5A) {
              // 如果检测到开始标识符（0x5A），开始接收数据
              receiving_data = true;
              data_buffer.push_back(header[0]);
          }
      } catch (const std::exception & ex) {
          RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
          reopenPort();
      }
  }

}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::SendSerial msg)
{
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    SendPacket packet;
    packet.header = 0xA5;
    packet.is_tracking = msg.is_tracking;
    packet.yaw = msg.yaw;
    packet.pitch = msg.pitch;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg.header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
