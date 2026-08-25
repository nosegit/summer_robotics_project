#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arduino_hardware_interface/arduino_node.hpp"

namespace arduino_hardware_interface
{

class ArduinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<ArduinoNode> arduino_node_;
  std::string port_;
  int baudrate_ {115200};

  double w1_multiplier_ {1.0};
  double w2_multiplier_ {1.0};
  double initial_servo_angle_ {90.0};

  // Command buffers (ROS 2 Control -> Hardware)
  double hw_cmd_w1_ {0.0};
  double hw_cmd_w2_ {0.0};
  double hw_cmd_servo_ {90.0};

  // State buffers (Hardware -> ROS 2 Control)
  double hw_state_w1_vel_ {0.0};
  double hw_state_w1_pos_ {0.0};
  double hw_state_w2_vel_ {0.0};
  double hw_state_w2_pos_ {0.0};

  double hw_state_servo_pos_ {90.0};

  double hw_state_ir1_ {0.0};
  double hw_state_ir2_ {0.0};

  // IMU Data Buffers
  double hw_state_qx_ {0.0}; double hw_state_qy_ {0.0};
  double hw_state_qz_ {0.0}; double hw_state_qw_ {1.0};
  double hw_state_gx_ {0.0}; double hw_state_gy_ {0.0}; double hw_state_gz_ {0.0};
  double hw_state_ax_ {0.0}; double hw_state_ay_ {0.0}; double hw_state_az_ {0.0};
};

}  // namespace arduino_hardware_interface
