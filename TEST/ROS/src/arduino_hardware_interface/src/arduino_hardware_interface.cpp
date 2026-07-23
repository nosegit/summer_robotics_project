#include "arduino_hardware_interface/arduino_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arduino_hardware_interface
{

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse USB parameters from URDF
  if (info_.hardware_parameters.find("serial_port") != info_.hardware_parameters.end()) {
    port_ = info_.hardware_parameters.at("serial_port");
  } else {
    port_ = "/dev/ttyACM0";
  }

  if (info_.hardware_parameters.find("baudrate") != info_.hardware_parameters.end()) {
    baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joint 0: Wheel 1
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_state_w1_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_state_w1_vel_));
  
  // Joint 1: Wheel 2
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &hw_state_w2_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_state_w2_vel_));

  // Joint 2: Servo
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[2].name, hardware_interface::HW_IF_POSITION, &hw_state_servo_pos_));

  // IMU State Interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.x", &hw_state_qx_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.y", &hw_state_qy_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.z", &hw_state_qz_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.w", &hw_state_qw_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.x", &hw_state_gx_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.y", &hw_state_gy_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.z", &hw_state_gz_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.x", &hw_state_ax_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.y", &hw_state_ay_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.z", &hw_state_az_));

  // IR Distance Sensors
  state_interfaces.emplace_back(hardware_interface::StateInterface("ir_sensor_1", "range", &hw_state_ir1_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("ir_sensor_2", "range", &hw_state_ir2_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_w1_));
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_w2_));
    
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[2].name, hardware_interface::HW_IF_POSITION, &hw_cmd_servo_));

  return command_interfaces;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Opening serial connection on %s...", port_.c_str());
  
  arduino_node_ = std::make_unique<ArduinoNode>(port_, baudrate_, info_.name);
  arduino_node_->start();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Closing serial connection...");
  if (arduino_node_) {
    arduino_node_->stop();
    arduino_node_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!arduino_node_) return hardware_interface::return_type::ERROR;

  // Retrieve data from background serial thread
  ArduinoData data = arduino_node_->getData();

  // Velocities (rad/s)
  hw_state_w1_vel_ = data.w1_actual;
  hw_state_w2_vel_ = data.w2_actual;
  
  // Position integration (rad = rad/s * dt)
  hw_state_w1_pos_ += hw_state_w1_vel_ * period.seconds();
  hw_state_w2_pos_ += hw_state_w2_vel_ * period.seconds();

  hw_state_servo_pos_ = data.servo_angle;
  hw_state_ir1_ = static_cast<double>(data.ir_distance_1);
  hw_state_ir2_ = static_cast<double>(data.ir_distance_2);

  // IMU Data
  hw_state_qx_ = data.qx; hw_state_qy_ = data.qy; 
  hw_state_qz_ = data.qz; hw_state_qw_ = data.qw;
  hw_state_gx_ = data.gx; hw_state_gy_ = data.gy; hw_state_gz_ = data.gz;
  hw_state_ax_ = data.ax; hw_state_ay_ = data.ay; hw_state_az_ = data.az;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!arduino_node_) return hardware_interface::return_type::ERROR;

  // Pass latest commands down to serial transmission buffer
  arduino_node_->sendCommand(
    static_cast<float>(hw_cmd_w1_), 
    static_cast<float>(hw_cmd_w2_), 
    static_cast<float>(hw_cmd_servo_)
  );

  return hardware_interface::return_type::OK;
}

}  // namespace arduino_hardware_interface

// Export C++ plugin
PLUGINLIB_EXPORT_CLASS(
  arduino_hardware_interface::ArduinoHardwareInterface, hardware_interface::SystemInterface)