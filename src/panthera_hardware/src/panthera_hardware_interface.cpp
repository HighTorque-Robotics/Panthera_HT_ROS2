#include "panthera_hardware/panthera_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "panthera/Panthera.hpp"

namespace panthera_hardware
{

hardware_interface::CallbackReturn PantheraHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get configuration file path
  if (info_.hardware_parameters.find("config_file") != info_.hardware_parameters.end())
  {
    config_file_ = info_.hardware_parameters["config_file"];
    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Config file: %s", config_file_.c_str());
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Parameter 'config_file' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get control mode (default: position_velocity)
  if (info_.hardware_parameters.find("control_mode") != info_.hardware_parameters.end())
  {
    control_mode_ = info_.hardware_parameters["control_mode"];
  }
  else
  {
    control_mode_ = "position_velocity";
  }
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
              "Control mode: %s", control_mode_.c_str());

  // Get gripper rad to meter conversion factor
  if (info_.hardware_parameters.find("gripper_rad_to_m") != info_.hardware_parameters.end())
  {
    gripper_rad_to_m_ = std::stod(info_.hardware_parameters["gripper_rad_to_m"]);
  }
  else
  {
    gripper_rad_to_m_ = 0.01;  // Default: 1 radian = 0.01 meter
  }
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
              "Gripper rad to meter conversion: %.4f", gripper_rad_to_m_);

  // Check if velocity and effort commands are enabled
  use_velocity_commands_ = (control_mode_ == "full_control" || control_mode_ == "position_velocity");
  use_effort_commands_ = (control_mode_ == "full_control");

  // Initialize state and command storage
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_efforts_.resize(info_.joints.size(), 0.0);

  // Initialize control parameters
  max_torques_.resize(info_.joints.size(), 0.0);
  max_velocities_.resize(info_.joints.size(), 0.5);
  kp_gains_.resize(info_.joints.size(), 0.0);
  kd_gains_.resize(info_.joints.size(), 0.0);

  // Load joint-specific parameters
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Max torque
    if (info_.joints[i].parameters.find("max_torque") != info_.joints[i].parameters.end())
    {
      max_torques_[i] = std::stod(info_.joints[i].parameters.at("max_torque"));
    }
    else
    {
      max_torques_[i] = 10.0;  // Default value
    }

    // Max velocity
    if (info_.joints[i].parameters.find("max_velocity") != info_.joints[i].parameters.end())
    {
      max_velocities_[i] = std::stod(info_.joints[i].parameters.at("max_velocity"));
    }
    else
    {
      max_velocities_[i] = 0.5;  // Default value
    }

    // PD gains
    if (info_.joints[i].parameters.find("kp") != info_.joints[i].parameters.end())
    {
      kp_gains_[i] = std::stod(info_.joints[i].parameters.at("kp"));
    }
    else
    {
      kp_gains_[i] = 4.0;  // Default value
    }

    if (info_.joints[i].parameters.find("kd") != info_.joints[i].parameters.end())
    {
      kd_gains_[i] = std::stod(info_.joints[i].parameters.at("kd"));
    }
    else
    {
      kd_gains_[i] = 0.5;  // Default value
    }

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Joint %s: max_torque=%.2f, max_velocity=%.2f, kp=%.2f, kd=%.2f",
                info_.joints[i].name.c_str(), max_torques_[i], max_velocities_[i],
                kp_gains_[i], kd_gains_[i]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Configuring...");

  // Initialize Panthera robot
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Initializing Panthera robot with config: %s", config_file_.c_str());

    robot_ = std::make_unique<panthera::Panthera>(config_file_);

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Panthera robot initialized successfully");
  }
  catch (const std::bad_alloc & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Memory allocation failed during Panthera initialization: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Failed to initialize Panthera robot: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Unknown exception during Panthera initialization");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial joint states
  try
  {
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read 6 arm joint states
    auto positions = robot_->getCurrentPos();
    auto velocities = robot_->getCurrentVel();
    auto torques = robot_->getCurrentTorque();

    for (size_t i = 0; i < 6; i++)
    {
      hw_positions_[i] = positions[i];
      hw_velocities_[i] = velocities[i];
      hw_efforts_[i] = torques[i];
      hw_commands_positions_[i] = positions[i];  // Initialize commands to current position
    }

    // Read gripper state (7th joint, index 6 = L_finger_joint) if present
    // Convert from radians to meters for prismatic joint
    if (info_.joints.size() > 6)
    {
      double gripper_rad = robot_->getCurrentPosGripper();
      hw_positions_[6] = gripper_rad * gripper_rad_to_m_;
      hw_velocities_[6] = robot_->getCurrentVelGripper() * gripper_rad_to_m_;
      hw_efforts_[6] = robot_->getCurrentTorqueGripper();
      hw_commands_positions_[6] = hw_positions_[6];  // Initialize commands to current position
    }

    // R_finger_joint (8th joint, index 7) is a mimic joint that follows L_finger_joint
    if (info_.joints.size() > 7)
    {
      hw_positions_[7] = hw_positions_[6];  // Mimic L_finger_joint position
      hw_velocities_[7] = hw_velocities_[6];  // Mimic L_finger_joint velocity
      hw_efforts_[7] = 0.0;  // Passive joint, no actuator
      hw_commands_positions_[7] = hw_positions_[7];
    }

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Initial joint states read successfully");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Failed to read initial joint states: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PantheraHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PantheraHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Skip command interfaces for mimic joints (R_finger_joint)
    if (info_.joints[i].name == "R_finger_joint")
    {
      continue;
    }

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));

    if (use_velocity_commands_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    }

    if (use_effort_commands_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Activating...");

  // Read current state and set as command
  try
  {
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Read 6 arm joint positions
    auto positions = robot_->getCurrentPos();
    for (size_t i = 0; i < 6; i++)
    {
      hw_commands_positions_[i] = positions[i];
    }

    // Read gripper position (7th joint, index 6 = L_finger_joint) if present
    // Convert from radians to meters for prismatic joint
    if (info_.joints.size() > 6)
    {
      double gripper_rad = robot_->getCurrentPosGripper();
      hw_commands_positions_[6] = gripper_rad * gripper_rad_to_m_;
    }

    // R_finger_joint (8th joint, index 7) is a mimic joint that follows L_finger_joint
    if (info_.joints.size() > 7)
    {
      hw_commands_positions_[7] = hw_commands_positions_[6];  // Mimic L_finger_joint position
    }

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Hardware activated successfully");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Failed to activate hardware: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Deactivating...");

  // Optionally send stop command or hold position
  // For safety, we'll just log the deactivation

  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PantheraHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read joint states from hardware
  try
  {
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();

    // Read 6 arm joint states
    auto positions = robot_->getCurrentPos();
    auto velocities = robot_->getCurrentVel();
    auto torques = robot_->getCurrentTorque();

    for (size_t i = 0; i < 6; i++)
    {
      hw_positions_[i] = positions[i];
      hw_velocities_[i] = velocities[i];
      hw_efforts_[i] = torques[i];
    }

    // Read gripper state (7th joint, index 6 = L_finger_joint)
    // Convert from radians to meters for prismatic joint
    if (info_.joints.size() > 6)
    {
      double gripper_rad = robot_->getCurrentPosGripper();
      hw_positions_[6] = gripper_rad * gripper_rad_to_m_;
      hw_velocities_[6] = robot_->getCurrentVelGripper() * gripper_rad_to_m_;
      hw_efforts_[6] = robot_->getCurrentTorqueGripper();
    }

    // R_finger_joint (8th joint, index 7) is a mimic joint that follows L_finger_joint
    if (info_.joints.size() > 7)
    {
      hw_positions_[7] = hw_positions_[6];  // Mimic L_finger_joint position
      hw_velocities_[7] = hw_velocities_[6];  // Mimic L_finger_joint velocity
      hw_efforts_[7] = 0.0;  // Passive joint, no actuator
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PantheraHardwareInterface"),
                          *rclcpp::Clock::make_shared(), 1000,
                          "Failed to read joint states: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PantheraHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write commands to hardware
  try
  {
    // Extract first 6 joints for arm control
    std::vector<double> arm_positions(hw_commands_positions_.begin(),
                                       hw_commands_positions_.begin() + 6);
    std::vector<double> arm_velocities(hw_commands_velocities_.begin(),
                                        hw_commands_velocities_.begin() + 6);
    std::vector<double> arm_efforts(hw_commands_efforts_.begin(),
                                     hw_commands_efforts_.begin() + 6);
    std::vector<double> arm_max_torques(max_torques_.begin(), max_torques_.begin() + 6);
    std::vector<double> arm_max_velocities(max_velocities_.begin(), max_velocities_.begin() + 6);
    std::vector<double> arm_kp(kp_gains_.begin(), kp_gains_.begin() + 6);
    std::vector<double> arm_kd(kd_gains_.begin(), kd_gains_.begin() + 6);

    // Control 6 arm joints
    if (control_mode_ == "full_control")
    {
      // Full control mode: use position, velocity, and effort commands
      robot_->posVelTorqueKpKd(arm_positions, arm_velocities,
                               arm_efforts, arm_kp, arm_kd);
    }
    else if (control_mode_ == "position_velocity")
    {
      // Position-Velocity-MaxTorque control mode
      std::vector<double> velocities = use_velocity_commands_ ? arm_velocities : arm_max_velocities;
      robot_->posVelMaxTorque(arm_positions, velocities, arm_max_torques, false);
    }
    else if (control_mode_ == "pd_control")
    {
      // PD control mode (MIT mode with zero velocity and torque)
      std::vector<double> zero_vel(6, 0.0);
      std::vector<double> zero_torque(6, 0.0);
      robot_->posVelTorqueKpKd(arm_positions, zero_vel, zero_torque, arm_kp, arm_kd);
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PantheraHardwareInterface"),
                            *rclcpp::Clock::make_shared(), 1000,
                            "Unknown control mode: %s", control_mode_.c_str());
      return hardware_interface::return_type::ERROR;
    }

    // Control gripper (7th joint, index 6) if present
    // Convert from meters to radians for hardware
    if (info_.joints.size() > 6)
    {
      double gripper_pos_m = hw_commands_positions_[6];
      double gripper_pos_rad = gripper_pos_m / gripper_rad_to_m_;
      double gripper_vel_m = use_velocity_commands_ ? hw_commands_velocities_[6] : max_velocities_[6];
      double gripper_vel_rad = gripper_vel_m / gripper_rad_to_m_;
      double gripper_max_torque = max_torques_[6];

      if (control_mode_ == "full_control")
      {
        // Use MIT mode for gripper
        double gripper_torque = hw_commands_efforts_[6];
        double gripper_kp = kp_gains_[6];
        double gripper_kd = kd_gains_[6];
        robot_->gripperControlMIT(gripper_pos_rad, gripper_vel_rad, gripper_torque,
                                   gripper_kp, gripper_kd);
      }
      else
      {
        // Use posVelMaxTorque mode for gripper
        robot_->gripperControl(gripper_pos_rad, gripper_vel_rad, gripper_max_torque);
      }
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PantheraHardwareInterface"),
                          *rclcpp::Clock::make_shared(), 1000,
                          "Failed to write commands: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace panthera_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  panthera_hardware::PantheraHardwareInterface, hardware_interface::SystemInterface)
