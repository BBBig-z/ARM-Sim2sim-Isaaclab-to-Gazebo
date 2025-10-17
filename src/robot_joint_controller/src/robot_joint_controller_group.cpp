#include "robot_joint_controller/robot_joint_controller_group.hpp"

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_joint_controller
{
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

RobotJointControllerGroup::RobotJointControllerGroup()
  : controller_interface::ControllerInterface(),
    command_subscriber_(nullptr),
    state_publisher_(nullptr)
{
}

#if defined(ROS_DISTRO_HUMBLE)
CallbackReturn RobotJointControllerGroup::on_init()
{
  return CallbackReturn::SUCCESS;
}
#endif

CallbackReturn RobotJointControllerGroup::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  name_space_ = get_node()->get_namespace();

  if (!get_node()->get_parameter("joints", joint_names_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints given in namespace: '%s'", name_space_.c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Configured %zu joints", joint_names_.size());
  for (const auto & joint_name : joint_names_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "  - %s", joint_name.c_str());
  }

  // Initialize command and state messages
  last_command_.motor_command.resize(joint_names_.size());
  last_state_.motor_state.resize(joint_names_.size());

  // Start command subscriber
  command_subscriber_ = get_node()->create_subscription<robot_msgs::msg::RobotCommand>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotJointControllerGroup::set_command_callback, this, std::placeholders::_1));

  // Start realtime state publisher
  state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_msgs::msg::RobotState>>(
    get_node()->create_publisher<robot_msgs::msg::RobotState>("~/state", rclcpp::SystemDefaultsQoS()));

#if defined(ROS_DISTRO_FOXY)
  previous_update_timestamp_ = get_node()->get_clock()->now();
#endif

  RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotJointControllerGroup::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint_name : joint_names_)
  {
    command_interfaces_config.names.push_back(joint_name + "/" + HW_IF_EFFORT);
  }
  
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration RobotJointControllerGroup::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint_name : joint_names_)
  {
    state_interfaces_config.names.push_back(joint_name + "/" + HW_IF_POSITION);
    state_interfaces_config.names.push_back(joint_name + "/" + HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(joint_name + "/" + HW_IF_EFFORT);
  }
  
  return state_interfaces_config;
}

CallbackReturn RobotJointControllerGroup::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize command with current positions
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    size_t state_idx = i * 3;  // position, velocity, effort
    if (state_idx < state_interfaces_.size())
    {
      double init_pos = state_interfaces_[state_idx].get_value();
      last_command_.motor_command[i].q = init_pos;
      last_state_.motor_state[i].q = init_pos;
    }
    
    last_command_.motor_command[i].dq = 0.0;
    last_command_.motor_command[i].tau = 0.0;
    last_command_.motor_command[i].kp = 0.0;
    last_command_.motor_command[i].kd = 0.0;
    
    last_state_.motor_state[i].dq = 0.0;
    last_state_.motor_state[i].ddq = 0.0;
    last_state_.motor_state[i].tau_est = 0.0;
    last_state_.motor_state[i].cur = 0.0;
  }

  // Reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<robot_msgs::msg::RobotCommand>();

  RCLCPP_INFO(get_node()->get_logger(), "Activated successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotJointControllerGroup::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<robot_msgs::msg::RobotCommand>();
  return CallbackReturn::SUCCESS;
}

#if defined(ROS_DISTRO_FOXY)
controller_interface::return_type RobotJointControllerGroup::update()
{
  const auto current_time = get_node()->get_clock()->now();
  const auto period_seconds = (current_time - previous_update_timestamp_).seconds();
  previous_update_timestamp_ = current_time;
  
  auto joint_command = rt_command_ptr_.readFromRT();
  if (joint_command)
  {
    last_command_ = *joint_command;
  }
  
  update_controller(period_seconds);
  return controller_interface::return_type::OK;
}
#elif defined(ROS_DISTRO_HUMBLE)
controller_interface::return_type RobotJointControllerGroup::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto joint_command = rt_command_ptr_.readFromRT();
  if (joint_command)
  {
    last_command_ = *joint_command;
  }
  
  update_controller(period.seconds());
  return controller_interface::return_type::OK;
}
#endif

void RobotJointControllerGroup::update_controller(double /*period_seconds*/)
{
  // Process each joint
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    size_t state_idx = i * 3;        // position, velocity, effort
    size_t command_idx = i;          // effort command

    // Read current state from hardware interfaces
    if (state_idx + 2 < state_interfaces_.size())
    {
      last_state_.motor_state[i].q = state_interfaces_[state_idx].get_value();
      last_state_.motor_state[i].dq = state_interfaces_[state_idx + 1].get_value();
      last_state_.motor_state[i].tau_est = state_interfaces_[state_idx + 2].get_value();
    }

    // Ensure command has the right size
    if (i >= last_command_.motor_command.size())
    {
      continue;
    }

    // Compute PD control + feedforward torque
    double pos_error = last_command_.motor_command[i].q - last_state_.motor_state[i].q;
    double vel_error = last_command_.motor_command[i].dq - last_state_.motor_state[i].dq;
    double commanded_torque = last_command_.motor_command[i].kp * pos_error + 
                             last_command_.motor_command[i].kd * vel_error + 
                             last_command_.motor_command[i].tau;

    // Send torque command to hardware
    if (command_idx < command_interfaces_.size())
    {
      command_interfaces_[command_idx].set_value(commanded_torque);
    }
  }

  // Publish state
  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_ = last_state_;
    state_publisher_->unlockAndPublish();
  }
}

void RobotJointControllerGroup::set_command_callback(const robot_msgs::msg::RobotCommand::SharedPtr msg)
{
  rt_command_ptr_.writeFromNonRT(*msg);
}

}  // namespace robot_joint_controller

PLUGINLIB_EXPORT_CLASS(
  robot_joint_controller::RobotJointControllerGroup, controller_interface::ControllerInterface)

