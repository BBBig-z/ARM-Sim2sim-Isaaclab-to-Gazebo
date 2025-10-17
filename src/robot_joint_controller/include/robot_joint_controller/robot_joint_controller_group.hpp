#ifndef ROBOT_JOINT_CONTROLLER__ROBOT_JOINT_CONTROLLER_GROUP_HPP_
#define ROBOT_JOINT_CONTROLLER__ROBOT_JOINT_CONTROLLER_GROUP_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"

namespace robot_joint_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotJointControllerGroup : public controller_interface::ControllerInterface
{
public:
  RobotJointControllerGroup();
  virtual ~RobotJointControllerGroup() = default;

#if defined(ROS_DISTRO_HUMBLE)
  CallbackReturn on_init() override;
#endif

  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

#if defined(ROS_DISTRO_FOXY)
  controller_interface::return_type update() override;
#elif defined(ROS_DISTRO_HUMBLE)
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
#endif

private:
  void set_command_callback(const robot_msgs::msg::RobotCommand::SharedPtr msg);
  void update_controller(double period_seconds);

  std::string name_space_;
  std::vector<std::string> joint_names_;
  
  // Command and state
  robot_msgs::msg::RobotCommand last_command_;
  robot_msgs::msg::RobotState last_state_;
  
  // Realtime buffers
  realtime_tools::RealtimeBuffer<robot_msgs::msg::RobotCommand> rt_command_ptr_;
  
  // Publishers and subscribers
  rclcpp::Subscription<robot_msgs::msg::RobotCommand>::SharedPtr command_subscriber_;
  std::shared_ptr<realtime_tools::RealtimePublisher<robot_msgs::msg::RobotState>> state_publisher_;

#if defined(ROS_DISTRO_FOXY)
  rclcpp::Time previous_update_timestamp_;
#endif
};

}  // namespace robot_joint_controller

#endif  // ROBOT_JOINT_CONTROLLER__ROBOT_JOINT_CONTROLLER_GROUP_HPP_

