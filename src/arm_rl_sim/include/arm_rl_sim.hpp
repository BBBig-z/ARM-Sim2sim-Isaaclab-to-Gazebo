#ifndef ARM_RL_SIM_HPP
#define ARM_RL_SIM_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <gazebo_msgs/srv/set_model_state.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

#include <csignal>

class ARM_RL_Sim : public RL, public rclcpp::Node
{
public:
    ARM_RL_Sim();
    ~ARM_RL_Sim();

private:
    // RL functions
    torch::Tensor Forward() override;
    void GetState(RobotState<double> *state) override;
    void SetCommand(const RobotCommand<double> *command) override;
    void RunModel();
    void RobotControl();

    // history buffer
    ObservationBuffer history_obs_buf;
    torch::Tensor history_obs;

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;

    // ROS interface
    std::string ros_namespace;
    geometry_msgs::msg::PoseStamped target_pose;
    robot_msgs::msg::RobotCommand robot_command_publisher_msg;
    robot_msgs::msg::RobotState robot_state_subscriber_msg;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber;
    rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr robot_command_publisher;
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr robot_state_subscriber;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_client;
    rclcpp::Client<gazebo_msgs::srv::SetModelState>::SharedPtr gazebo_set_model_state_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_pause_physics_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_unpause_physics_client;
    
    void TargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg);

    // others
    std::string gazebo_model_name;
    int motiontime = 0;
};

#endif // ARM_RL_SIM_HPP

