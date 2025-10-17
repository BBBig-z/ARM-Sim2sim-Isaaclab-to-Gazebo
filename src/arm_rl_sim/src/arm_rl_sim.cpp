#include "arm_rl_sim.hpp"

ARM_RL_Sim::ARM_RL_Sim()
    : rclcpp::Node("arm_rl_sim_node")
{
    this->ros_namespace = this->get_namespace();

    // get params from param_node
    param_client = this->create_client<rcl_interfaces::srv::GetParameters>("/param_node/get_parameters");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()) {
            std::cout << LOGGER::ERROR << "Interrupted while waiting for param_node service. Exiting." << std::endl;
            return;
        }
        std::cout << LOGGER::WARNING << "Waiting for param_node service to be available..." << std::endl;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_name", "gazebo_model_name"};

    // Use a timeout for the future
    auto future = param_client->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5));

    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (result->values.size() < 2)
        {
            std::cout << LOGGER::ERROR << "Failed to get all parameters from param_node" << std::endl;
        }
        else
        {
            this->robot_name = result->values[0].string_value;
            this->gazebo_model_name = result->values[1].string_value;
            std::cout << LOGGER::INFO << "Get param robot_name: " << this->robot_name << std::endl;
            std::cout << LOGGER::INFO << "Get param gazebo_model_name: " << this->gazebo_model_name << std::endl;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "Failed to call param_node service" << std::endl;
    }

    // read params from yaml
    this->ReadYaml(this->robot_name);

    // init rl
    torch::autograd::GradMode::set_enabled(false);
    if (this->params.observations_history.size() != 0)
    {
        this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, this->params.observations_history.size());
    }
    this->robot_command_publisher_msg.motor_command.resize(this->params.num_of_dofs);
    this->robot_state_subscriber_msg.motor_state.resize(this->params.num_of_dofs);
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();
    running_state = STATE_RL_RUNNING;

    // model
    std::string model_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + this->robot_name + "/" + this->params.model_name;
    this->model = torch::jit::load(model_path);

    // publisher
    this->robot_command_publisher = this->create_publisher<robot_msgs::msg::RobotCommand>(
        this->ros_namespace + "robot_joint_controller/command", rclcpp::SystemDefaultsQoS());

    // subscriber
    this->target_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {this->TargetPoseCallback(msg);}
    );
    this->robot_state_subscriber = this->create_subscription<robot_msgs::msg::RobotState>(
        this->ros_namespace + "robot_joint_controller/state", rclcpp::SystemDefaultsQoS(),
        [this] (const robot_msgs::msg::RobotState::SharedPtr msg) {this->RobotStateCallback(msg);}
    );

    // service
    this->gazebo_set_model_state_client = this->create_client<gazebo_msgs::srv::SetModelState>("/gazebo/set_model_state");
    this->gazebo_pause_physics_client = this->create_client<std_srvs::srv::Empty>("/gazebo/pause_physics");
    this->gazebo_unpause_physics_client = this->create_client<std_srvs::srv::Empty>("/gazebo/unpause_physics");

    // 初始化目标位姿（默认值）
    this->target_pose.pose.position.x = 0.3;
    this->target_pose.pose.position.y = 0.0;
    this->target_pose.pose.position.z = 0.3;
    this->target_pose.pose.orientation.w = 1.0;
    this->target_pose.pose.orientation.x = 0.0;
    this->target_pose.pose.orientation.y = 0.0;
    this->target_pose.pose.orientation.z = 0.0;

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&ARM_RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&ARM_RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&ARM_RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

    std::cout << LOGGER::INFO << "ARM_RL_Sim start" << std::endl;
}

ARM_RL_Sim::~ARM_RL_Sim()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    std::cout << LOGGER::INFO << "ARM_RL_Sim exit" << std::endl;
}

void ARM_RL_Sim::GetState(RobotState<double> *state)
{
    // 机械臂不需要 IMU 数据，只需要关节状态
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = this->robot_state_subscriber_msg.motor_state[i].q;
        state->motor_state.dq[i] = this->robot_state_subscriber_msg.motor_state[i].dq;
        state->motor_state.tau_est[i] = this->robot_state_subscriber_msg.motor_state[i].tau_est;
    }
}

void ARM_RL_Sim::SetCommand(const RobotCommand<double> *command)
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->robot_command_publisher_msg.motor_command[i].q = command->motor_command.q[i];
        this->robot_command_publisher_msg.motor_command[i].dq = command->motor_command.dq[i];
        this->robot_command_publisher_msg.motor_command[i].kp = command->motor_command.kp[i];
        this->robot_command_publisher_msg.motor_command[i].kd = command->motor_command.kd[i];
        this->robot_command_publisher_msg.motor_command[i].tau = command->motor_command.tau[i];
    }

    this->robot_command_publisher->publish(this->robot_command_publisher_msg);
}

void ARM_RL_Sim::RobotControl()
{
    if (simulation_running)
    {
        this->motiontime++;
        this->GetState(&this->robot_state);
        this->StateController(&this->robot_state, &this->robot_command);
        this->SetCommand(&this->robot_command);
    }
}

void ARM_RL_Sim::TargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->target_pose = *msg;
}

void ARM_RL_Sim::RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg)
{
    this->robot_state_subscriber_msg = *msg;
}

void ARM_RL_Sim::RunModel()
{
    if (this->running_state == STATE_RL_RUNNING && simulation_running)
    {
        // 构建观察值
        // 1. 关节位置
        this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        
        // 2. 关节速度
        this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        
        // 3. 目标位姿（7维：xyz + quat(wxyz)）
        std::vector<double> pose_cmd = {
            this->target_pose.pose.position.x,
            this->target_pose.pose.position.y,
            this->target_pose.pose.position.z,
            this->target_pose.pose.orientation.w,
            this->target_pose.pose.orientation.x,
            this->target_pose.pose.orientation.y,
            this->target_pose.pose.orientation.z
        };
        torch::Tensor target_pose_tensor = torch::tensor(pose_cmd).unsqueeze(0);
        
        // 注意：这里需要一个自定义的观察值字段来存储 target_pose
        // 由于 RL SDK 中的 Observations 结构体没有 target_pose 字段，
        // 我们临时使用 base_quat 来存储（需要后续修改 SDK）
        // 为了兼容性，我们直接在 Forward() 中处理
        
        torch::Tensor clamped_actions = this->Forward();

        for (int i : this->params.hip_scale_reduction_indices)
        {
            clamped_actions[0][i] *= this->params.hip_scale_reduction;
        }

        this->obs.actions = clamped_actions;

        torch::Tensor origin_output_torques = this->ComputeTorques(this->obs.actions);

        this->output_torques = torch::clamp(origin_output_torques, -(this->params.torque_limits), this->params.torque_limits);
        this->output_dof_pos = this->ComputePosition(this->obs.actions);
    }
}

torch::Tensor ARM_RL_Sim::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    // 手动构建观察值（因为 ComputeObservation 需要修改以支持 pose_command）
    std::vector<torch::Tensor> obs_tensors;
    
    for (auto &obs_name : this->params.observations)
    {
        if (obs_name == "joint_pos")
        {
            obs_tensors.push_back(this->obs.dof_pos * this->params.dof_pos_scale);
        }
        else if (obs_name == "joint_vel")
        {
            obs_tensors.push_back(this->obs.dof_vel * this->params.dof_vel_scale);
        }
        else if (obs_name == "pose_command")
        {
            // 构建目标位姿张量
            std::vector<double> pose_cmd = {
                this->target_pose.pose.position.x,
                this->target_pose.pose.position.y,
                this->target_pose.pose.position.z,
                this->target_pose.pose.orientation.w,
                this->target_pose.pose.orientation.x,
                this->target_pose.pose.orientation.y,
                this->target_pose.pose.orientation.z
            };
            obs_tensors.push_back(torch::tensor(pose_cmd).unsqueeze(0));
        }
        else if (obs_name == "actions")
        {
            obs_tensors.push_back(this->obs.actions);
        }
    }
    
    torch::Tensor clamped_obs = torch::clamp(torch::cat(obs_tensors, 1), -this->params.clip_obs, this->params.clip_obs);

    torch::Tensor actions;
    if (this->params.observations_history.size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.observations_history);
        actions = this->model.forward({this->history_obs}).toTensor();
    }
    else
    {
        actions = this->model.forward({clamped_obs}).toTensor();
    }

    if (this->params.clip_actions_upper.numel() != 0 && this->params.clip_actions_lower.numel() != 0)
    {
        return torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);
    }
    else
    {
        return actions;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ARM_RL_Sim>());
    rclcpp::shutdown();
    return 0;
}

