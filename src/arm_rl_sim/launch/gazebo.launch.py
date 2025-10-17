import os
import subprocess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 清理之前的 Gazebo 进程（解决 "Entity already exists" 问题）
    subprocess.run(['pkill', '-9', 'gzserver'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', 'gzclient'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', 'gz'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    import time
    time.sleep(2)  # 增加等待时间，确保进程完全终止
    robot_name_arg = LaunchConfiguration("robot_name")
    
    # 默认使用 arm_t_isaaclab
    robot_name = ParameterValue(Command(["echo -n ", robot_name_arg]), value_type=str)
    ros_namespace = ParameterValue(Command(["echo -n ", "/panther_arm"]), value_type=str)
    gazebo_model_name = ParameterValue(Command(["echo -n ", "panther_arm"]), value_type=str)

    # 使用 arm_description 包中的机械臂模型
    arm_description_share = get_package_share_directory('arm_description')
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(arm_description_share, "urdf", "panther_with_ros2_control.urdf.xacro")
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "verbose": "false",
            "pause": "false",
            "world": "",  # 明确指定空世界
        }.items(),
    )

    # 延迟 spawn_entity，等待 Gazebo 完全启动
    spawn_entity = TimerAction(
        period=2.0,  # 等待 2 秒
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic", "/robot_description", 
                    "-entity", "panther_arm",
                    "-timeout", "120.0",
                    "-robot_namespace", "",
                    "-unpause"  # 立即取消暂停，确保物理引擎启动
                ],
                output="screen",
            )
        ]
    )

    # 延迟启动控制器 spawner（等待 gazebo_ros2_control 完全加载）
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )
    
    robot_joint_controller_group_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["robot_joint_controller_group", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
        }],
    )

    # ARM RL Sim node - 延迟启动，确保控制器已加载 (使用 Python 版本)
    arm_rl_sim_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="arm_rl_sim",
                executable="arm_rl_sim.py",
                name="arm_rl_sim_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    # Target pose publisher (for testing)
    target_pose_publisher = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="arm_rl_sim",
                executable="target_pose_publisher.py",
                name="target_pose_publisher",
                output="screen",
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            description="Robot name (e.g., arm_t_isaaclab)",
            default_value=TextSubstitution(text="arm_t_isaaclab"),
        ),
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        param_node,
        joint_state_broadcaster_spawner,
        robot_joint_controller_group_spawner,
        arm_rl_sim_node,
        target_pose_publisher,
    ])

