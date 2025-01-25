from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,set_environment_variable
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    model_arg = (
        DeclareLaunchArgument(  # Declare the model file path as a launch argument
            name="model",
            default_value=os.path.join(
                get_package_share_directory("manipulator_description"),
                "urdf",
                "bot.urdf.xacro",
            ),
            description="Path to the URDF file",
        )
    )

    # Robot description parameter using xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    # Node to publish the robot state
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Node for joint state publisher with GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # RViz2 node with configuration file
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("manipulator_description"),
                "rviz",
                "manipulator_rviz.rviz",
            ),
        ],
    )

    # Return the launch description
    return LaunchDescription(
        [
            model_arg,
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz_node,
        ]
    )
