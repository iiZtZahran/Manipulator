from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable,IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    manipulator_dir = get_package_share_directory("manipulator_description")
  
    model_arg = DeclareLaunchArgument( # Declare the model file path as a launch argument
        name="model",
        default_value=os.path.join(
            get_package_share_directory("manipulator_description"),
            "urdf",
            "bot.urdf.xacro",
        ),
        description="Path to the URDF file",
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value= [str(Path(manipulator_dir).parent.resolve())]
    )
    
    ros_distro = os.environ["ROS_DISTRO"]
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"
    
    
    # Robot description parameter using xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    # Node to publish the robot state
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )  
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch"
                ),"/gz_sim.launch.py"]
            
        ),
        launch_arguments=[(
            "gz_args",[" -v 4 -r empty.sdf ", physics_engine]
        )]
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic","robot_description","-name","manipulator"]
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.clock]"
        ]
    )
    
    
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])