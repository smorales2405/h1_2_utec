import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

# Coupled joint -> the actuated joint of its DOF.
COUPLED = {'thumb_2': 'thumb_1', 'thumb_3': 'thumb_1', 'index_2': 'index_1',
           'middle_2': 'middle_1', 'ring_2': 'ring_1', 'little_2': 'little_1'}


def generate_launch_description():
    pkg = get_package_share_directory('h1_2_inspire_description')

    xacro_file = os.path.join(pkg, 'urdf', 'h1_2_with_inspire_hands.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'h1_2_hands_display.rviz')
    coupling = LaunchConfiguration('coupling')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' coupling:=', coupling]),
        value_type=str
    )
    software = PythonExpression(["'", coupling, "' == 'software'"])

    # coupling:=software: the URDF has no <mimic>. The sliders move only the 6 actuated joints of
    # each hand (the coupled ones are declared dependent, so they get no slider) and publish on
    # joint_states_input; hand_coupling_node computes the 12 joints of each hand from the curves
    # measured on the real hand and publishes the whole robot on joint_states.
    dependent = {f'{side}_{joint}_joint': {'parent': f'{side}_{parent}_joint', 'factor': 0.0}
                 for side in ('left', 'right') for joint, parent in COUPLED.items()}

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config,
            description='Path to RViz config file'
        ),
        DeclareLaunchArgument(
            'coupling',
            default_value='software',
            choices=['software', 'mimic'],
            description='software: curves measured on the real hand (hand_coupling_node); '
                        'mimic: the URDF <mimic> tags (linear approximation)'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'robot_description': robot_description}],
            output='screen',
            condition=UnlessCondition(software)
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'robot_description': robot_description, 'dependent_joints': dependent}],
            remappings=[('joint_states', 'joint_states_input')],
            output='screen',
            condition=IfCondition(software)
        ),

        Node(
            package='h1_2_inspire_description',
            executable='hand_coupling_node.py',
            output='screen',
            condition=IfCondition(software)
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen'
        ),
    ])
