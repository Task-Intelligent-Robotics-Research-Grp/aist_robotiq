from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               OpaqueFunction,
                                               IncludeLaunchDescription)
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               ThisLaunchFileDir,
                                               PathJoinSubstitution,
                                               IfElseSubstitution,
                                               EqualsSubstitution)
from launch_ros.substitutions          import FindPackageShare
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context):
    robot_description = ParameterValue(
                            Command([
                                FindExecutable(name='xacro'), ' ',
                                PathJoinSubstitution([
                                    FindPackageShare('aist_robotiq'), 'urdf',
                                    'dual_gripper.urdf'
                                ])
                            ]),
                            value_type=str)
    return [
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[
                 {'robot_description': robot_description}
             ],
             output='screen'),
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(), 'launch.py']),
            launch_arguments=[
                ('gripper_names',  'robotiq_85,right_epick'),
                ('gripper_types',  'RobotiqGripper,RobotiqSuction'),
                ('driver_ns',      'dual_gripper_driver'),
            ]),
        # Node(name=['test_dual_gripper_client'],
        #      package='aist_robotiq',
        #      executable=['test_dual_gripper_client.py'],
        #      # parameters=[{'gripper_name': LaunchConfiguration('gripper_type')}],
        #      prefix=['gnome-terminal --geometry=80x60 --'],
        #      output='screen'),
        Node(name='rviz', package='rviz2', executable='rviz2',
             output='screen',
             arguments=[
                 '-d',
                 PathJoinSubstitution([FindPackageShare('aist_robotiq'),
                                       'launch', 'aist_robotiq.rviz'])
             ]),
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
