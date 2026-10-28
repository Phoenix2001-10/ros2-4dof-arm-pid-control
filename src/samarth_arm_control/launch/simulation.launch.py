from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # -------------------------------------------------
    # Robot description from Xacro
    # -------------------------------------------------

    xacro_file = PathJoinSubstitution([
        FindPackageShare('samarth_arm_description'),
        'urdf',
        'samarth_arm.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )


    # -------------------------------------------------
    # Robot State Publisher
    # -------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )


    # -------------------------------------------------
    # Gazebo
    # -------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),

        launch_arguments={
            'gz_args':
            '-r -v 3 empty.sdf --render-engine ogre'
        }.items()
    )


    # -------------------------------------------------
    # Spawn robot into Gazebo
    # -------------------------------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',

        arguments=[
            '-world', 'empty',
            '-name', 'samarth_arm',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ]
    )


    # -------------------------------------------------
    # Gazebo -> ROS simulation clock
    # -------------------------------------------------

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )


    # -------------------------------------------------
    # Joint State Broadcaster
    # -------------------------------------------------

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',

        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30'
        ],

        output='screen'
    )


    # -------------------------------------------------
    # Arm Trajectory Controller
    # -------------------------------------------------

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',

        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30'
        ],

        output='screen'
    )
    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'effort_controller',
            '--controller-manager',
            '/controller_manager',
            '--inactive'
       ],
       output='screen'
    )

    # -------------------------------------------------
    # Load controllers only after robot is spawned
    # -------------------------------------------------

    load_joint_state_broadcaster = RegisterEventHandler(

        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner
            ]
        )
    )


    load_arm_controller = RegisterEventHandler(

        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                arm_controller_spawner
            ]
        )
    )

    load_effort_controller = RegisterEventHandler(

        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                effort_controller_spawner
            ]
        )
    )  
    return LaunchDescription([

        robot_state_publisher,

        gazebo,

        clock_bridge,

        spawn_robot,

        load_joint_state_broadcaster,

        load_effort_controller,
 
        load_arm_controller

    ])
