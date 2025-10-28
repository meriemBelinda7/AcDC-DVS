from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo,TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('robot_ip', default_value= "192.168.1.3", description="robot ip"),
        DeclareLaunchArgument('Image_topic', default_value= "Camera/Image", description="Name of the image topic"),
        DeclareLaunchArgument('Current_focus_topic', default_value= "camera/Current_focus", description="Name of the topic that carries the current value of focus"),
        DeclareLaunchArgument('joint_topic', default_value= "cmd/joint", description="Name of the topic that carries the topic of the joint position at the desired position"),

        DeclareLaunchArgument('Desired_focus_topic', default_value= "camera/Desired_focus", description="Name of the topic that carries the desired value of focus"),
        DeclareLaunchArgument('Initial_focus', default_value= "0", description="Initial value of the focus"),
        DeclareLaunchArgument('Frame_rate', default_value= "30", description="Frame rate"),
        DeclareLaunchArgument('Height', default_value= "1920", description="Height of the image"),
        DeclareLaunchArgument('Width', default_value= "1080", description="Width of the image"),
        DeclareLaunchArgument('binning', default_value= "6", description="Binning Value"),
        #DeclareLaunchArgument('Log_folder', default_value=PathJoinSubstitution([EnvironmentVariable('HOME'),'.ros','ros2_dvs_bridge','exp_28_0_7_2025','monroe_fronto_parallel_6dof']),description="path to log file"),
        DeclareLaunchArgument('Log_folder', default_value=PathJoinSubstitution([EnvironmentVariable('HOME'),'.ros','ros2_dvs_bridge','3D']),description="path to log file"),
        DeclareLaunchArgument('Auto_focus', default_value= "true", description="Activating/deactivating autofocus mode"),
        DeclareLaunchArgument('endTopic', default_value= "/autofocus", description="Activating/deactivating autofocus topic"),


        DeclareLaunchArgument('J1', default_value= "79.03", description="Joint position at desired position"),
        DeclareLaunchArgument('J2', default_value= "-10.30", description="Joint position at desired position"),
        DeclareLaunchArgument('J3', default_value= "55.89", description="Joint position at desired position"),
        DeclareLaunchArgument('J4', default_value= "-229.74", description="Joint position at desired position"),
        DeclareLaunchArgument('J5', default_value= "-95.46", description="Joint position at desired position"),
        DeclareLaunchArgument('J6', default_value= "88.55", description="Joint position at desired position"),   
        
        # #6dof
        # DeclareLaunchArgument('J1', default_value= "42.69", description="Joint position at desired position"),
        # DeclareLaunchArgument('J2', default_value= "-103.65", description="Joint position at desired position"),
        # DeclareLaunchArgument('J3', default_value= "143.75", description="Joint position at desired position"),
        # DeclareLaunchArgument('J4', default_value= "-131.41", description="Joint position at desired position"),
        # DeclareLaunchArgument('J5', default_value= "-87.61", description="Joint position at desired position"),
        # DeclareLaunchArgument('J6', default_value= "51.74", description="Joint position at desired position"),

        #initial error along Z
        # DeclareLaunchArgument('J1', default_value= "51.42", description="Joint position at desired position"),
        # DeclareLaunchArgument('J2', default_value= "-82.98", description="Joint position at desired position"),
        # DeclareLaunchArgument('J3', default_value= "147.49", description="Joint position at desired position"),
        # DeclareLaunchArgument('J4', default_value= "-156.17", description="Joint position at desired position"),
        # DeclareLaunchArgument('J5', default_value= "-87.96", description="Joint position at desired position"),
        # DeclareLaunchArgument('J6', default_value= "60.44", description="Joint position at desired position"),

        #hex_key
        #DeclareLaunchArgument('J1', default_value= "53.61", description="Joint position at desired position"),
        #DeclareLaunchArgument('J2', default_value= "-61.21", description="Joint position at desired position"),
        #DeclareLaunchArgument('J3', default_value= "128.10", description="Joint position at desired position"),
        #DeclareLaunchArgument('J4', default_value= "-156.17", description="Joint position at desired position"),
        #DeclareLaunchArgument('J5', default_value= "-88.47", description="Joint position at desired position"),
        #DeclareLaunchArgument('J6', default_value= "-2.61", description="Joint position at desired position"),
        #DeclareLaunchArgument("INDICATOR", default_value= "true", description="To enable/desable datasaving"),

        #mc_rtc test
        # DeclareLaunchArgument('J1', default_value= "51.20", description="Joint position at desired position"),
        # DeclareLaunchArgument('J2', default_value= "-95.85", description="Joint position at desired position"),
        # DeclareLaunchArgument('J3', default_value= "144.76", description="Joint position at desired position"),
        # DeclareLaunchArgument('J4', default_value= "-140.81", description="Joint position at desired position"),
        # DeclareLaunchArgument('J5', default_value= "-88.06", description="Joint position at desired position"),
        # DeclareLaunchArgument('J6', default_value= "67.87", description="Joint position at desired position"),
        DeclareLaunchArgument("INDICATOR", default_value= "true", description="To enable/desable datasaving"),

        Node(
            package    = "the_imaging_source",
            namespace  = "",
            executable = "TIS_image_publisher",
            name       = "Image_publisher",
            remappings = [('/image', "/Camera/Image")],

            parameters=[{'Image_topic': LaunchConfiguration('Image_topic'),
                        'Current_focus_topic': LaunchConfiguration('Current_focus_topic'),
                        'Desired_focus_topic': LaunchConfiguration('Desired_focus_topic'),
                        'Initial_focus': LaunchConfiguration('Initial_focus'),
                        'Frame_rate': LaunchConfiguration('Frame_rate'),
                        'Height': LaunchConfiguration('Height'),
                        'Width': LaunchConfiguration('Width'),
                        'binning': LaunchConfiguration('binning'),
                        'Auto_focus': LaunchConfiguration('Auto_focus'),
                        'endTopic': LaunchConfiguration('endTopic')}],
            output     = 'screen'
        ),
        Node(
            package    = "ur_control",
            namespace  = "",
            executable = "ur_control_node",
            name       = "control_node",
            remappings = [("joint_topic", "cmd/joint")],

            parameters=[{'joint_topic': LaunchConfiguration('joint_topic'),
                         'robot_ip': LaunchConfiguration('robot_ip')}],
            output     = 'screen'
        ),
        
        TimerAction(
            period=2.0,
            actions=[
            Node(
                package="ros2_dvs_bridge",
                namespace="",
                executable="CaptureAndSaveDesiredImage",
                name="CaptureAndSaveDesiredImage",
                remappings=[('image', 'Camera/Image')],
                parameters=[{'Image_topic': LaunchConfiguration('Image_topic'),
                             'joint_topic': LaunchConfiguration('joint_topic'),
                             'Log_folder': LaunchConfiguration('Log_folder'),
                             'J1': LaunchConfiguration('J1'),
                             'J2': LaunchConfiguration('J2'),
                             'J3': LaunchConfiguration('J3'),
                             'J4': LaunchConfiguration('J4'),
                             'J5': LaunchConfiguration('J5'),
                             'J6': LaunchConfiguration('J6'),
                             'INDICATOR': LaunchConfiguration('INDICATOR')
                             }],
                output='screen'
                )
            ]       
        )

        
        
        





    ])