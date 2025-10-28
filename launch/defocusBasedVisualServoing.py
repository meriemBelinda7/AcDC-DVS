from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo,TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value= "192.168.1.3", description="robot ip"),
        DeclareLaunchArgument('Image_topic', default_value= "Camera/Image", description="Name of the image topic"),
        DeclareLaunchArgument('Current_focus_topic', default_value= "/camera/Current_focus", description="Name of the topic that carries the current value of focus"),
        DeclareLaunchArgument('current_camera_position', default_value= "/camera/Current_pos", description="Name of the topic that carries the current camera position"),
        DeclareLaunchArgument('relative_pos_topic', default_value= "cmd/relative_pos", description="Name of the topic that carries the relative position to go to the initial position"),
        DeclareLaunchArgument('camera_velocity_topic', default_value= "cmd/camera_velocity", description="Name of the topic that carries the velocity in the camera frame"),
        DeclareLaunchArgument('twist_topic_', default_value= "/twist_msgy", description="Name of the topic that carries the twist_msg"),
        DeclareLaunchArgument('Desired_focus_topic', default_value= "/camera/focus", description="Name of the topic that carries the desired value of focus"),
        DeclareLaunchArgument('Initial_focus', default_value= "0", description="Initial value of the focus"),
        DeclareLaunchArgument('Frame_rate', default_value= "30", description="Frame rate"),
        DeclareLaunchArgument('Height', default_value= "1920", description="Height of the image"),
        DeclareLaunchArgument('Width', default_value= "1080", description="Width of the image"),
        DeclareLaunchArgument('binning', default_value= "6", description="Binning Value"),
        DeclareLaunchArgument('Log_folder', default_value=PathJoinSubstitution([EnvironmentVariable('HOME'),'.ros','ros2_dvs_bridge','3D']),description="path to log file"),
        # DeclareLaunchArgument('Log_folder', default_value=PathJoinSubstitution([EnvironmentVariable('HOME'),'.ros','ros2_dvs_bridge','exp_28_0_7_2025','monroe_fronto_parallel_6dof']),description="path to log file"),
        #DeclareLaunchArgument('Log_folder', default_value=PathJoinSubstitution([EnvironmentVariable('HOME'),'.ros','ros2_dvs_bridge','pgmvsf']),description="path to log file"),
        DeclareLaunchArgument('diffTopic', default_value= "Id_diff", description="Name of the difference image topic"),
        
        DeclareLaunchArgument('errTopic', default_value= "/Residuals", description="Name of the residual topic"),
        DeclareLaunchArgument('Lambda', default_value= "1.0", description="Proportional value"),
        DeclareLaunchArgument('focal', default_value= "0.0005", description="Focal length"),
        DeclareLaunchArgument('pixelSize', default_value= "0.0000022", description="physical size of a pixel"),
        DeclareLaunchArgument('u0', default_value= "160.0", description="principal point"),
        DeclareLaunchArgument('v0', default_value= "90.0", description="principal point"),
        DeclareLaunchArgument('FNumber', default_value= "2.5", description="Lens aperture"),
        DeclareLaunchArgument('Zf', default_value= "0.20", description="Focus depth"),
        DeclareLaunchArgument('tx', default_value= "true", description="Translation along X-axis"),
        DeclareLaunchArgument('ty', default_value= "true", description="Translation along y-axis"),
        DeclareLaunchArgument('tz', default_value= "true", description="Translation along z-axis"),
        DeclareLaunchArgument('rx', default_value= "false", description="rotation along X-axis"),
        DeclareLaunchArgument('ry', default_value= "true", description="rotation along y-axis"),
        DeclareLaunchArgument('rz', default_value= "false", description="rotation along z-axis"),


        # DeclareLaunchArgument('tx', default_value= "false", description="Translation along X-axis"),
        # DeclareLaunchArgument('ty', default_value= "false", description="Translation along y-axis"),
        # DeclareLaunchArgument('tz', default_value= "false", description="Translation along z-axis"),
        # DeclareLaunchArgument('rx', default_value= "false", description="rotation along X-axis"),
        # DeclareLaunchArgument('ry', default_value= "false", description="rotation along y-axis"),
        # DeclareLaunchArgument('rz', default_value= "false", description="rotation along z-axis"),
        # #DeclareLaunchArgument('sceneDepth', default_value= "0.20", description="Depth of the scene"),
        DeclareLaunchArgument('sceneDepth', default_value= "0.20", description="Depth of the scene"),

        DeclareLaunchArgument('controlInBaseFrame', default_value= "True", description=""),
        
        # DeclareLaunchArgument('dtX', default_value= "0.04", description="Relative initial position"),
        # DeclareLaunchArgument('dtY', default_value= "0.08", description="Relative initial position"),
        # DeclareLaunchArgument('dtZ', default_value= "0.21", description="Relative initial position"),
        # DeclareLaunchArgument('drX', default_value= "-7.0", description="Relative initial position"),
        # DeclareLaunchArgument('drY', default_value= "7.0", description="Relative initial position"),
        # DeclareLaunchArgument('drZ', default_value= "10.0", description="Relative initial position"),
        
        # DeclareLaunchArgument('dtX', default_value= "-0.0716", description="Relative initial position"),
        # DeclareLaunchArgument('dtY', default_value= "0.0464", description="Relative initial position"),
        # DeclareLaunchArgument('dtZ', default_value= "-0.165", description="Relative initial position"),
        # DeclareLaunchArgument('drX', default_value= "0.0", description="Relative initial position"),
        # DeclareLaunchArgument('drY', default_value= "-18.0", description="Relative initial position"),
        # DeclareLaunchArgument('drZ', default_value= "0.0", description="Relative initial position"),

        # DeclareLaunchArgument('dtX', default_value= "-0.29", description="Relative initial position"),
        # DeclareLaunchArgument('dtY', default_value= "0.03", description="Relative initial position"),
        # DeclareLaunchArgument('dtZ', default_value= "-0.034", description="Relative initial position"),
        # DeclareLaunchArgument('drX', default_value= "0.0", description="Relative initial position"),
        # DeclareLaunchArgument('drY', default_value= "-30.0", description="Relative initial position"),
        # DeclareLaunchArgument('drZ', default_value= "0.0", description="Relative initial position"),

        DeclareLaunchArgument('dtX', default_value= "0.0", description="Relative initial position"),
        DeclareLaunchArgument('dtY', default_value= "0.1", description="Relative initial position"),
        DeclareLaunchArgument('dtZ', default_value= "0.0", description="Relative initial position"),
        DeclareLaunchArgument('drX', default_value= "0.0", description="Relative initial position"),
        DeclareLaunchArgument('drY', default_value= "0.0", description="Relative initial position"),
        DeclareLaunchArgument('drZ', default_value= "0.0", description="Relative initial position"),
        
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
                        'binning': LaunchConfiguration('binning')}],
            output     = 'screen'
        ),
        # Node(
        #     package    = "ur_control",
        #     namespace  = "",
        #     executable = "ur_control_node",
        #     name       = "control_node",
        #     remappings = [('relative_pos_topic','cmd/relative_pos'),
        #                   ('camera_velocity_topic','cmd/camera_velocity')],

        #     parameters=[{'relative_pos_topic': LaunchConfiguration('relative_pos_topic'),
        #                 'camera_velocity_topic': LaunchConfiguration('camera_velocity_topic'),}],
        #     output     = 'screen'
        # ),
        
        TimerAction(
            period=2.0,
            actions=[
            Node(
            package="rqt_gui",
            executable="rqt_gui",
            name="rqt_gui",
            output="screen"
            ),
            Node(
            package    = "indicator_save",
            namespace  = "",
            executable = "indicator",
            name       = "indicator_node",
            remappings = [('current_camera_position','/camera/Current_pos'),
                          ('camera_velocity_topic','cmd/camera_velocity'),],

            parameters=[{'current_camera_position': LaunchConfiguration('current_camera_position'),
                        'camera_velocity_topic': LaunchConfiguration('camera_velocity_topic'),
                        'errTopic': LaunchConfiguration('errTopic'),
                        'Log_folder': LaunchConfiguration('Log_folder'),
                        'Current_focus_topic': LaunchConfiguration('Current_focus_topic'),
                        }],
            output     = 'screen'
        ),
            Node(

                package="ros2_dvs_bridge",
                namespace="",
                executable="ros2_dvs_bridge_ddvsDirectVisualservoing",
                name="ddvsDirectVisualservoing",
               remappings=[('image', 'Camera/Image'),
                            ('relative_pos_topic','cmd/relative_pos'),
                            ('camera_velocity_topic','cmd/camera_velocity')],
                parameters=[{'Image_topic': LaunchConfiguration('Image_topic'),
                             'relative_pos_topic': LaunchConfiguration('relative_pos_topic'),
                             'camera_velocity_topic': LaunchConfiguration('camera_velocity_topic'),
                             'twist_topic_':LaunchConfiguration('twist_topic'),
                             'Log_folder': LaunchConfiguration('Log_folder'),
                            'diffTopic': LaunchConfiguration('diffTopic'),
                            'Image_topic': LaunchConfiguration('Image_topic'),
                            'errTopic': LaunchConfiguration('errTopic'),
                            'Desired_focus_topic': LaunchConfiguration('Desired_focus_topic'),
                            'Lambda': LaunchConfiguration('Lambda'),
                            'focal': LaunchConfiguration('focal'),
                            'Initial_focus': LaunchConfiguration('Initial_focus'),
                            'pixelSize': LaunchConfiguration('pixelSize'),
                            'u0': LaunchConfiguration('u0'),
                            'v0': LaunchConfiguration('v0'),
                            'FNUmber': LaunchConfiguration('FNumber'),
                            'Zf': LaunchConfiguration('Zf'),
                            'binning': LaunchConfiguration('binning'),
                            'tx': LaunchConfiguration('tx'),
                            'ty': LaunchConfiguration('ty'),
                            'tz': LaunchConfiguration('tz'),
                            'rx': LaunchConfiguration('rx'),
                            'ry': LaunchConfiguration('ry'),
                            'rz': LaunchConfiguration('rz'),
                            'dtX': LaunchConfiguration('dtX'),
                            'dtY': LaunchConfiguration('dtY'),
                            'dtZ': LaunchConfiguration('dtZ'),
                            'drX': LaunchConfiguration('drX'),
                            'drY': LaunchConfiguration('drY'),
                            'drZ': LaunchConfiguration('drZ')
                             
                             }],
                output='screen'
                )
            ]       
        )

        
        
        





    ])