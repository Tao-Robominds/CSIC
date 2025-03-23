from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'data_file',
            default_value='',
            description='Path to the point cloud data file'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Frame ID for the point cloud'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='1.0',
            description='Rate at which to publish the point cloud (Hz)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
    ]

    # Configuration
    data_file = LaunchConfiguration('data_file')
    frame_id = LaunchConfiguration('frame_id')
    publish_rate = LaunchConfiguration('publish_rate')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Nodes
    point_cloud_converter_node = Node(
        package='tunnel_segmentation',
        executable='point_cloud_converter',
        name='point_cloud_converter',
        parameters=[{
            'data_file': data_file,
            'frame_id': frame_id,
            'publish_rate': publish_rate,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )
    
    tunnel_segmentation_node = Node(
        package='tunnel_segmentation',
        executable='tunnel_segmentation',
        name='tunnel_segmentation',
        parameters=[{
            'cluster_tolerance': 0.2,
            'min_cluster_size': 100,
            'max_cluster_size': 25000,
            'voxel_size': 0.05,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('tunnel_segmentation'), 'config', 'tunnel_view.rviz']
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    # Create static transform publisher for visualization
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
    )

    return LaunchDescription(
        declared_arguments + 
        [
            point_cloud_converter_node,
            tunnel_segmentation_node,
            static_tf_node,
            rviz_node,
        ]
    ) 