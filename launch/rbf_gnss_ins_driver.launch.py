import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rbf_gnss_ins_driver'),
        'config',
        'rbf_gnss_ins_driver.param.yaml'
    )

    container = ComposableNodeContainer(
    name='rbf_gnss_ins_driver_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='rbf_gnss_ins_driver',
            plugin='rbf_gnss_ins_driver::GnssInsDriver',
            name='rbf_gnss_ins_driver',
            parameters=[config],
            extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    )

    return launch.LaunchDescription([container])