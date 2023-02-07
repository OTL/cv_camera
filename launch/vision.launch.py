"""Launch the vision stack in a component container."""

import launch
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription


def generate_launch_description():
    """Generate launch description with multiple components."""
    return LaunchDescription(
        [
            Node(
                name="vision_kronos",
                package="rclcpp_components",
                executable="component_container",
                output="both",
            ),
            LoadComposableNodes(
                target_container="vision_kronos",
                composable_node_descriptions=[
                    ComposableNode(
                        package="cv_camera", plugin="cv_camera::Driver", name="driver"
                    ),
                ],
            ),
        ]
    )
