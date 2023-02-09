"""Launch the vision stack in a component container."""

from launch_ros.actions import LoadComposableNodes, Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
import cv2

config_path = "/workspace/rover/ros2/src/vision_cpp/cv_camera/launch/camera_info.yaml"

camera_names_ports = {
    "left": "3-4.4.4:1.0",
    "right": "3-4.4.3:1.0",
    # "back": "1-1.5:1.0",
    # "zoom": "1-1.6:1.0",
}

video_format = "MJPG"


def generate_launch_description():

    """Declare parameters"""
    publish_rate = LaunchConfiguration("publish_rate")
    declare_publish_rate_cmd = DeclareLaunchArgument(
        "publish_rate",
        default_value="10.0",
        description="Rate of publish images",
    )
    camera_info_url = LaunchConfiguration("camera_info_url")
    declare_camera_info_url_cmd = DeclareLaunchArgument(
        "camera_info_url",
        default_value=config_path,
        description="path to the camera info file",
    )
    read_rate = LaunchConfiguration("read_rate")
    declare_read_rate_cmd = DeclareLaunchArgument(
        "read_rate",
        default_value="30.0",
        description="Rate of read images",
    )
    cam_format = LaunchConfiguration("cv_cap_prop_fourcc")
    declare_cam_format_cmd = DeclareLaunchArgument(
        "cv_cap_prop_fourcc",
        default_value=str(float(cv2.VideoWriter_fourcc(*video_format))),
        description="Format to open the cameras",
    )
    # remappings = [
    #     ("/left/image_raw", "/video_mapping/left"),
    #     ("/right/image_raw", "/video_mapping/right"),
    #     ("/back/image_raw", "/video_mapping/back"),
    #     ("/zoom/image_raw", "/video_mapping/zoom"),
    # ]

    """Iter through available cameras"""
    nodes = []
    for camera_name, port in camera_names_ports.items():
        node = ComposableNode(
            parameters=[
                {"device_path": port},
                {"image_width": 640},
                {"image_height": 360},
                {"publish_rate": publish_rate},
                {"read_rate": read_rate},
                {"cv_cap_prop_fourcc": cam_format},
                {"frame_id": "camera"},
                {"camera_info_url": camera_info_url},
                {"topic_name": "/video_mapping/" + camera_name},
            ],
            package="cv_camera",
            plugin="cv_camera::Driver",
            namespace=camera_name,
            name=f"cv_camera_{camera_name}",
            # remappings=remappings,
        )
        nodes.append(node)

    """Create the launch description"""
    return LaunchDescription(
        [
            declare_publish_rate_cmd,
            declare_read_rate_cmd,
            declare_camera_info_url_cmd,
            declare_cam_format_cmd,
            Node(
                name="vision_kronos",
                package="rclcpp_components",
                executable="component_container",
                output="both",
            ),
            LoadComposableNodes(
                target_container="vision_kronos",
                composable_node_descriptions=[*nodes],
            ),
        ]
    )
