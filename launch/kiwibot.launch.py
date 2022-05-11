from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

try:
    from vision.utils.cam_handler import find_cameras
except ImportError:
    find_cameras = None

if find_cameras is None:
    camera_names_ports = {
        "left": "/dev/video2",
        "right": "/dev/video0",
        # "back": "/dev/video0",
        # "zoom": "/dev/video0",
    }
else:
    camera_names_ports = {
        "left": "1-1.3:1.0",
        "right": "1-1.2:1.0",
        # "left": "1-13:1.0",
        # "right": "1-3:1.0",
        "back": "1-1.5:1.0",
        "zoom": "1-1.6:1.0",
    }
    usb_ports_cameras = find_cameras()
    print(usb_ports_cameras)
    for camera_name, port in camera_names_ports.items():
        if port in usb_ports_cameras:
            camera_names_ports[camera_name] = f"/dev/video{usb_ports_cameras[port]}"
        else:
            print(f"Camera {camera_name} not found")


base_path = get_package_share_directory("cv_camera")
config_path = "file:///" + os.path.join(base_path, "launch", "camera_info.yaml")


def generate_launch_description():

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

    remappings = [
        ("/left/image_raw", "/video_mapping/left"),
        ("/right/image_raw", "/video_mapping/right"),
        ("/back/image_raw", "/video_mapping/back"),
        ("/zoom/image_raw", "/video_mapping/zoom"),
    ]

    nodes = []
    for camera_name, port in camera_names_ports.items():
        print(read_rate)
        node = Node(
            parameters=[
                {"device_path": port},
                {"image_width": 1280},
                {"image_height": 720},
                {"publish_rate": publish_rate},
                {"read_rate": read_rate},
                {"cv_cap_prop_fourcc": 1196444237.0},
                {"frame_id": "camera"},
                {"camera_info_url": camera_info_url},
            ],
            package="cv_camera",
            executable="cv_camera_node",
            name=f"cv_camera_{camera_name}",
            output="screen",
            namespace=camera_name,
            remappings=remappings,
        )

        nodes.append(node)

    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_publish_rate_cmd,
            declare_read_rate_cmd,
            declare_camera_info_url_cmd,
            *nodes,
        ]
    )