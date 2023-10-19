from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text=str("WARN")),
        description="Logging level",
    )

    stream0_camera_args = {
        "camera0/identifier": DeclareLaunchArgument(
            "camera0/identifier",
            default_value=TextSubstitution(text=".*bfly.*"),
            description="Camera descriptor for the first stream.",
        ),
        "camera0/topic": DeclareLaunchArgument(
            "camera0/topic",
            default_value=TextSubstitution(text="stream0"),
            description="Topic to publish to for the first stream.",
        ),
        "camera0/binning": DeclareLaunchArgument(
            "camera0/binning",
            default_value=TextSubstitution(text="1"),
            description="Binning for the first stream.",
        ),
        "camera0/exposure_time_us": DeclareLaunchArgument(
            "camera0/exposure_time_us",
            default_value=TextSubstitution(text="10000"),
            description="Exposure time for the first stream.",
        ),
        "camera0/image_size": DeclareLaunchArgument(
            "camera0/image_size",
            default_value=TextSubstitution(text="[1920,1080]"),
            description="Image size for the first stream.",
        ),
        "camera1/identifier": DeclareLaunchArgument(
            "camera1/identifier",
            default_value=TextSubstitution(text=".*random.*"),
            description="Camera descriptor for the second stream.",
        ),
        "camera1/topic": DeclareLaunchArgument(
            "camera1/topic",
            default_value=TextSubstitution(text="stream1"),
            description="Topic to publish to for the second stream.",
        ),
        "camera1/binning": DeclareLaunchArgument(
            "camera1/binning",
            default_value=TextSubstitution(text="1"),
            description="Binning for the second stream.",
        ),
        "camera1/exposure_time_us": DeclareLaunchArgument(
            "camera1/exposure_time_us",
            default_value=TextSubstitution(text="20000"),
            description="Exposure time for the second stream.",
        ),
        "camera1/image_size": DeclareLaunchArgument(
            "camera1/image_size",
            default_value=TextSubstitution(text="[640,480]"),
            description="Image size for the second stream.",
        ),
    }

    stream1_camera_args = {
        "camera2/identifier": DeclareLaunchArgument(
            "camera2/identifier",
            default_value=TextSubstitution(text=".*random.*"),
            description="Camera descriptor for the third stream.",
        ),
        "camera2/topic": DeclareLaunchArgument(
            "camera2/topic",
            default_value=TextSubstitution(text="stream2"),
            description="Topic to publish to for the third stream.",
        ),
        "camera2/binning": DeclareLaunchArgument(
            "camera2/binning",
            default_value=TextSubstitution(text="1"),
            description="Binning for the third stream.",
        ),
        "camera2/exposure_time_us": DeclareLaunchArgument(
            "camera2/exposure_time_us",
            default_value=TextSubstitution(text="20000"),
            description="Exposure time for the third stream.",
        ),
        "camera2/image_size": DeclareLaunchArgument(
            "camera2/image_size",
            default_value=TextSubstitution(text="[640,480]"),
            description="Image size for the third stream.",
        ),
        "camera3/identifier": DeclareLaunchArgument(
            "camera3/identifier",
            default_value=TextSubstitution(text=".*radial.*"),
            description="Camera descriptor for the fourth stream.",
        ),
        "camera3/topic": DeclareLaunchArgument(
            "camera3/topic",
            default_value=TextSubstitution(text="stream3"),
            description="Topic to publish to for the fourth stream.",
        ),
        "camera1/binning": DeclareLaunchArgument(
            "camera3/binning",
            default_value=TextSubstitution(text="1"),
            description="Binning for the fourth stream.",
        ),
        "camera1/exposure_time_us": DeclareLaunchArgument(
            "camera3/exposure_time_us",
            default_value=TextSubstitution(text="50000"),
            description="Exposure time for the fourth stream.",
        ),
        "camera1/image_size": DeclareLaunchArgument(
            "camera3/image_size",
            default_value=TextSubstitution(text="[640,480]"),
            description="Image size for the fourth stream.",
        ),
    }

    stream0_launch_args = {
        "namespace": "acquire",
        "log_level": LaunchConfiguration("log_level"),
    }
    stream0_launch_args.update(
        {k: LaunchConfiguration(k) for k in stream0_camera_args.keys()}
    )

    stream1_launch_args = {
        "namespace": "acquire",
        "log_level": LaunchConfiguration("log_level"),
    }
    stream1_launch_args.update(
        {k: LaunchConfiguration(k) for k in stream1_camera_args.keys()}
    )

    return LaunchDescription(
        [
            log_level_arg,
        ]
        + list(stream0_camera_args.values())
        + list(stream1_camera_args.values())
        + [
            Node(
                package="napari_ros",
                namespace="acquire",
                executable="stream_viewer",
                name="stream_viewer",
                parameters=[{"streams": ["stream0", "stream1", "stream2", "stream3"]}],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("acquire"),
                                "launch",
                                "stream_video.launch.py",
                            ]
                        ),
                    ]
                ),
                launch_arguments=stream0_launch_args.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("acquire"),
                                "launch",
                                "stream_video.launch.py",
                            ]
                        ),
                    ]
                ),
                launch_arguments={
                    k.replace("camera2", "camera0").replace("camera3", "camera1"): v
                    for k, v in stream1_launch_args.items()
                }.items(),
            ),
        ]
    )
