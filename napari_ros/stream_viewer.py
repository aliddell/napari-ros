import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from napari.qt.threading import thread_worker  # type: ignore


class NapariStreamViewer(Node):
    def __init__(self):
        super().__init__("napari_stream_viewer")
        self._subscriptions = (
            self.create_subscription(
                Image, "stream0", self.stream0, qos_profile_sensor_data
            ),
            self.create_subscription(
                Image, "stream1", self.stream1, qos_profile_sensor_data
            ),
        )

    def topic_callback(self, stream: int, msg: Image):
        self.get_logger().info(f"Received image from stream {stream}.")

    def stream0(self, msg: Image):
        self.topic_callback(0, msg)

    def stream1(self, msg: Image):
        self.topic_callback(1, msg)


def main(args=None):
    rclpy.init(args=args)
    napari_stream_viewer = NapariStreamViewer()
    rclpy.spin(napari_stream_viewer)

    napari_stream_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
