import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image

from threading import Thread
from typing import Optional
import logging
from multiprocessing import Process, Queue

import napari
import numpy as np

queue = Queue()
viewer: Optional[napari.Viewer] = None


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

    def update_layer(self, data, stream_id: int):
        layer_key = f"Stream {stream_id}"
        assert isinstance(data, np.ndarray)
        queue.put((data, layer_key))

    def stream0(self, msg: Image):
        self.get_logger().info("Got new frame on stream 0")
        im = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=np.uint8,
            buffer=msg.data,
        )
        self.update_layer(im, 0)

    def stream1(self, msg: Image):
        self.get_logger().info("Got new frame on stream 1")
        im = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=np.uint8,
            buffer=msg.data,
        )
        self.update_layer(im, 1)


def add_image(args):
    global viewer
    (data, layer_key) = args
    try:
        layer = viewer.layers[layer_key]
        layer._slice.image._view = data
        layer.events.set_data()
    except KeyError:
        viewer.add_image([data], name=layer_key)


@napari.qt.thread_worker(connect={"yielded": add_image})
def process_queue(q: Queue):
    logging.info("starting thread worker")
    while True:
        logging.info("waiting")
        v = q.get()
        logging.info("Got data for ", v[1])
        yield v


def start_viewer(q):
    global viewer
    viewer = napari.Viewer()
    worker = process_queue(q)
    # worker.start()
    napari.run()


def main(args=None):
    Process(target=start_viewer, args=(queue,), daemon=True).start()

    rclpy.init(args=args)

    napari_stream_viewer = NapariStreamViewer()
    rclpy.spin(napari_stream_viewer)

    napari_stream_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
