import rclpy
from rclpy.node import Node

import napari
from skimage.data import cells3d


class NapariViewer(Node):
    def __init__(self):
        super().__init__("napari_viewer")

        self.viewer = napari.Viewer()
        napari.run()  # TODO (aliddell): if we close the viewer window, this will hang


def main(args=None):
    rclpy.init(args=args)
    napari_viewer = NapariViewer()
    rclpy.spin(napari_viewer)

    napari_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
