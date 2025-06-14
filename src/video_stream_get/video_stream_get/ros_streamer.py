from  . import streaming

import rclpy as rp
from rclpy.node import Node

class mystreamer(Node):
    def __init__(self):
        super().__init__('streamer')

        streaming.begin()

def main(args=None):
    rp.init(args=args)

    node = mystreamer()

    rp.spin(node)

    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()