from  . import crawlling

import rclpy as rp
from rclpy.node import Node

class mycollector(Node):
    def __init__(self):
        super().__init__('collector')

        #self.c = crawlling.crawller('192.168.2.165:8000')
        self.c = crawlling.crawller('192.168.137.145:8000')

        self.crawl_timer_callback = self.create_timer(2.0, self.do)

    def do(self):
        self.c.run()


def main(args=None):
    rp.init(args=args)

    node = mycollector()

    rp.spin(node)

    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()