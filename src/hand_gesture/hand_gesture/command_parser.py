import rclpy as rp
from rclpy.node import Node

from . import text_cleaner as tc

from rp_project_interfaces.msg import Command
from rp_project_interfaces.msg import HandGestureChar
from rp_project_interfaces.srv import ForceString
import time

class cmd_parser(Node):
    def __init__(self, parent_dir):
        super().__init__('command_node')
        self.parent_dir = parent_dir

        self.sub = self.create_subscription(HandGestureChar, '/team/hand_gesture/recognized_char',
                                            self.collect, 10)
        self.pub = self.create_publisher(Command, '/team/hand_gesture/command', 10)

        self.run_string_server = self.create_service(ForceString,
                                                     '/team/hand_gesture/force_string',
                                                     self.force_string_run)
        
        self.protocol_begin_char = 'x'; self.protocol_end_char = 'w'
        self.cmd = ''
        self.corrected_cmd = ''

        self.worker = tc.myTextCleanerWithOpenAI(txtfile ='/home/rail/project/DO_NOT_SHARE_OR_SHOW.txt')
        
    def collect(self, msg):
        entry = chr(msg.hand_gesture_char)

        if entry == self.protocol_begin_char:
            self.cmd = '' + self.protocol_begin_char
        elif len(self.cmd) >= 1:
            if entry == self.protocol_end_char:
                self.cmd += self.protocol_end_char
                self.call_bing_api(self.cmd[1:-1])
                self.publish_command()
                self.cmd = ''
            else:
                self.cmd += entry

        self.get_logger().info('Got: {0}, Total: {1}'.format(entry, self.cmd))

    def call_bing_api(self, cmd):
        key = cmd
        is_error, original, corrected = self.worker.run(key) #run gpt to get corrected text
        #call bing api to correct key <- ex) hpllu -> hello
        self.corrected_cmd = corrected
        self.get_logger().info("Corrected text: {0}".format(self.corrected_cmd))
                
    def publish_command(self):
        #key = self.cmd[1:-1]
        self.get_logger().info("Command: {0}".format(self.corrected_cmd))

        #print((lambda x: [True if substr in x  else False for substr in self.text_to_action_id])(self.corrected_cmd))

        msg = Command()
        msg.cmd = self.corrected_cmd
        self.pub.publish(msg)
        self.get_logger().info("Published action_cmd: {0}".format(msg.cmd))
    
    def force_string_run(self, request, response):
        key = request.mystr

        self.get_logger().info("Got str: {0}".format(key))

        self.call_bing_api(key)
        self.publish_command()

        response.res = self.corrected_cmd

        return response


        

def main(args=None):
    rp.init(args=args)

    nd = cmd_parser('/home/kijung914/robot_programming')

    rp.spin(nd)

    nd.destroy_node()

    rp.shutdown()

if __name__ == '__main__':
    main()
