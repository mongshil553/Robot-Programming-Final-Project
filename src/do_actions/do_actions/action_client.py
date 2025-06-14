import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
import time

from rp_project_interfaces.action import Moving
from rp_project_interfaces.msg import Command
from rp_project_interfaces.srv import ForceCommand

class myClients(Node):
    def __init__(self):
        super().__init__('action_clients')

        self._goal_handle = None
        self.can_execute_new_action = True
        self.key = None

        ### Define words that connects to action
        self.text_to_action_id = {'hello': 1, 'spin': 2, 'kitchen': 3, 'bedroom': 4, 'bathroom': 5, 'return': 6}

        self.run_judge_hand_gesture_id_server = self.create_service(ForceCommand,
                                                                '/team/hand_gesture/force_command',
                                                                    self.force_command_run)

        #self.current_action_key = 0

        self.sub = self.create_subscription(Command, '/team/hand_gesture/command',
                                            self.execute_action, 10)
        

        self.action_hello_client = ActionClient(self, Moving, '/team/action_list/action_hello_execute')
        self.action_spin_client = ActionClient(self, Moving, '/team/action_list/action_spin_execute')
        self.action_kitchen_client = ActionClient(self, Moving, '/team/action_list/action_kitchen_execute')
        self.action_bedroom_client = ActionClient(self, Moving, '/team/action_list/action_bedroom_execute')
        self.action_bathroom_client = ActionClient(self, Moving, '/team/action_list/action_bathroom_execute')
        self.action_return_client = ActionClient(self, Moving, '/team/action_list/action_return_execute')

### --- Action HELLO ---
    def action_hello(self):
        self.action_hello_client.wait_for_server()
        self.get_logger().info("Action_hello Server Alive")
        self.can_execute_new_action = False

        goal_msg = Moving.Goal()
        goal_msg.moving_goal = 1
        self._send_goal_future = self.action_hello_client.send_goal_async(
            goal_msg,
            feedback_callback = self.feedback_callback1
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def feedback_callback1(self, feedback):
        ### Write your code here
        ###
        print(feedback.feedback.moving_feedback)

### --- Action SPIN ---
    def action_spin(self):
        self.action_spin_client.wait_for_server()
        self.get_logger().info("Action_spin Server Alive")
        self.can_execute_new_action = False

        goal_msg = Moving.Goal()
        goal_msg.moving_goal = 2
        self._send_goal_future = self.action_spin_client.send_goal_async(
            goal_msg,
            feedback_callback = self.feedback_callback3
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)
    def feedback_callback3(self, feedback):
        ### Write your code here
        ###
        print(feedback.feedback.moving_feedback)

### --- Action related to SLAM ---
    def action_kitchen(self): #LD
        self.action_kitchen_client.wait_for_server()
        self.get_logger().info("Action_move Server Alive")
        self.can_execute_new_action = False

        goal_msg = Moving.Goal()
        goal_msg.moving_goal = 3
        goal_msg.goal_pose = PoseStamped()
        goal_msg.goal_pose.header.frame_id = "map"
        goal_msg.goal_pose.pose.position.x = 0.054501437321305275
        goal_msg.goal_pose.pose.position.y = 1.1658751964569092
        goal_msg.goal_pose.pose.orientation.z = 0.9962
        goal_msg.goal_pose.pose.orientation.w = 0.0872
        
        # goal_msg.goal_pose.pose.position.x = -2.2245430946350098
        # goal_msg.goal_pose.pose.position.y = -0.16289296746253967
        # goal_msg.goal_pose.pose.orientation.z = -0.001373291015625
        self._send_goal_future = self.action_kitchen_client.send_goal_async(
            goal_msg,
            feedback_callback = self.feedback_callback2
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        self.action_return()

    def action_bedroom(self): #RU
        self.action_bedroom_client.wait_for_server()
        self.get_logger().info("Action_move Server Alive")
        self.can_execute_new_action = False

        goal_msg = Moving.Goal()
        goal_msg.moving_goal = 4
        goal_msg.goal_pose = PoseStamped()
        goal_msg.goal_pose.header.frame_id = "map"
        goal_msg.goal_pose.pose.position.x = 0.6798840165138245
        goal_msg.goal_pose.pose.position.y = -0.023582641035318375
        goal_msg.goal_pose.pose.orientation.w = -0.001434326171875
        
        # goal_msg.goal_pose.pose.position.x = -0.029908152297139168
        # goal_msg.goal_pose.pose.position.y = 1.2904294729232788
        self._send_goal_future = self.action_bedroom_client.send_goal_async(
            goal_msg,
            feedback_callback = self.feedback_callback2
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)


    def action_bathroom(self): #LU
        self.action_bathroom_client.wait_for_server()
        self.get_logger().info("Action_move Server Alive")
        self.can_execute_new_action = False

        goal_msg = Moving.Goal()
        goal_msg.moving_goal = 5
        goal_msg.goal_pose = PoseStamped()
        goal_msg.goal_pose.header.frame_id = "map"
        goal_msg.goal_pose.pose.position.x = 0.78911954164505
        goal_msg.goal_pose.pose.position.y = 1.150700569152832
        goal_msg.goal_pose.pose.orientation.w = 0.002471923828125
        
        # goal_msg.goal_pose.pose.position.x = -2.295172691345215
        # goal_msg.goal_pose.pose.position.y = 1.3771889209747314
        self._send_goal_future = self.action_bathroom_client.send_goal_async(
            goal_msg,
            feedback_callback = self.feedback_callback2
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def action_return(self): #LU
        self.action_return_client.wait_for_server()
        self.get_logger().info("Action_return Server Alive")
        self.can_execute_new_action = False

        goal_msg = Moving.Goal()
        goal_msg.moving_goal = 6
        goal_msg.goal_pose = PoseStamped()
        goal_msg.goal_pose.header.frame_id = "map"
        goal_msg.goal_pose.pose.position.x = -0.005844820756465197
        goal_msg.goal_pose.pose.position.y = -0.01751142367720604
        goal_msg.goal_pose.pose.orientation.w = 0.183349609375
        self._send_goal_future = self.action_return_client.send_goal_async(
            goal_msg,
            feedback_callback = self.feedback_callback2
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def feedback_callback2(self, feedback):
        ### Write your code here
        ###
        self.get_logger().info(f"[feedback] {feedback.feedback.moving_feedback} | 진행률: {feedback.feedback.progress_percent:.1f}%")



### Action Cancellation
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected')
            return
        
        self._goal_handle = goal_handle
        self.get_logger().info("Goal Accepted, Waiting for result")
        #self.get_logger().info('Action{0} Accepcted'.format(future))

        #self.get_logger().info('Goal accepted, waiting for result...')
        # result_future = goal_handle.get_result_async()
        # result_future.add_done_callback(self.result_callback)
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_callback)

    def cancel_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self.cancel_result = True
            self._goal_handle = None
            self.can_execute_new_action = True
        else:
            self.get_logger().info('Goal failed to cancel')
            self.cancel_result = False
            self.can_execute_new_action = False

    def _result_callback(self, future):
        self.get_logger().info(">> inside _result_callback")
        result = future.result().result
        self.get_logger().info(f'Result: {result.moving_result}')

        ### return service
        # Kitchen 액션 결과이면 자동으로 return 실행
        if "kitchen" in result.moving_result.lower():
            self.get_logger().info("자동 return 실행 중...")
            self.action_return()

        self.can_execute_new_action = True
        self._goal_handle = None

        

### Action Control
    async def f_cancel_action(self, request, response):
        print('in')
        return response
    
    def cancel_action(self):
        print(self._goal_handle)
        if self._goal_handle is not None:
            print("trying to cancel")
            #self._goal_handle.can
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_callback)
            
    
    def tmr_callback_for_executing_action(self):
        
        
        if self.can_execute_new_action:
            if self.key == 1:
                self.get_logger().info('action_hello_executing')
                self.action_hello()
            elif self.key == 2:
                self.get_logger().info('action_spin_executing')
                self.action_spin()
            elif self.key == 3:
                self.get_logger().info('action_kitchen_executing')
                self.action_kitchen()
            elif self.key == 4:
                self.get_logger().info('action_bedroom_executing')
                self.action_bedroom()
            elif self.key == 5:
                self.get_logger().info('action_bathroom_executing')
                self.action_bathroom()
            elif self.key == 6:
                self.get_logger().info('action_return_executing')
                self.action_return()
            
            self.tmr.destroy()
        else:
            if self.key == 1:
                self.cancel_action()
            elif self.key == 2:
                self.cancel_action()
            elif self.key == 3:
                self.cancel_action()
            elif self.key == 4:
                self.cancel_action()
            elif self.key == 5:
                self.cancel_action()
            elif self.key == 6:
                self.cancel_action()

    def _execute(self, key):
        self.key = key
        self.tmr = self.create_timer(0.1, self.tmr_callback_for_executing_action)

    # async def execute_action(self, msg):

    #     for foo in (lambda x: [True if substr in x else False for substr in self.text_to_action_id])(msg.cmd.lower()):
    #         if foo:
    #             self.key = self.text_to_action_id.get(msg.cmd)
    #             self._execute(self.key)
    #             break
    #     else:
    #         self.get_logger().info("{0} is not a valid command".format(msg.cmd))

        #self.key = msg.cmd
        #self.tmr = self.create_timer(0.1, self.tmr_callback_for_executing_action)
    def execute_action(self, msg):
        cmd = msg.cmd.strip().lower()
        self.get_logger().info(f"받은 명령어: '{cmd}'")

        if cmd in self.text_to_action_id:
            self.key = self.text_to_action_id[cmd]
            self.get_logger().info(f"유효한 명령 확인: '{cmd}' → 실행 번호: {self.key}")
            self._execute(self.key)
        else:
            self.get_logger().warn(f"'{cmd}'은 유효한 명령이 아님")


    def force_command_run(self, request, response):
        key = request.cmd

        if key in [1, 2, 3, 4, 5, 6]:
            self._execute(key)
        else:
            self.get_logger().info("No key {0} available".format(key))

        return response


def main(args=None):
    rp.init(args=args)

    nd = myClients()
    executor = MultiThreadedExecutor()
    executor.add_node(nd)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        nd.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()