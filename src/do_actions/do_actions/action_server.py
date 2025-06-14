import rclpy as rp
import math
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import time

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rp_project_interfaces.action import Moving
from std_srvs.srv import SetBool

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray 
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rp_project_interfaces.msg import Command

from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool


class myActions(Node):
    def __init__(self):
        super().__init__('action_servers')

        self._goal_handle = None
        self.yolo_detected = False
        self.group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub = self.create_publisher(Command, '/team/hand_gesture/command', 10)
        self.current_move_goal_handle = None
        self.last_cmd_time = None
        self.cmd_is_zero = False
        self.current_path = None
        self.current_pose = None
        self.nav_status = None
        
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback,10)
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.status_sub = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status_cb, 10)
        self.path_sub = self.create_subscription(Path, '/plan', self.path_cb, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.create_subscription(Bool, '/yolo/class0_detected', self.yolo_callback,10)
        

        self.action_hi_server = ActionServer(self, Moving,
                                           '/team/action_list/action_hello_execute',
                                           execute_callback=self.action_hello,
                                           goal_callback=self.goal_callback,
                                           cancel_callback=self.cancel_callback,
                                           callback_group=self.group)
        self.action_spin_server = ActionServer(self, Moving, '/team/action_list/action_spin_execute',
                                               execute_callback=self.action_spin,
                                               goal_callback=self.goal_callback,
                                               cancel_callback=self.cancel_callback,
                                               callback_group=self.group)
        self.action_kitchen_server = ActionServer(self, Moving, '/team/action_list/action_kitchen_execute',
                                               execute_callback=self.action_kitchen,
                                               goal_callback=self.goal_callback,
                                               cancel_callback=self.cancel_callback,
                                               callback_group=self.group)
        self.action_bedroom_server = ActionServer(self, Moving, '/team/action_list/action_bedroom_execute',
                                               execute_callback=self.action_bedroom,
                                               goal_callback=self.goal_callback,
                                               cancel_callback=self.cancel_callback,
                                               callback_group=self.group)
        self.action_bathroom_server = ActionServer(self, Moving, '/team/action_list/action_bathroom_execute',
                                               execute_callback=self.action_bathroom,
                                               goal_callback=self.goal_callback,
                                               cancel_callback=self.cancel_callback,
                                               callback_group=self.group)
        self.action_return_server = ActionServer(self,Moving,'/team/action_list/action_return_execute',
                                                execute_callback=self.action_return,
                                                goal_callback=self.goal_callback,
                                                cancel_callback=self.cancel_callback,
                                                callback_group=self.group)

        #self.create_client(Se)
        self.servo_client = self.create_client(SetBool, '/team/servo/set', callback_group=self.group)

        '''request = SetBool.Request()
        request.data = True
        res = self.servo_client.call(request)
        print(res.message)''' #예시 코드임 servo_client를 통해 서비스 요청 True: Grab, False: Release

    def yolo_callback(self, msg):
        self.yolo_detected = msg.data

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def _cancel_action(self, goal_handle):
         if self.cmd_sub:
            self.destroy_subscription(self.cmd_sub)
            self.cmd_sub = None
            self.current_move_goal_handle = None

            goal_handle.canceled()
            return Moving.Result(moving_result="Action canceled")
        
    def _nav2_feedback_callback(self, feedback_msg, goal_handle):
        try:
            distance = feedback_msg.feedback.distance_remaining
            feedback = Moving.Feedback()

            # 진행률 계산
            feedback.progress_percent = max(0.0, min(100.0, 100.0 - (distance * 100)))

           
            feedback.moving_feedback = f"목표까지 {distance:.2f}m 남음"

            goal_handle.publish_feedback(feedback)
        except Exception as e:
            self.get_logger().warn(f"[feedback error] {str(e)}")

    def reset_parameter(self):
        if self.cmd_sub is not None:
            self.destroy_subscription(self.cmd_sub)
            self.cmd_sub = None

        self.current_move_goal_handle = None
        self.last_cmd_time = None
        self.cmd_is_zero = False
        self.current_path = None
        self.current_pose = None
      
    def status_cb(self, msg):
        if msg.status_list:
            self.nav_status = msg.status_list[-1].status

    def path_cb(self, msg):
        self.current_path = msg.poses

    def pose_cb(self, msg):
        self.current_pose = msg.pose.pose

    def compute_path_length(self, path):
        total = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1].pose.position.x - path[i].pose.position.x
            dy = path[i+1].pose.position.y - path[i].pose.position.y
            total += math.hypot(dx, dy)
        return total

    def compute_remaining_distance(self, current_pose, path):
        if not path or not current_pose:
            return float('inf')
        goal = path[-1].pose.position
        curr = current_pose.position
        return math.hypot(goal.x - curr.x, goal.y - curr.y)
    
    def cmd_vel_callback(self, msg):
        if self.current_move_goal_handle is None or self.current_move_goal_handle.is_cancel_requested:
            return

        self.cmd_pub.publish(msg)

        
        if abs(msg.linear.x) < 0.01:
            if self.last_cmd_time is None:
                self.last_cmd_time = time.time()
            else:
                if time.time() - self.last_cmd_time > 3.0:  
                    self.cmd_is_zero = True
        else:
            self.last_cmd_time = None
            self.cmd_is_zero = False
    async def action_return(self, goal_handle):
        self.get_logger().info("Action Return Executing...")

        target_pose = goal_handle.request.goal_pose

        self.nav2_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        send_goal_future = self.nav2_client.send_goal_async(goal_msg, feedback_callback=lambda fb: self._nav2_feedback_callback(fb, goal_handle))
        nav2_goal_handle = await send_goal_future

        if not nav2_goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            goal_handle.abort()
            return Moving.Result(moving_result="Nav2 goal rejected")

        self.get_logger().info("Nav2 Goal Accepted.")

        result_future = nav2_goal_handle.get_result_async()
        result = await result_future

        if result.status == 4:  # SUCCEEDED
            goal_handle.succeed()
            return Moving.Result(moving_result="Arrive at the Origin")
        else:
            goal_handle.abort()
            return Moving.Result(moving_result=f"error: {result.status}")
            
    async def action_hello(self, goal_handle):
        self.get_logger().info("Action Hello Executing...")
        feedback = Moving.Feedback()
        twist = Twist()
        
        for i in range(3):
            if goal_handle.is_cancel_requested:
                return self._cancel_action(goal_handle)

            # LEFT
            twist.angular.z = 1.5
            self.cmd_pub.publish(twist)
            feedback.moving_feedback = f"[{i+1}/3] 왼쪽 회전 중..."
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            time.sleep(1)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Moving.Result(moving_result=" Hi Cancelled")

            # RIGHT
            twist.angular.z = -1.5
            self.cmd_pub.publish(twist)
            feedback.moving_feedback = f"[{i+1}/3] 오른쪽 회전 중..."
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            time.sleep(1)
            
        goal_handle.succeed()
        self.reset_parameter()
        
        return Moving.Result(moving_result="Action Hi done")
        

    async def action_spin(self, goal_handle):
        self.get_logger().info("Action Spin Executing...")
        feedback = Moving.Feedback()
        twist = Twist()
        
        twist.angular.z = -2.0
        self.cmd_pub.publish(twist)
        feedback.moving_feedback = "회전 중..."
        goal_handle.publish_feedback(feedback)
        time.sleep(3)

        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.5)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Moving.Result(moving_result="Spin Cancelled")

    
        goal_handle.succeed()
        self.reset_parameter()
        return Moving.Result(moving_result="Action Spin done")

    async def action_kitchen(self, goal_handle):
        self.get_logger().info("Action kitchen Executing...")

        target_pose = goal_handle.request.goal_pose

        self.nav2_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        send_goal_future = self.nav2_client.send_goal_async(
            goal_msg, feedback_callback=lambda fb: self._nav2_feedback_callback(fb, goal_handle))
        nav2_goal_handle = await send_goal_future

        if not nav2_goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            goal_handle.abort()
            return Moving.Result(moving_result="Nav2 goal rejected")

        self.get_logger().info("Nav2 Goal Accepted.")
        result_future = nav2_goal_handle.get_result_async()
        result = await result_future

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("Nav2 goal 도달 완료, YOLO 감지 대기 중...")

            timeout = time.time() + 6.0
            while time.time() < timeout:
                if self.yolo_detected:
                    self.get_logger().info("Object 감지됨 → 이동 시작")
                    break
                rp.spin_once(self, timeout_sec=0.1)

            if self.yolo_detected:
                twist = Twist()
                twist.linear.x = 0.1
                self.cmd_pub.publish(twist)
                time.sleep(1.8)
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                self.get_logger().info("2초간 전진 후 정지")

                #서보 모터 서비스 요청
                request = SetBool.Request()
                request.data = True
                self.get_logger().info("[서보] 요청 전송 중...")
                future = self.servo_client.call_async(request)
                res = await future

                if res.success:
                    self.get_logger().info(f"[서보] 성공: {res.message}")
                else:
                    self.get_logger().warn(f"[서보] 실패: {res.message}")
            else:
                self.get_logger().warn("YOLO 감지 실패 → 추가 동작 없음")

            # 복귀용 Nav2 goal 설정
            return_pose = PoseStamped()
            return_pose.header.frame_id = "map"
            return_pose.header.stamp = self.get_clock().now().to_msg()
            return_pose.pose.position.x = -0.005844820756465197
            return_pose.pose.position.y = -0.01751142367720604
            return_pose.pose.orientation.w = 0.183349609375

            return_goal = NavigateToPose.Goal()
            return_goal.pose = return_pose

            self.get_logger().info("복귀 경로 전송 중...")
            send_back_future = self.nav2_client.send_goal_async(return_goal)
            return_handle = await send_back_future

            if not return_handle.accepted:
                self.get_logger().error("복귀 경로 거부됨")
                goal_handle.abort()
                return Moving.Result(moving_result="서보 완료, 복귀 실패")

            back_result_future = return_handle.get_result_async()
            back_result = await back_result_future

            if back_result.status == 4:
                self.get_logger().info("복귀 완료")
                goal_handle.succeed()
                return Moving.Result(moving_result="kitchen → 서보 → 복귀 완료")
            else:
                goal_handle.abort()
                return Moving.Result(moving_result=f"복귀 중 실패: {back_result.status}")

        else:
            goal_handle.abort()
            return Moving.Result(moving_result=f"Nav2 이동 실패: {result.status}")


    async def action_bedroom(self, goal_handle):
        self.get_logger().info("Action bedroom Executing...")

        target_pose = goal_handle.request.goal_pose

        self.nav2_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        send_goal_future = self.nav2_client.send_goal_async(
            goal_msg, feedback_callback=lambda fb: self._nav2_feedback_callback(fb, goal_handle))
        nav2_goal_handle = await send_goal_future

        if not nav2_goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            goal_handle.abort()
            return Moving.Result(moving_result="Nav2 goal rejected")

        self.get_logger().info("Nav2 Goal Accepted.")

        result_future = nav2_goal_handle.get_result_async()
        result = await result_future

        if result.status == 4:  # SUCCEEDED
            goal_handle.succeed()
            return Moving.Result(moving_result="Arrive at the bedroom")
        else:
            goal_handle.abort()
            return Moving.Result(moving_result=f"error: {result.status}")

    async def action_bathroom(self, goal_handle):
        self.get_logger().info("Action bathroom Executing...")

        target_pose = goal_handle.request.goal_pose

        self.nav2_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        send_goal_future = self.nav2_client.send_goal_async(goal_msg, feedback_callback=lambda fb: self._nav2_feedback_callback(fb, goal_handle))
        nav2_goal_handle = await send_goal_future

        if not nav2_goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            goal_handle.abort()
            return Moving.Result(moving_result="Nav2 goal rejected")

        self.get_logger().info("Nav2 Goal Accepted.")

        result_future = nav2_goal_handle.get_result_async()
        result = await result_future

        if result.status == 4:  # SUCCEEDED
            goal_handle.succeed()
            return Moving.Result(moving_result="Arrived at the bathroom")
        else:
            goal_handle.abort()
            return Moving.Result(moving_result=f"Nav2 실패: {result.status}")


def main(args=None):
    rp.init(args=args)

    nd = myActions()

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