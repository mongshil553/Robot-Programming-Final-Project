import rclpy as rp
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_srvs.srv import SetBool
from std_msgs.msg import Bool 

class MyNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        # 서비스 설정
        self.subscription = self.create_subscription(
            Bool,
            '/team/servo/cmd',  # 변경된 토픽 이름
            self.servo_callback,
            10)

        # GPIO 설정
        self.servo_pin = 12  # 서보 모터가 연결된 GPIO 핀 번호
        GPIO.setmode(GPIO.BCM)  # BCM 모드 사용
        GPIO.setup(self.servo_pin, GPIO.OUT)  # GPIO 12 핀을 출력으로 설정

        # PWM 설정 (주파수는 서보 모터의 사양에 맞추어 50Hz)
        self.pwm = GPIO.PWM(self.servo_pin, 50)  
        self.pwm.start(0)  # 초기 듀티 사이클은 0%

        # PWM 값 (서보 모터를 잡기 위한 PWM 값)
        self.grab, self.release = 10, 5  # 서보의 최소 및 최대 듀티 사이클 (예시)
        
    def servo_callback(self, msg):
        self.get_logger().info("서보 토픽 수신함")
        if msg.data:
            self.set_servo(self.grab)  # 잡기
        else:
            self.set_servo(self.release)  # 놓기

    def set_servo(self, pwm_val):
        # PWM 듀티 사이클을 설정하여 서보를 제어
        self.pwm.ChangeDutyCycle(pwm_val)
        self.get_logger().info(f'Setting servo to {pwm_val}% duty cycle')

    def __del__(self):
        # 객체가 삭제될 때 GPIO 클린업
        self.pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rp.init(args=args)
    node = MyNode()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()

