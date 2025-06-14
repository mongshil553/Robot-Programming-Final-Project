import rclpy as rp
from rclpy.node import Node

from rp_project_interfaces.msg import HandGestureChar
import time

import cv2
import mediapipe as mp
import numpy as np
import time
import os

gesture = {
    0:'a',1:'b',2:'c',3:'d',4:'e',5:'f',6:'g',7:'h',
    8:'i',9:'j',10:'k',11:'l',12:'m',13:'n',14:'o',
    15:'p',16:'q',17:'r',18:'s',19:'t',20:'u',21:'v',
    22:'w',23:'x',24:'y',25:'z',26:'0',27:'1',28:'2'
}

class Hand_Recognition(Node):
    def __init__(self, parent_dir):
        super().__init__('hand_recognition_char_publish')

        self.parent_dir = parent_dir

        self.pub = self.create_publisher(HandGestureChar, '/team/hand_gesture/recognized_char', 10)

        # MediaPipe 초기화
        self.hands = mp.solutions.hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils

        # KNN 모델 로드
        dataset_path = os.path.expanduser(self.parent_dir + '/project/src/hand_gesture/hand_gesture/datasetset.txt')
        file = np.genfromtxt(dataset_path, delimiter=',')
        angle = file[:, :-1].astype(np.float32)
        label = file[:, -1].astype(np.float32)
        self.knn = cv2.ml.KNearest_create()
        self.knn.train(angle, cv2.ml.ROW_SAMPLE, label)

        self.frame_path = os.path.expanduser(self.parent_dir + '/project/frame.jpg')
        self.prev_index = -1
        self.startTime = time.time()

        self.result_img_save_path = r"self.parent_dir + '/project/frame.jpg"

        self.sampling_interval = 3 # 3초 간격으로 수행

        self.run()

    def run(self):
        msg = HandGestureChar()
        while rp.ok():
            if not os.path.exists(self.frame_path):
                self.get_logger().warn("frame.jpg not found.")
                time.sleep(2)
                continue

            img = cv2.imread(self.frame_path)
            
            if img is None:
                self.get_logger().warn("Cannot read frame.jpg.")
                time.sleep(2)
                continue

            

            imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            result = self.hands.process(imgRGB)

            if result.multi_hand_landmarks is not None:
                for res in result.multi_hand_landmarks:
                    joint = np.zeros((21, 3))
                    for j, lm in enumerate(res.landmark):
                        joint[j] = [lm.x, lm.y, lm.z]

                    v1 = joint[[0,1,2,3,0,5,6,7,0,9,10,11,0,13,14,15,0,17,18,19], :]
                    v2 = joint[[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20], :]
                    v = v2 - v1
                    v = v / np.linalg.norm(v, axis=1)[:, np.newaxis]

                    compareV1 = v[[0,1,2,4,5,6,7,8,9,10,12,13,14,16,17], :]
                    compareV2 = v[[1,2,3,5,6,7,9,10,11,13,14,15,17,18,19], :]

                    angle = np.arccos(np.einsum('nt,nt->n', compareV1, compareV2))
                    angle = np.degrees(angle)

                    if angle.shape[0] == 15:
                        data = np.array([angle], dtype=np.float32)
                        ret, results, neighbors, dist = self.knn.findNearest(data, 3)
                        index = int(results[0][0])

                        if index in gesture:
                            char = gesture[index]
                            self.get_logger().info(f"Recognized: {char}")
                            msg.hand_gesture_char = ord(char[0])  # 첫 글자의 아스키 코드로 변환
                            self.pub.publish(msg)

                    cv2.putText(img, gesture[index].upper(),
                                (int(res.landmark[0].x * img.shape[1]) - 10,
                                 int(res.landmark[0].y * img.shape[0]) + 40),
                                 cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    self.mp_drawing.draw_landmarks(img, res, mp.solutions.hands.HAND_CONNECTIONS)
                    #cv2.imshow('rgb', imgRGB)
                    
                    cv2.imwrite(self.result_img_save_path, img)
                    #self.get_logger().info("IMAGE SAVED")
            else:
                self.get_logger().info("No hand detected.")

            time.sleep(self.sampling_interval) # 샘플링 시간마다 수행
        
        

def main(args=None):
    rp.init(args=args)

    nd = Hand_Recognition('~')

    rp.spin(nd)

    nd.destroy_node()

    rp.shutdown()

if __name__ == '__main__':
    main()

