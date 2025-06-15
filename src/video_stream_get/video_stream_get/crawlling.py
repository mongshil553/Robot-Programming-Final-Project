import requests
import cv2
import numpy as np

class crawller():
    def __init__(self, ip):
        self.url = 'http://' + ip + '/video_feed'

    def run(self):
        response = requests.get(self.url, stream=True)

        # Check if response is OK
        if response.status_code == 200:
            bytes_data = bytes()
            for chunk in response.iter_content(chunk_size=1024):
                bytes_data += chunk
                a = bytes_data.find(b'\xff\xd8')  # JPEG start
                b = bytes_data.find(b'\xff\xd9')  # JPEG end
                if a != -1 and b != -1:
                    jpg_data = bytes_data[a:b+2]
                    image = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if image is not None:
                        cv2.imwrite('/home/rail/project/frame.jpg', image)
                        print("Saved '/home/rail/project/frame.jpg'")
                    else:
                        print("Failed to decode image")
                    break
        else:
            print(f"Failed to connect. Status code: {response.status_code}")

# Connect to the MJPEG stream

if __name__ == '__main__':
    c = crawller('192.168.137.145:8000')
    c.run()

