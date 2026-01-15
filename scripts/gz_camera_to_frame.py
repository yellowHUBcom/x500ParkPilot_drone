import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import asyncio
from collections import deque

class AsyncCameraInterface(Node):
    def __init__(self, topic):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.frame_queue = deque(maxlen=1) 
        
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.listener_callback,
            10) 
        
        print(f"[CAMERA] Subscribed to {topic}")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_queue.append(cv_image)
        except Exception as e:
            print(f"[CAMERA ERROR] {e}")

    async def get_frame(self):
        if len(self.frame_queue) > 0:
            return self.frame_queue[0]
        return None

    def start(self):
        import threading
        self.thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.thread.start()

def analyze_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    detector = cv2.QRCodeDetector()
    data, points, _ = detector.detectAndDecode(gray)
    
    status = "Searching..."
    score = 0
    barcode_data = None
    
    if data:
        barcode_data = data
        score = 100
        status = "AUTHORIZED"
        if points is not None:
            points = points[0].astype(int)
            for i in range(len(points)):
                cv2.line(frame, tuple(points[i]), tuple(points[(i+1)%len(points)]), (0, 255, 0), 3)
            cv2.putText(frame, "VALID PASS", (points[0][0], points[0][1]-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    
    return frame, barcode_data, status
