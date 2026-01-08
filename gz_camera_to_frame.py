import cv2
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

def analyze_frame(frame):
    if frame is None: return None, 0, "No Data"
    
    # كشف الباركود - نسخة متوافقة مع جميع الإصدارات
    detector = cv2.barcode_BarcodeDetector()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = detector.detectAndDecode(gray)
    
    # حل مشكلة الـ Unpack تلقائياً
    retval = res[0]
    points = res[2] if len(res) > 2 else None
    is_detected = retval if isinstance(retval, bool) else any(retval)
    
    # كشف الرصيف الأصفر (Bonus)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    is_yellow = (np.sum(mask > 0) / mask.size) > 0.01

    status = "AUTHORIZED"
    if not is_detected: status = "NO BARCODE"
    if is_yellow: status = "ILLEGAL (YELLOW CURB)"
    
    return frame, (100 if (is_detected and not is_yellow) else 0), status

class AsyncCameraInterface:
    def __init__(self, topic):
        self.topic, self._frame, self._lock = topic, None, threading.Lock()
    def start(self):
        def _spin():
            if not rclpy.ok(): rclpy.init()
            n = Node('cam_node')
            n.create_subscription(Image, self.topic, self._cb, 10)
            rclpy.spin(n)
        threading.Thread(target=_spin, daemon=True).start()
    def _cb(self, msg):
        with self._lock: self._frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    async def get_frame(self):
        import asyncio
        while self._frame is None: await asyncio.sleep(0.1)
        with self._lock: return self._frame.copy()