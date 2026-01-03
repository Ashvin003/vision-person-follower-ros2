import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_cb,
            10
        )

        self.pub_size = self.create_publisher(Float32, '/person_size', 10)

        self.get_logger().info("Person Detector Started")

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, verbose=False)

        detected = False
        size = 0.0

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                if self.model.names[cls] == 'person':
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    size = float(y2 - y1)
                    detected = True

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(frame, "PERSON", (x1, y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    break

        if detected:
            self.pub_size.publish(Float32(data=size))

        cv2.imshow("YOLOv8 Person Follower", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
