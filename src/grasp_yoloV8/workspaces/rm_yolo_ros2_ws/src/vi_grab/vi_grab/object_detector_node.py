import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO
from vi_msgs.msg import ObjectInfo


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detect')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('show_image', True)

        model_path = self.get_parameter('model_path').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.show_image = bool(self.get_parameter('show_image').value)

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.pub = self.create_publisher(ObjectInfo, '/object_pose', 10)

        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        self.info_sub = self.create_subscription(
            CameraInfo,
            info_topic,
            self.camera_info_callback,
            10
        )

        self.color_sub = Subscriber(self, Image, color_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.08
        )
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info('YOLOv8 object detector started.')

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_to_meter(self, depth_img, u, v):
        h, w = depth_img.shape[:2]
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        x1, x2 = max(0, u - 2), min(w, u + 3)
        y1, y2 = max(0, v - 2), min(h, v + 3)
        patch = depth_img[y1:y2, x1:x2].astype(np.float32)
        valid = patch[patch > 0]

        if valid.size == 0:
            return None

        z = float(np.median(valid))

        if depth_img.dtype == np.uint16:
            z *= 0.001

        if z <= 0.0 or np.isnan(z):
            return None

        return z

    def deproject(self, u, v, z):
        x = (u - self.cx) / self.fx * z
        y = (v - self.cy) / self.fy * z
        return x, y, z

    def image_callback(self, color_msg: Image, depth_msg: Image):
        if self.fx is None or self.fy is None:
            return

        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        result = self.model.predict(color, conf=self.confidence, verbose=False)[0]
        canvas = result.plot()

        if result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            cls_ids = result.boxes.cls.cpu().numpy().astype(int)

            for box, score, cls_id in zip(boxes, confs, cls_ids):
                x1, y1, x2, y2 = box.astype(int)
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                z = self.depth_to_meter(depth, u, v)
                if z is None:
                    continue

                x, y, z = self.deproject(u, v, z)
                name = result.names[int(cls_id)]

                msg = ObjectInfo()
                msg.header = color_msg.header
                if not msg.header.frame_id:
                    msg.header.frame_id = 'camera_color_optical_frame'
                msg.object_class = str(name)
                msg.x = float(x)
                msg.y = float(y)
                msg.z = float(z)
                msg.score = float(score)
                msg.center_x = int(u)
                msg.center_y = int(v)
                self.pub.publish(msg)

                cv2.circle(canvas, (u, v), 4, (255, 255, 255), -1)
                cv2.putText(
                    canvas,
                    f'{name} ({x:.3f},{y:.3f},{z:.3f})',
                    (u + 10, v + 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

        if self.show_image:
            cv2.imshow('detection', canvas)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
