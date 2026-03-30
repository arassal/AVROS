import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String


class DetectionsToObstacles(Node):
    def __init__(self):
        super().__init__('detections_to_obstacles')
        self.declare_parameter('detection_topic', '/seg_ros/detections')
        self.declare_parameter('obstacle_topic', '/obstacle_points')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('x_scale', 0.05)
        self.declare_parameter('y_scale', 0.02)

        self.frame_id = self.get_parameter('frame_id').value
        self.x_scale = float(self.get_parameter('x_scale').value)
        self.y_scale = float(self.get_parameter('y_scale').value)

        detection_topic = self.get_parameter('detection_topic').value
        obstacle_topic = self.get_parameter('obstacle_topic').value

        self.sub = self.create_subscription(String, detection_topic, self._cb, 10)
        self.pub = self.create_publisher(PointCloud2, obstacle_topic, 10)

    def _cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            detections = payload.get('detections', [])
        except json.JSONDecodeError:
            self.get_logger().warning('Invalid detection JSON payload')
            return

        points = []
        for det in detections:
            xyxy = det.get('xyxy', [])
            if len(xyxy) != 4:
                continue
            cx = (xyxy[0] + xyxy[2]) / 2.0
            cy = (xyxy[1] + xyxy[3]) / 2.0
            x_m = max(0.1, (720.0 - cy) * self.x_scale / 100.0)
            y_m = (cx - 640.0) * self.y_scale / 100.0
            points.append((float(x_m), float(y_m), 0.0))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.pub.publish(cloud)
        self.get_logger().info(f'Published {len(points)} obstacle points')


def main(args=None):
    rclpy.init(args=args)
    node = DetectionsToObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
