#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')
        
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Define the static transform
        static_transform = TransformStamped()
        
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'livox_frame'
        static_transform.child_frame_id = 'flipped_livox_frame'
        
        # Set translation (no translation in this case, only rotation)
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        
        # Set rotation (180 degrees around the X-axis to flip the Z-axis)
        quat = tf_transformations.quaternion_from_euler(3.14159, 0, 0)  # 180 degrees around X-axis
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]
        
        # Broadcast the static transform
        self.static_broadcaster.sendTransform(static_transform)

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()