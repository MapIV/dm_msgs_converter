import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from autoware_perception_msgs.msg import DynamicObjectWithFeature
from autoware_perception_msgs.msg import DynamicObjectWithFeatureArray
from unique_identifier_msgs.msg import UUID
from autoware_perception_msgs.msg import DynamicObject
from autoware_perception_msgs.msg import DynamicObjectArray
from dm_object_info_msgs.msg import ObjectInfo


class AwDmConverter(Node):

    def __init__(self):
        super().__init__('aw_dm_converter')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        self.subscription = self.create_subscription(
            DynamicObjectWithFeatureArray,
            '/labeled_clusters',
            self.aw_perception_callback,
            10)

    def aw_perception_callback(self, aw_msg):
        if len(aw_msg.feature_objects) > 0:
            self.get_logger().info("Message Received")
            msg = String()
            msg.data = f"{aw_msg.feature_objects[0].object.id}"
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    aw_dm_conv = AwDmConverter()

    rclpy.spin(aw_dm_conv)

    aw_dm_conv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
