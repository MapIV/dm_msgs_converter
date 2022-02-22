import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from autoware_perception_msgs.msg import DynamicObjectWithFeature
from autoware_perception_msgs.msg import DynamicObjectWithFeatureArray
from unique_identifier_msgs.msg import UUID
from autoware_perception_msgs.msg import DynamicObject
from autoware_perception_msgs.msg import Semantic
from autoware_perception_msgs.msg import DynamicObjectArray
from dm_object_info_msgs.msg import ObjectInfo
from dm_object_info_msgs.msg import ObjectInfoArray
from dm_object_info_msgs.msg import ObjectClass
from dm_object_info_msgs.msg import ClassId


class AwDmConverter(Node):

    def __init__(self):
        super().__init__('aw_dm_converter')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        self.subscription = self.create_subscription(
            DynamicObjectWithFeatureArray,
            '/labeled_clusters',
            self.aw_perception_callback,
            10)

    def aw_class_to_dm(self, dynamic_object: DynamicObject):
        dm_object_class = ObjectClass()
        if dynamic_object.semantic.type == Semantic.CAR \
                or dynamic_object.semantic.type == Semantic.TRUCK \
                or dynamic_object.semantic.type == Semantic.BUS \
                or dynamic_object.semantic.type == Semantic.BICYCLE \
                or dynamic_object.semantic.type == Semantic.MOTORBIKE:
            dm_object_class.id.value = ClassId.VEHICLE
        if dynamic_object.semantic.type == Semantic.PEDESTRIAN:
            dm_object_class.id.value = ClassId.PERSON
        if dynamic_object.semantic.type == Semantic.ANIMAL:
            dm_object_class.id.value = ClassId.ANIMAL
        if dynamic_object.semantic.type == Semantic.UNKNOWN:
            dm_object_class.id.value = ClassId.OTHER
        dm_object_class.confidence.value = dynamic_object.semantic.confidence*100
        if dm_object_class.confidence.value > 100:
            dm_object_class.confidence.value = 100
        return dm_object_class, dm_object_class.confidence.value

    def aw_perception_callback(self, aw_msg: DynamicObjectWithFeatureArray):
        for dynamic_object in aw_msg.feature_objects:
            dm_object = ObjectInfo()
            # AW uses uuid (big endian), but dm uses uint64, use only the 64 most significant bits
            dm_object.id = int.from_bytes(dynamic_object.id.uuid[:8], 'big')
            object_class, confidence = self.aw_class_to_dm(dynamic_object)
            dm_object.object_class.append(object_class)
            dm_object.existency = confidence


def main(args=None):
    rclpy.init(args=args)

    aw_dm_conv = AwDmConverter()

    rclpy.spin(aw_dm_conv)

    aw_dm_conv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
