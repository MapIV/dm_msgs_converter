# Perception Engine Inc. 2022

import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Header

import dm_object_info_msgs.msg
from dm_object_info_msgs.msg import ObjectId
from dm_object_info_msgs.msg import ObjectColor
from dm_object_info_msgs.msg import ObjectInfo
from dm_object_info_msgs.msg import ObjectInfoArray
from dm_object_info_msgs.msg import SteeringAngle
from dm_object_info_msgs.msg import BrakeState
from dm_object_info_msgs.msg import AuxiliaryBrakeState
from dm_object_info_msgs.msg import ThrottlePosition
from dm_object_info_msgs.msg import ExteriorLights
from dm_object_info_msgs.msg import VehicleRole
from dm_object_info_msgs.msg import VehicleExtendedInformation
from dm_object_info_msgs.msg import ExistenceConfidence

from autoware_perception_msgs.msg import DynamicObjectWithFeatureArray
from autoware_perception_msgs.msg import DynamicObjectWithFeature

from .dm_conversions import *

VENDOR_ID = [0x77, 0x77, 0x77, 0x77]


class AwDmConverter(Node):

    def __init__(self):
        super().__init__('aw_dm_converter')
        self._plane_number = self.declare_parameter('plane_number', 7).value
        self.dm_publisher_ = self.create_publisher(ObjectInfoArray, 'output', 10)
        # PARAMS:
        # - reference point (x, y, z)
        self.subscription = self.create_subscription(
            DynamicObjectWithFeatureArray,
            'input',
            self.aw_perception_callback,
            10)
        self.get_logger().info('Plane number: %d' % self._plane_number)
        self.counter = 0

    def aw_perception_callback(self, aw_msg: DynamicObjectWithFeatureArray):
        if self.counter < 5:
            self.counter += 1
            return
        from struct import unpack
        dm_object_info_array = ObjectInfoArray()
        for aw_dynamic_object in aw_msg.feature_objects:
            dm_object = ObjectInfo()
            try:
                # 物標 ID REQUIRED
                # AW uses uuid (big endian), but dm uses uint64, use only the 64 most significant bits
                #tmp_id = int(unpack('>Q', bytearray(reserved + aw_dynamic_object.object.id.uuid[:4].tolist() ))[0])

                dm_object.id.value = int.from_bytes([0x80]+aw_dynamic_object.object.id.uuid[:3].tolist()+VENDOR_ID, signed=False, byteorder='big')
                #self.get_logger().info("id: {:016x}".format(dm_object.id.value))
                # 情報取得時刻［必須］Timestamp REQUIRED
                # DE_TimestampIts: Number of milliseconds since 2004-01-01T00:00:00.000Z
                dm_object.time = ros_stamp_to_de_time(aw_msg.header.stamp)
                # 物標種別 OPTIONAL
                object_class, confidence = aw_class_to_dm_object_class(aw_dynamic_object.object)
                dm_object.object_class.append(object_class)
                # 存在信頼度 OPTIONAL
                existency = ExistenceConfidence()
                existency.value = confidence
                dm_object.existency = existency
                # 物標位置
                dm_object.object_location = aw_position_to_dm_location(dynamic_object=aw_dynamic_object.object,
                                                                       plane_number=self._plane_number,
                                                                       logger=self.get_logger()
                                                                       )
                # dm_object.ref_point.value = ??

                # 物標参照位置 dm_object.ref_point.value = UNKNOWN
                # 移動方向 Heading WGS84Angle
                dm_object.direction = aw_pose_to_dm_direction(aw_dynamic_object.object,
                                                              logger=self.get_logger()
                                                              )
                dm_object.orientation = dm_object.direction
                # 速さ Speed
                dm_object.speed = aw_twist_to_dm_speed(aw_dynamic_object.object)
                # 回転速度 YawRate
                dm_object.yaw_rate = aw_twist_to_dm_yaw_rate(aw_dynamic_object.object)
                # 前後加速度 Acceleration dm_object.acceleration UNKNOWN
                # 物標の向き WGS84Angle dm_object.orientation UNKNOWN
                # 物標のサイズ
                dm_object.size = aw_shape_to_dm_size(aw_dynamic_object.object)
                # 物標の色 dm_object.color = ObjectColor.UNKNOWN
                # 前輪舵角 dm_object.steering_angle_front = SteeringAngle.UNKNOWN
                # 後輪舵角 dm_object.steering_angle_rear = SteeringAngle.UNKNOWN
                # ブレーキ状態 dm_object.brake_state = BrakeState.UNKNOWN
                # 補助ブレーキ状態 dm_object.auxiliary_brake_state = AuxiliaryBrakeState.UNKNOWN
                # アクセルペダル開度 dm_object.throttle_position = ThrottlePosition.UNKNOWN
                # 灯火の状態 dm_object.exterior_lights = ExteriorLights.UNKNOWN
                # 各種のシステムの作動状態 Default UNKNOWN
                # 車両用途種別 dm_object.vehicle_role = VehicleRole.UNKNOWN
                # 車両用途種別毎の状態 dm_object.vehicle_extended_info = VehicleExtendedInformation.UNKNOWN
                # 牽引車両 dm_object.towing_vehicle = ObjectId.UNKNOWN
            except Exception as e:
                self.get_logger().error("%s" % e)

            # 情報源のリスト
            data_source = ObjectId()
            data_source.value = int.from_bytes([0,0,0,0]+VENDOR_ID, signed=False, byteorder='big')
            # self.get_logger().info("id: {:016x}".format(data_source.value))
            dm_object.information_source_list.append(data_source)

            # Add complete object to the array
            dm_object_info_array.array.append(dm_object)
        # end foreach
        self.dm_publisher_.publish(dm_object_info_array)


def main(args=None):
    rclpy.init(args=args)

    aw_dm_conv = AwDmConverter()

    rclpy.spin(aw_dm_conv)

    aw_dm_conv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
