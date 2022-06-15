# Perception Engine Inc. 2022

import datetime
import math

from rclpy.time import Time
import dm_object_info_msgs.msg
from autoware_perception_msgs.msg import DynamicObject
from autoware_perception_msgs.msg import Semantic
from dm_object_info_msgs.msg import ObjectClass
from dm_object_info_msgs.msg import ClassId
from dm_object_info_msgs.msg import TimestampIts
from dm_object_info_msgs.msg import Location
from dm_object_info_msgs.msg import WGS84Angle
from dm_object_info_msgs.msg import Speed
from dm_object_info_msgs.msg import YawRate
from dm_object_info_msgs.msg import ObjectSize

from .geo_utils import *
from .transformations import *

ALTITUDE_UNAVAILABLE = 0.800001


def aw_shape_to_dm_size(dynamic_object: DynamicObject) -> ObjectSize:
    from dm_object_info_msgs.msg import ObjectDimensionAccuracy
    from dm_object_info_msgs.msg import ObjectDimensionValue
    size = ObjectSize()  # 0,1 m
    dm_w = ObjectDimensionValue()
    dm_accuracy = ObjectDimensionAccuracy()
    dm_accuracy.value = int(ObjectDimensionAccuracy.MAX * 0.8)

    dm_w.value = int(dynamic_object.shape.dimensions.y * 100)
    size.width.value = dm_w
    size.width.accuracy = dm_accuracy
    
    dm_h = ObjectDimensionValue()
    dm_h.value = int(dynamic_object.shape.dimensions.z * 100)
    size.height.value = dm_h
    size.height.accuracy = dm_accuracy
    
    dm_l = ObjectDimensionValue()
    dm_l.value = int(dynamic_object.shape.dimensions.x * 100)
    size.length.value = dm_l
    size.length.accuracy = dm_accuracy
    return size


def aw_twist_to_dm_yaw_rate(dynamic_object: DynamicObject) -> YawRate:
    from dm_object_info_msgs.msg import YawRateAccuracy
    from dm_object_info_msgs.msg import YawRateValue
    yaw_rate = YawRate()

    tmp_yaw = math.degrees(math.sqrt(dynamic_object.state.twist_covariance.twist.angular.x *
                                     dynamic_object.state.twist_covariance.twist.angular.x +
                                     dynamic_object.state.twist_covariance.twist.angular.y *
                                     dynamic_object.state.twist_covariance.twist.angular.y))
    dm_yaw_rate_value = YawRateValue()
    dm_yaw_rate_value.value = int(tmp_yaw * 100)
    yaw_rate.value = dm_yaw_rate_value  # 0,01 degree per second
    yaw_rate.accuracy.value = YawRateAccuracy.MAX
    return yaw_rate


def aw_twist_to_dm_speed(dynamic_object: DynamicObject) -> Speed:
    from dm_object_info_msgs.msg import SpeedAccuracy
    from dm_object_info_msgs.msg import SpeedValue
    speed = Speed()
    tmp_speed = math.sqrt(dynamic_object.state.twist_covariance.twist.linear.x *
                          dynamic_object.state.twist_covariance.twist.linear.x +
                          dynamic_object.state.twist_covariance.twist.linear.y *
                          dynamic_object.state.twist_covariance.twist.linear.y)
    dm_speed_value = SpeedValue()
    dm_speed_value.value = int(tmp_speed * 100)
    speed.value = dm_speed_value  # 0,01 m/s units
    speed.accuracy.value = SpeedAccuracy.MAX
    return speed


def aw_pose_to_dm_direction(dynamic_object: DynamicObject) -> WGS84Angle:
    from dm_object_info_msgs.msg import WGS84AngleAccuracy
    from dm_object_info_msgs.msg import WGS84AngleValue
    angle = WGS84Angle()
    # angle.value
    roll, pitch, yaw = euler_from_quaternion([dynamic_object.state.pose_covariance.pose.orientation.x,
                                                     dynamic_object.state.pose_covariance.pose.orientation.y,
                                                     dynamic_object.state.pose_covariance.pose.orientation.z,
                                                     dynamic_object.state.pose_covariance.pose.orientation.w])

    # WGS84 測地系における方位。
    # 北を 0 とし，東回りで，0.01 度単位で表す。
    # 具体的な表現値は以下の通り。
    # 0: 北
    # ……
    # 9000: 東
    # ……
    # 18000: 南
    # ……
    # 27000: 西
    # ……
    # 36000: 使用しない
    # 36001: 不明
    dm_angle = WGS84AngleValue()
    tmp = abs(int(math.degrees(yaw) * 100))
    print(tmp)
    dm_angle.value = tmp
    angle.value = dm_angle
    angle.accuracy.value = int(WGS84AngleAccuracy.MAX * 0.8)

    return angle


def aw_position_to_dm_location(dynamic_object: DynamicObject,
                               altitude: float = ALTITUDE_UNAVAILABLE,
                               plane_number: int = 7) -> Location:
    from dm_object_info_msgs.msg import Latitude
    from dm_object_info_msgs.msg import Longitude
    from dm_object_info_msgs.msg import Altitude

    dm_location = Location()
    dm_location.geodetic_system.value = dm_object_info_msgs.msg.GeodeticSystem.WGS84
    lat, long = xyp_to_lat_lon(x=dynamic_object.state.pose_covariance.pose.position.x,
                               y=dynamic_object.state.pose_covariance.pose.position.y,
                               plane_num=plane_number)
    dm_latitude = Latitude()
    dm_longitude = Longitude()
    dm_altitude = Altitude()

    dm_latitude.value =int(lat * 1e6)
    dm_longitude.value = int(long * 1e6)
    dm_altitude.value = int(altitude * 1e6)

    dm_location.latitude = dm_latitude
    dm_location.longitude = dm_longitude
    dm_location.altitude = dm_altitude

    dm_location.crp_id.value = 0  # Unknown
    dm_location.dx_crp.value = int(dynamic_object.state.pose_covariance.pose.position.x * 1e6)
    dm_location.dy_crp.value = int(dynamic_object.state.pose_covariance.pose.position.y * 1e6)

    # dm_location.lane_count.value =
    # dm_location.lane_position.value =
    # dm_location.lane_lateral_position.value
    # dm_location.lane_id.value =

    # dm_location.dx_lane =
    # dm_location.dh_lane =
    # dm_location.dh_lane =

    # dm_location.semi_axis_length_major.value =
    # dm_location.semi_axis_length_minor.value =

    # dm_location.orientation.value =
    # dm_location.altitude_accuracy.value =

    return dm_location


def aw_class_to_dm_object_class(dynamic_object: DynamicObject) -> (ObjectClass, int):
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
    dm_object_class.confidence.value = int(dynamic_object.semantic.confidence * 100)
    if dm_object_class.confidence.value > 100:
        dm_object_class.confidence.value = 100
    return dm_object_class, dm_object_class.confidence.value


def ros_stamp_to_de_time(ros_stamp: Time) -> TimestampIts:
    dm_time = TimestampIts()
    de_ts_ms = datetime.datetime(2004, 1, 1, 0, 0).timestamp() * 1000  # ISO 8601
    ros_ts_ms = Time.from_msg(ros_stamp).nanoseconds / 1e3  # epoch time
    dm_time.value = int(ros_ts_ms - de_ts_ms)
    return dm_time
