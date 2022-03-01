# Perception Engine Inc. 2022

import datetime

from rclpy.time import Time
import dm_object_info_msgs.msg
from autoware_perception_msgs.msg import DynamicObject
from autoware_perception_msgs.msg import Semantic
from dm_object_info_msgs.msg import ObjectClass
from dm_object_info_msgs.msg import ClassId
from dm_object_info_msgs.msg import TimestampIts
from dm_object_info_msgs.msg import Location

from geo_utils import *

ALTITUDE_UNAVAILABLE = 0.800001


def aw_position_to_dm_location(dynamic_object: DynamicObject, altitude=ALTITUDE_UNAVAILABLE) -> Location:
    dm_location = Location()
    dm_location.geodetic_system.value = dm_object_info_msgs.msg.GeodeticSystem.WGS84
    lat, long = xyp_to_lat_lon(dynamic_object.state.pose_covariance.pose.position.x,
                               dynamic_object.state.pose_covariance.pose.position.y)
    dm_location.latitude = lat * 1e6
    dm_location.longitude = long * 1e6
    dm_location.altitude = altitude * 1e6

    dm_location.crp_id.value = 0  # Unknown
    dm_location.dx_crp.value = dynamic_object.state.pose_covariance.pose.position.x * 1e6
    dm_location.dy_crp.value = dynamic_object.state.pose_covariance.pose.position.y * 1e6

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
    dm_object_class.confidence.value = dynamic_object.semantic.confidence * 100
    if dm_object_class.confidence.value > 100:
        dm_object_class.confidence.value = 100
    return dm_object_class, dm_object_class.confidence.value


def ros_stamp_to_de_time(ros_stamp: Time) -> TimestampIts:
    dm_time = TimestampIts()
    de_ts_ms = datetime.datetime(2004, 1, 1, 0, 0).timestamp() * 1000  # ISO 8601
    ros_ts_ms = Time.from_msg(ros_stamp).nanoseconds / 1e6  # epoch time
    dm_time.value = ros_ts_ms - de_ts_ms
    return dm_time
