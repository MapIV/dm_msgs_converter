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


def aw_shape_to_dm_size(dynamic_object: DynamicObject, logger=None) -> ObjectSize:
    from dm_object_info_msgs.msg import ObjectDimensionAccuracy
    from dm_object_info_msgs.msg import ObjectDimensionValue
    size = ObjectSize()  # 0,1 m
    dm_w = ObjectDimensionValue()
    dm_accuracy = ObjectDimensionAccuracy()
    try:
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
    except Exception as e:
        if logger is not None:
            logger.error("aw_shape_to_dm_size: %s" % e)
    return size


def aw_twist_to_dm_yaw_rate(dynamic_object: DynamicObject, logger=None) -> YawRate:
    from dm_object_info_msgs.msg import YawRateAccuracy
    from dm_object_info_msgs.msg import YawRateValue
    yaw_rate = YawRate()
    try:
        tmp_yaw = math.degrees(math.sqrt(dynamic_object.state.twist_covariance.twist.angular.x *
                                         dynamic_object.state.twist_covariance.twist.angular.x +
                                         dynamic_object.state.twist_covariance.twist.angular.y *
                                         dynamic_object.state.twist_covariance.twist.angular.y))
        dm_yaw_rate_value = YawRateValue()
        yaw_value = int(tmp_yaw * 100)
        if yaw_value > YawRateValue.MAX or yaw_value < YawRateValue.MIN:
            yaw_value = YawRateValue.UNKNOWN
        dm_yaw_rate_value.value = yaw_value
        yaw_rate.value = dm_yaw_rate_value  # 0,01 degree per second
        yaw_rate.accuracy.value = YawRateAccuracy.MAX
    except Exception as e:
        if logger is not None:
            logger.error("aw_twist_to_dm_yaw_rate: %s" % e)

    return yaw_rate


def aw_twist_to_dm_speed(dynamic_object: DynamicObject, logger=None) -> Speed:
    from dm_object_info_msgs.msg import SpeedAccuracy
    from dm_object_info_msgs.msg import SpeedValue
    speed = Speed()
    try:
        tmp_speed = math.sqrt(dynamic_object.state.twist_covariance.twist.linear.x *
                              dynamic_object.state.twist_covariance.twist.linear.x +
                              dynamic_object.state.twist_covariance.twist.linear.y *
                              dynamic_object.state.twist_covariance.twist.linear.y)
        dm_speed_value = SpeedValue()
        dm_speed_value.value = int(tmp_speed * 100)
        speed.value = dm_speed_value  # 0,01 m/s units
        speed.accuracy.value = SpeedAccuracy.MAX
    except Exception as e:
        if logger is not None:
            logger.error("aw_twist_to_dm_speed: %s" % e)
    return speed


def aw_pose_to_dm_direction(dynamic_object: DynamicObject, logger=None) -> WGS84Angle:
    from dm_object_info_msgs.msg import WGS84AngleAccuracy
    from dm_object_info_msgs.msg import WGS84AngleValue
    angle = WGS84Angle()
    # angle.value
    try:
        roll, pitch, yaw = euler_from_quaternion([dynamic_object.state.pose_covariance.pose.orientation.x,
                                                         dynamic_object.state.pose_covariance.pose.orientation.y,
                                                         dynamic_object.state.pose_covariance.pose.orientation.z,
                                                         dynamic_object.state.pose_covariance.pose.orientation.w])
        #if logger is not None:
        #    logger.info("CLASS: {:}, roll: {:.4f},  pitch:{:.4f}, yaw: {:.4f}".format(dynamic_object.semantic.type, roll, pitch, yaw))
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
        tmp = abs(int(math.degrees(-yaw+math.pi/2) * 100))
        dm_angle.value = tmp
        angle.value = dm_angle
        angle.accuracy.value = int(WGS84AngleAccuracy.MAX * 0.8)
    except Exception as e:
        if logger is not None:
            logger.error("aw_pose_to_dm_direction: %s" % e)

    return angle


def aw_position_to_dm_location(dynamic_object: DynamicObject,
                               plane_number: int = 7,
                               lidar_offset: float = 0.0,
                               logger=None) -> Location:
    from dm_object_info_msgs.msg import Latitude
    from dm_object_info_msgs.msg import Longitude
    from dm_object_info_msgs.msg import Altitude

    dm_location = Location()
    try:
        # Get yaw information
        roll, pitch, yaw = euler_from_quaternion([dynamic_object.state.pose_covariance.pose.orientation.x,
                                                  dynamic_object.state.pose_covariance.pose.orientation.y,
                                                  dynamic_object.state.pose_covariance.pose.orientation.z,
                                                  dynamic_object.state.pose_covariance.pose.orientation.w])
        x_center = dynamic_object.state.pose_covariance.pose.position.x
        y_center = dynamic_object.state.pose_covariance.pose.position.y
        z_center = dynamic_object.state.pose_covariance.pose.position.z
        speed = math.sqrt(dynamic_object.state.twist_covariance.twist.linear.x ** 2 + \
                          dynamic_object.state.twist_covariance.twist.linear.y ** 2 + \
                          dynamic_object.state.twist_covariance.twist.linear.z ** 2)
        z_bottom = z_center - dynamic_object.shape.dimensions.z / 2
        smartpole_yaw = (-(yaw+lidar_offset + math.pi) % (2.0 * math.pi) - math.pi) * -1.0

        if dynamic_object.semantic.type == 6 or speed < 0.05:       # pedestrian should be set to center
            reference_point_location = [x_center, y_center, z_bottom]
            x_front, y_front, z_bottom = x_center, y_center, z_center
            reference_point_id = 1
        else: # Other objects types are orientation dependent.
            if -math.pi/4 <= smartpole_yaw <= math.pi/4:
                # # Car is moving perpendicular to the left
                x_front = x_center + dynamic_object.shape.dimensions.y * math.sin(yaw) / 2
                y_front = y_center + dynamic_object.shape.dimensions.y * math.cos(yaw) / 2
                reference_point_id = 5
            elif math.pi/4 <= smartpole_yaw <= 3*math.pi/4:
                # Car is moving towards the smartpole
                x_front = x_center + dynamic_object.shape.dimensions.x * math.cos(yaw) / 2
                y_front = y_center + dynamic_object.shape.dimensions.x * math.sin(yaw) / 2
                reference_point_id = 2
            elif -3*math.pi/4 <= smartpole_yaw <= -math.pi/4:
                # Car moving directly away from the smartpole
                reference_point_id = 3
                x_front = x_center + dynamic_object.shape.dimensions.x * math.cos(yaw + math.pi) / 2
                y_front = y_center + dynamic_object.shape.dimensions.x * math.sin(yaw + math.pi) / 2
            else:
                # Car is moving perpendicular to the right
                x_front = x_center + dynamic_object.shape.dimensions.y * math.sin(yaw + math.pi) / 2
                y_front = y_center + dynamic_object.shape.dimensions.y * math.cos(yaw + math.pi) / 2
                reference_point_id = 4
            reference_point_location = [x_front, y_front, z_bottom]

        ### DEBUG INFORMATION
        # if dynamic_object.semantic.type != 0 and speed > 2:
        #     logger.info("class: {}".format(dynamic_object.semantic.type))
        #     logger.info("speed: {}".format(speed))
        #     logger.info("xyz center: {}, {}, {}".format(x_center, y_center, z_center))
        #     logger.info("xyz ref: {}, {}, {}".format(x_front, y_front, z_bottom))
        #     logger.info("rpy: {}, {}, {}".format(roll, pitch, yaw))
        #     logger.info("ref id: {}, actual yaw: {}".format(reference_point_id, smartpole_yaw))
        #     logger.info("--------------------------")

        dm_location.geodetic_system.value = dm_object_info_msgs.msg.GeodeticSystem.WGS84
        lat_rad, long_rad = xyp_to_lat_lon(x=dynamic_object.state.pose_covariance.pose.position.x,
                                           y=dynamic_object.state.pose_covariance.pose.position.y,
                                           plane_num=plane_number)

        lat_deg = np.rad2deg(lat_rad)
        long_deg = np.rad2deg(long_rad)
        altitude_obj = z_bottom

        # if logger is not None:
        #     logger.info("x: {:.4f},  y:{:.4f}".format(dynamic_object.state.pose_covariance.pose.position.y, dynamic_object.state.pose_covariance.pose.position.x))
        #     logger.info("lat: {:.4f},  long:{:.4f}, altitude: {:.4f}".format(lat_deg, long_deg, altitude_obj))
        dm_latitude = Latitude()
        dm_longitude = Longitude()
        dm_altitude = Altitude()

        dm_latitude.value = int(lat_deg * 1e7)
        dm_longitude.value = int(long_deg * 1e7)
        dm_altitude.value = int(altitude_obj * 1e2)
        dm_location.latitude = dm_latitude
        dm_location.longitude = dm_longitude
        dm_location.altitude = dm_altitude

        # TODO: DISTANCE from sensor?
        dm_location.crp_id.value = 0  # Unknown
        dm_location.dx_crp.value = 0 # int(dynamic_object.state.pose_covariance.pose.position.x * 1e6)
        dm_location.dy_crp.value = 0 # int(dynamic_object.state.pose_covariance.pose.position.y * 1e6)

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
    except Exception as e:
        if logger is not None:
            logger.error("aw_position_to_dm_location: %s" % e)

    return dm_location, reference_point_location, reference_point_id


def aw_class_to_dm_object_class(dynamic_object: DynamicObject, logger=None) -> (ObjectClass, int):
    dm_object_class = ObjectClass()
    try:
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
        tmp = int(dynamic_object.semantic.confidence * 10)
        if tmp > 100:
            tmp = 100
        dm_object_class.confidence.value = tmp
    except Exception as e:
        if logger is not None:
            logger.error("aw_class_to_dm_object_class: %s" % e)

    return dm_object_class, dm_object_class.confidence.value


def ros_stamp_to_de_time(ros_stamp: Time, logger=None) -> TimestampIts:
    dm_time = TimestampIts()
    try:
        de_ts = datetime.datetime(2004, 1, 1, 0, 0)  # ISO 8601
        tmp = Time.from_msg(ros_stamp).nanoseconds/1e9
        ros_ts = datetime.datetime.fromtimestamp(tmp)  # epoch time
        time_diff = ros_ts - de_ts
        dm_time.value = int(time_diff.total_seconds()*1e3)
    except Exception as e:
        if logger is not None:
            logger.error("ros_stamp_to_de_time: %s" % e)
    return dm_time
