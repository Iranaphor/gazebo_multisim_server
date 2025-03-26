#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from sensor_msgs.msg import NavSatFix


class NavSatTransformBase(Node):
    def __init__(self):
        super().__init__('navsat_transform_base')

        # -- Declare default parameters --

        # Booleans
        self.declare_parameter(
            name='broadcast_cartesian_transform',
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='broadcast_cartesian_transform == Boolean value is: False'
            )
        )
        self.declare_parameter(
            name='broadcast_cartesian_transform_as_parent_frame',
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='broadcast_cartesian_transform_as_parent_frame == Boolean value is: False'
            )
        )
        self.declare_parameter(
            name='broadcast_utm_transform',
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='broadcast_utm_transform == Boolean value is: False'
            )
        )
        self.declare_parameter(
            name='broadcast_utm_transform_as_parent_frame',
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='broadcast_utm_transform_as_parent_frame == Boolean value is: False'
            )
        )
        self.declare_parameter(
            name='publish_filtered_gps',
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='publish_filtered_gps == Boolean value is: True'
            )
        )
        self.declare_parameter(
            name='use_local_cartesian',
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='use_local_cartesian == Boolean value is: False'
            )
        )
        self.declare_parameter(
            name='use_odometry_yaw',
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='use_odometry_yaw == Boolean value is: False'
            )
        )
        # self.declare_parameter(
        #     name='use_sim_time',
        #     value=True,
        #     descriptor=ParameterDescriptor(
        #         type=ParameterType.PARAMETER_BOOL,
        #         description='use_sim_time == Boolean value is: True'
        #     )
        # )
        self.declare_parameter(
            name='wait_for_datum',
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='wait_for_datum == Boolean value is: True'
            )
        )
        self.declare_parameter(
            name='zero_altitude',
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='zero_altitude == Boolean value is: True'
            )
        )

        # Doubles
        self.declare_parameter(
            name='delay',
            value=0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='delay == Double value is: 0.0'
            )
        )
        self.declare_parameter(
            name='frequency',
            value=10.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='frequency == Double value is: 10.0'
            )
        )
        self.declare_parameter(
            name='magnetic_declination_radians',
            value=0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='magnetic_declination_radians == Double value is: 0.0'
            )
        )
        self.declare_parameter(
            name='transform_timeout',
            value=0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='transform_timeout == Double value is: 0.0'
            )
        )
        self.declare_parameter(
            name='yaw_offset',
            value=0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='yaw_offset == Double value is: 0.0'
            )
        )

        # A DOUBLE_ARRAY for datum [lat, lon, alt]
        self.declare_parameter(
            name='datum',
            value=[53.26838, -0.524501, 0.0],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description='datum == Double values are: array(\'d\', [53.26838, -0.524501, 0.0])'
            )
        )

        # -- Done declaring default parameters --

        # Track topics we have subscribed to so we don't duplicate them
        self.subscribed_topics = set()

        # Create a timer to periodically discover any new topics that end with "/datum"
        self.discovery_timer = self.create_timer(2.0, self.discover_datum_topics)

        self.get_logger().info('navsat_transform_base node initialized with default parameters.')

    def discover_datum_topics(self):
        """
        Periodically discovers topics whose names end with '/datum'.
        If the topic type is sensor_msgs/msg/NavSatFix, we subscribe.
        """
        topic_list = self.get_topic_names_and_types()

        for (topic_name, types) in topic_list:
            if topic_name.endswith('/datum'):
                if 'sensor_msgs/msg/NavSatFix' in types:
                    if topic_name not in self.subscribed_topics:
                        self.get_logger().info(f"Discovered new NavSatFix /datum topic: {topic_name}")
                        self.create_subscription(
                            NavSatFix,
                            topic_name,
                            self.datum_callback,
                            QoSProfile(depth=10)
                        )
                        self.subscribed_topics.add(topic_name)

    def datum_callback(self, msg: NavSatFix):
        """
        Callback for NavSatFix messages on discovered /datum topics.
        Updates the 'datum' parameter with [latitude, longitude, altitude].
        """
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        self.get_logger().info(f"Received NavSatFix => lat={lat}, lon={lon}, alt={alt}")

        # Update the 'datum' parameter as a DOUBLE_ARRAY
        new_datum_param = Parameter(
            name='datum',
            type_=Parameter.Type.DOUBLE_ARRAY,
            value=[lat, lon, alt]
        )
        self.set_parameters([new_datum_param])
        self.get_logger().info(f"Parameter 'datum' updated to: {[lat, lon, alt]}")


def main(args=None):
    rclpy.init(args=args)
    node = NavSatTransformBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
