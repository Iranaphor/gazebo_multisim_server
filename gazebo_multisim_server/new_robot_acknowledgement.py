#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import re

# Message we plan to publish
from rcl_interfaces.msg import Log

class RosoutChecker(Node):
    def __init__(self):
        """ Node that periodically checks for '/namespace/rosout' topics
            and publishes a Log message stating '<namespace> is connected' 
            whenever discovered. """
        super().__init__('rosout_checker')

        # Track any publishers we've created so we don't recreate them
        self.rosout_publishers = {}

        # Create a timer to repeatedly check for topics
        # Adjust the timer period (in seconds) as needed
        self.timer = self.create_timer(2.0, self.check_for_rosout_topics)

    def check_for_rosout_topics(self):
        all_topics = self.get_topic_names_and_types()
        for topic_name, _ in all_topics:
            # Check if topic matches the pattern /namespace/rosout
            match = re.match(r'^/([^/]+)/rosout$', topic_name)
            if match:
                namespace = match.group(1)
                # If we haven't already created a publisher for this exact topic, do so
                if topic_name not in self.rosout_publishers:
                    # Create a QoS profile matching what is typically used by rosout
                    qos = QoSProfile(depth=10)
                    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
                    qos.reliability = ReliabilityPolicy.RELIABLE

                    # Create publisher for rcl_interfaces/msg/Log on the discovered rosout
                    pub = self.create_publisher(Log, topic_name, qos)
                    self.rosout_publishers[topic_name] = pub
                    self.get_logger().info(f"Created publisher for topic: {topic_name}")

                # Construct and publish the Log message
                log_msg = Log()
                log_msg.msg = f"{namespace} is connected"
                log_pub = self.rosout_publishers[topic_name]
                log_pub.publish(log_msg)
                self.get_logger().info(f"Published Log msg to {topic_name}: '{log_msg.msg}'")


def main(args=None):
    rclpy.init(args=args)

    node = RosoutChecker()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
