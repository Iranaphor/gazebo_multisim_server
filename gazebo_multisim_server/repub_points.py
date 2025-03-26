#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2

class DynamicTopicRepublisher(Node):
    def __init__(self):
        super().__init__('dynamic_topic_republisher')

        # How often (in seconds) to rediscover topics
        self.timer_period = 30.0

        # Keep track of subscriptions/publishers so we don't double-subscribe
        self.my_subscriptions = {}
        self.my_publishers = {}

        # Discover and subscribe to topics on startup
        self.discover_topics()

        # Create a timer to rediscover topics periodically
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            f"[DynamicTopicRepublisher] Node started. Will rediscover topics every {self.timer_period} seconds."
        )

    def discover_topics(self):
        """Discover all ROS topics and subscribe to those ending with '/points_PointCloud2'."""
        all_topics = self.get_topic_names_and_types()
        for (topic_name, topic_types) in all_topics:
            # Optionally check the type is sensor_msgs/msg/PointCloud2 if you want additional safety:
            # if 'sensor_msgs/msg/PointCloud2' in topic_types and topic_name.endswith('/points_PointCloud2'):
            if topic_name.endswith('/points_PointCloud2'):
                self.subscribe_to_topic(topic_name)

    def subscribe_to_topic(self, topic_name: str):
        """Create a subscription for the topic if not already subscribed."""
        if topic_name in self.my_subscriptions:
            return  # Already subscribed

        self.get_logger().info(f"Subscribing to: {topic_name}")
        subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            lambda msg, t=topic_name: self._lidar_callback(msg, t),
            QoSProfile(depth=10)
        )
        self.my_subscriptions[topic_name] = subscription

    def _lidar_callback(self, msg: PointCloud2, topic_name: str):
        """Callback for each subscribed topic. Republishes to the matching '/points' topic."""
        # Determine the publish topic by removing '/points_PointCloud2' and appending '/points'
        publish_topic = topic_name.rsplit('/points_PointCloud2', 1)[0] + '/points'

        # Create publisher if not already created for this topic
        if topic_name not in self.my_publishers:
            self.get_logger().info(f"Creating publisher for: {publish_topic}")
            publisher = self.create_publisher(PointCloud2, publish_topic, QoSProfile(depth=10))
            self.my_publishers[topic_name] = publisher

        # Publish the received message directly to the '/points' topic
        self.my_publishers[topic_name].publish(msg)

    def timer_callback(self):
        """
        Called periodically to clear existing subscriptions/publishers and 
        discover new topics that end with '/points_PointCloud2'.
        """
        self.get_logger().info("[DynamicTopicRepublisher] Refreshing subscriptions...")

        # Tear down existing subscriptions and publishers so we can discover fresh
        self.my_subscriptions.clear()
        self.my_publishers.clear()

        self.discover_topics()

def main(args=None):
    rclpy.init(args=args)

    node = DynamicTopicRepublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
