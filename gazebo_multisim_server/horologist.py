import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rosgraph_msgs.msg import Clock

class ClockRepublisher(Node):
    def __init__(self):
        super().__init__('clock_republisher')

        # Dictionary to hold publishers for each detected namespace
        self.mypublishers = {}

        # Subscribe to the global /clock topic
        self.clock_subscriber = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            self.get_qos()
        )

        # Timer to periodically scan for topics with the format /{namespace}/clock
        self.create_timer(1.0, self.scan_topics)

        self.get_logger().info('Clock Republisher initialized.')

    def get_qos(self):
        """Sets a QoS profile to ensure reliable and latched message delivery."""
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

    def scan_topics(self):
        """
        Scans for topics ending with '/clock' (excluding the global /clock)
        and creates a publisher for each namespace.
        """
        topic_list = self.get_topic_names_and_types()
        for topic, types in topic_list:
            # Skip the global /clock topic
            if topic == '/clock':
                continue
            # Check for topics that end with '/clock'
            if topic.endswith('/clock'):
                # Extract namespace by removing the trailing '/clock'
                namespace = topic[:-len('/clock')]
                if namespace not in self.mypublishers:
                    # Create a publisher for this namespace-specific clock topic
                    self.mypublishers[namespace] = self.create_publisher(
                        Clock,
                        topic,
                        self.get_qos()
                    )
                    self.get_logger().info(f'Publishing to {topic} for namespace "{namespace}"')

    def clock_callback(self, msg):
        """Republishes the /clock message to every namespace-specific clock topic."""
        for namespace, publisher in self.mypublishers.items():
            publisher.publish(msg)

def main():
    rclpy.init()
    node = ClockRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
