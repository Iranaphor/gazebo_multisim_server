import rclpy
import json
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import String  # Assuming messages are JSON-encoded strings

class Republisher(Node):
    def __init__(self):
        super().__init__('repub')
        self.subscribers = {}  # Dictionary to store active subscriptions
        self.timer = self.create_timer(1.0, self.timer_cb)  # Check topics every second
        self.client = self.create_client(SpawnEntity, '/spawn_entity')

    def timer_cb(self):
        # Get list of all topics
        topic_list = self.get_topic_names_and_types()
        spawn_topics = [t for t, _ in topic_list if t.endswith('/spawn_entity') and t != '/spawn_entity']

        # Extract namespaces
        namespaces = [t[:-len('/spawn_entity')] for t in spawn_topics]

        # Subscribe to new topics
        for ns, topic in zip(namespaces, spawn_topics):
            if ns not in self.subscribers:
                qos_profile = QoSProfile(
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=10,
                    reliability=QoSReliabilityPolicy.RELIABLE
                )
                self.subscribers[ns] = self.create_subscription(
                    String, topic, lambda msg, ns=ns: self.spawn_cb(msg, ns), qos_profile
                )
                self.get_logger().info(f'Subscribed to {topic}')

    def spawn_cb(self, msg, ns):
        """ Callback to handle incoming messages, parse JSON, and call service. """
        try:
            data = json.loads(msg.data)  # Decode JSON
            request = SpawnEntity.Request()
            request.name = data.get("name", "default_name")
            request.xml = data.get("xml", "<sdf></sdf>")
            request.robot_namespace = ns  # Use namespace from topic
            request.initial_pose.position.x = data.get("x", 0.0)
            request.initial_pose.position.y = data.get("y", 0.0)
            request.initial_pose.position.z = data.get("z", 0.0)

            # Call the service
            if self.client.wait_for_service(timeout_sec=2.0):
                future = self.client.call_async(request)
                future.add_done_callback(self.spawn_response_cb)
                self.get_logger().info(f'Spawn request sent for {request.name}')
            else:
                self.get_logger().error('SpawnEntity service unavailable')

        except json.JSONDecodeError:
            self.get_logger().error('Failed to decode JSON from topic')

    def spawn_response_cb(self, future):
        """ Handle the response from the spawn service. """
        try:
            response = future.result()
            self.get_logger().info(f'SpawnEntity response: {response.success}')
        except Exception as e:
            self.get_logger().error(f'SpawnEntity call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = Republisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
