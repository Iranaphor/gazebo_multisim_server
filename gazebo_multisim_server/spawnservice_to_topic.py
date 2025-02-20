import rclpy
import json
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from gazebo_msgs.srv import SpawnEntity  # Use the correct service type
from std_msgs.msg import String  # JSON data will be sent as a String message

class ServiceToTopic(Node):
    def __init__(self):
        super().__init__('service_to_topic')

        # Create service proxy
        self.service = self.create_service(SpawnEntity, '/spawn_entity', self.service_callback)
        self.publisher = self.create_publisher(String, '/spawn_entity', self.get_qos())

        self.get_logger().info('Service to Topic bridge initialized.')

    def get_qos(self):
        """ Sets QoS profile to ensure reliable message delivery. """
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

    def service_callback(self, request, response):
        """ Intercept the SpawnEntity service request and publish it as JSON. """
        try:
            # Convert request to dictionary
            data = {
                "name": request.name,
                "xml": request.xml,
                "x": request.initial_pose.position.x,
                "y": request.initial_pose.position.y,
                "z": request.initial_pose.position.z,
                "namespace": request.robot_namespace,
            }
            
            # Encode as JSON and publish
            msg = String()
            msg.data = json.dumps(data)
            self.publisher.publish(msg)

            self.get_logger().info(f'Published spawn request: {msg.data}')

            # Respond to the original service call
            response.success = True
            response.status_message = "Spawn request forwarded as JSON"
            return response

        except Exception as e:
            self.get_logger().error(f'Error processing service request: {str(e)}')
            response.success = False
            response.status_message = str(e)
            return response

def main():
    rclpy.init()
    node = ServiceToTopic()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
