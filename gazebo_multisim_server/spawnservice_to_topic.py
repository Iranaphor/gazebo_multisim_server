import rclpy
import json
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import String
import re

class ServiceToTopic(Node):
    def __init__(self):
        super().__init__('service_to_topic')

        # Create service proxy
        self.service = self.create_service(SpawnEntity, '/spawn_entity', self.service_callback)
        self.publisher = self.create_publisher(String, '/spawn_entity', self.get_qos())

        # Dictionary to track file publishers
        self.file_publishers = {}

        self.get_logger().info('Service to Topic bridge initialized.')

    def get_qos(self):
        """ Sets QoS profile to ensure reliable and latched message delivery. """
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL  # Enables latching
        )

    def service_callback(self, request, response):
        """ Intercept the SpawnEntity service request, process file references, and publish as JSON. """
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

            # Identify file references in the XML
            file_references = self.extract_file_paths(request.xml)

            # Read and publish file contents
            for file_path in file_references:
                file_content = self.read_file(file_path.replace('file://', ''))
                if file_content:
                    topic_name = self.create_topic_name(file_path)
                    self.publish_file_content(topic_name, file_content)
                    self.get_logger().info(f'Published file ({file_path}) on: {topic_name}')

            # Encode as JSON and publish
            msg = String()
            msg.data = json.dumps(data)
            self.publisher.publish(msg)

            self.get_logger().info(f'Published spawn request:')
            self.get_logger().debug(f'{msg.data}')

            # Respond to the original service call
            response.success = True
            response.status_message = "Spawn request forwarded as JSON"
            return response

        except Exception as e:
            self.get_logger().error(f'Error processing service request: {str(e)}')
            response.success = False
            response.status_message = str(e)
            return response
        

    def extract_file_paths(self, xml_string):
        """
        Extract file paths from XML content, looking for .dae, .urdf, .sdf,
        .xacro, .yaml, or .csv files. Handles two common cases:
        1) Paths inside quotes (e.g. filename="..." or '...')
        2) Paths inside element tags (e.g. <parameters>...</parameters>)
        """
        # Extend the set of file extensions:
        file_extensions = ('.dae', '.urdf', '.sdf', '.xacro', '.yaml', '.csv')

        # We create a single pattern that tries to match either:
        #   1) A path in quotes (group 1)
        #   2) A path in the > ... < tag content (group 2)
        #
        # Explanation:
        #   - (?:["\']([^"\'<>]*\.(?:extensions))["\']) captures quoted paths
        #   - |>([^<>]*\.(?:extensions))< captures unquoted paths between > and <
        #
        pattern = (
            r'(?:["\']([^"\'<>]*\.(?:dae|urdf|sdf|xacro|yaml|csv))["\'])'
            r'|>([^<>]*\.(?:dae|urdf|sdf|xacro|yaml|csv))<'
        )

        # Run the findall; each match is a tuple (group1, group2)
        matches = re.findall(pattern, xml_string)

        # Collect whichever capturing group is non-empty
        file_paths = []
        for g1, g2 in matches:
            file_paths.append(g1 if g1 else g2)

        return file_paths


    def read_file(self, file_path):
        """ Reads the content of a file if it exists. """
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r') as file:
                    return file.read()
            except Exception as e:
                self.get_logger().error(f'Failed to read {file_path}: {str(e)}')
        else:
            self.get_logger().warn(f'File not found: {file_path}')
        return None

    def create_topic_name(self, file_path):
        """ Generates a valid topic name based on the file path. """
        file_name = os.path.basename(file_path).replace(".", "_")  # Replace dots with underscores
        return f"spawn_entity/file_content/{file_name}"

    def publish_file_content(self, topic_name, content):
        """ Publishes the file content to an appropriately named topic. """
        if topic_name not in self.file_publishers:
            self.file_publishers[topic_name] = self.create_publisher(String, topic_name, self.get_qos())

        msg = String()
        msg.data = content
        self.file_publishers[topic_name].publish(msg)
        self.get_logger().info(f'Published file content to {topic_name}')

def main():
    rclpy.init()
    node = ServiceToTopic()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


