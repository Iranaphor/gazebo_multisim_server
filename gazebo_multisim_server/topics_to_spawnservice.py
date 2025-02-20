import rclpy
import json
import os
import re
import re
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import String

class TopicsToService(Node):
    def __init__(self):
        super().__init__('topics_to_service')

        # Dictionary to track subscriptions for each namespace
        self.subscribers = {}

        # Dictionary to track pending requests and required file references
        self.pending_requests = {}

        # Dictionary to track received file contents
        self.file_contents = {}

        # Timer to scan for new `/spawn_entity` topics dynamically
        self.create_timer(1.0, self.scan_topics)

        self.get_logger().info('Topics to Service bridge initialized.')

    def get_qos(self):
        """ Sets QoS profile for reliable message delivery. """
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

    def scan_topics(self):
        """ Scans and subscribes to all active `/spawn_entity` topics dynamically. """
        topic_list = self.get_topic_names_and_types()
        spawn_topics = [t for t, _ in topic_list if t.endswith('/spawn_entity')]

        for topic in spawn_topics:
            namespace = topic[:-len('/spawn_entity')]  # Extract namespace
            if namespace not in self.subscribers:
                self.subscribers[namespace] = self.create_subscription(
                    String, topic, lambda msg, ns=namespace: self.spawn_callback(msg, ns), self.get_qos()
                )
                self.get_logger().info(f'Subscribed to {topic}')

    def spawn_callback(self, msg, namespace):
        """ Handles JSON-encoded spawn requests, subscribes to required file topics, and processes them. """
        try:
            data = json.loads(msg.data)
            file_references = self.extract_file_paths(data["xml"])
            print(file_references)

            # If there are file references, subscribe to them and wait
            if file_references:
                self.pending_requests[namespace] = {"data": data, "remaining_files": set(file_references)}
                self.get_logger().info(f'Waiting for {len(set(file_references))} file(s) for namespace {namespace}.')

                for file_path in set(file_references):
                    topic_name = self.create_topic_name(file_path, namespace)
                    self.get_logger().info(f'- {topic_name} -> {file_path}')
                    if topic_name not in self.file_contents:
                        print('subscribed')
                        self.create_subscription(String, topic_name, lambda msg, f=file_path, ns=namespace: self.file_callback(msg, f, ns), self.get_qos())
                    print(' ')

            else:
                self.forward_spawn_request(namespace, data)

        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON from {namespace}/spawn_entity topic.")

    def file_callback(self, msg, file_path, namespace):
        """ Saves received file content and checks if all required files are received. """
        try:
            # Rework filename
            original_file_path = file_path
            file_path = file_path.replace('file://', '')
            filename = file_path.split('/meshes/')[1]
            pkg = get_package_share_directory('gazebo_multisim_server')
            file_path = os.path.join(pkg, 'meshes', filename)
            #print(f"Filepath reworked to: {file_path}")
            
            file_dir = os.path.dirname(file_path)
            if file_dir and not os.path.exists(file_dir):
                os.makedirs(file_dir, exist_ok=True)

            with open(file_path, 'w') as file:
                file.write(msg.data)
                self.get_logger().info(f'Saved file: {file_path}')

            # Mark this file as received
            self.file_contents[file_path] = True

            # Check if all required files are received for this namespace
            if namespace in self.pending_requests:
                self.pending_requests[namespace]["remaining_files"].discard(original_file_path)

                if not self.pending_requests[namespace]["remaining_files"]:
                    self.forward_spawn_request(namespace, self.pending_requests[namespace]["data"])
                    del self.pending_requests[namespace]
                else:
                    print([r.split('/meshes/')[1] for r in self.pending_requests[namespace]["remaining_files"]], '\n')


        except Exception as e:
            self.get_logger().error(f'Error writing file {file_path}: {str(e)}')

    def rename_files_to_be_local(self, xml):
        """ Rewrites file paths in XML to point to the local package directory. """
        pkg = get_package_share_directory('gazebo_multisim_server')
        replace_from_rgx = r'file://[^<"]*/meshes/'  # Match 'file://...' paths inside XML
        replace_with_str = f'file://{pkg}/meshes/'

        # Perform the replacement
        updated_xml = re.sub(replace_from_rgx, replace_with_str, xml)

        return updated_xml


    def forward_spawn_request(self, namespace, data):
        """ Calls the correct `/spawn_entity` service with the reconstructed request. """

        request = SpawnEntity.Request()
        request.name = data.get("name", "default_name") + namespace
        request.xml = self.rename_files_to_be_local(data.get("xml", "<sdf></sdf>"))
        request.robot_namespace = namespace
        request.initial_pose.position.x = data.get("x", 0.0)
        request.initial_pose.position.y = data.get("y", 0.0)
        request.initial_pose.position.z = data.get("z", 0.0)

        service_name = f"/spawn_entity"
        client = self.create_client(SpawnEntity, service_name)

        if client.wait_for_service(timeout_sec=2.0):
            future = client.call_async(request)
            future.add_done_callback(lambda fut: self.spawn_response_callback(fut, namespace))
            self.get_logger().info(f'Forwarded spawn request for {request.name} to {service_name}')
        else:
            self.get_logger().error(f'SpawnEntity service unavailable at {service_name}')

    def spawn_response_callback(self, future, namespace):
        """ Logs the response from the spawn service. """
        try:
            response = future.result()
            self.get_logger().info(f'SpawnEntity response from {namespace}: {response.success}')
        except Exception as e:
            self.get_logger().error(f'SpawnEntity call failed in {namespace}: {str(e)}')

    def extract_file_paths(self, xml_string):
        """ Extracts file paths from XML content. """
        file_extensions = (".dae", ".urdf", ".sdf", ".xacro")
        file_paths = re.findall(r'["\'](.*?(' + '|'.join(file_extensions) + r'))["\']', xml_string)
        return [fp[0] for fp in file_paths]

    def create_topic_name(self, file_path, namespace):
        """ Generates a topic name based on the file path. """
        file_name = os.path.basename(file_path).replace(".", "_")
        return f"{namespace}/spawn_entity/file_content/{file_name}"

def main():
    rclpy.init()
    node = TopicsToService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
