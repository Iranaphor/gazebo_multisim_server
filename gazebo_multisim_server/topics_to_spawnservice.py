import rclpy
import json
import os
import re
from copy import deepcopy
import xml.etree.ElementTree as ET
import ctypes.util

from ament_index_python import get_package_share_directory

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue


class TopicsToService(Node):
    def __init__(self):
        super().__init__('topics_to_service')

        # Dictionary to track subscriptions for each namespace
        self.subscribers = {}

        # Dictionary to track pending requests and required file references
        self.pending_requests = {}

        # Dictionary to track received file contents
        self.file_contents = {}

        # Create publishers for inspection
        self.xml_pub = self.create_publisher(String, '/spawn_entity_srv_content', self.get_qos())
        self.meta_pub = self.create_publisher(String, '/spawn_entity_srv_meta_content', self.get_qos())
        self.all_installed_plugins = self.create_publisher(KeyValue, '/spawn_entity_plugins_installed', self.get_qos())

        # Timer to scan for new `/spawn_entity` topics dynamically
        self.create_timer(1.0, self.scan_topics)

        self.get_logger().info('Topics to Service bridge initialized.')

    def get_qos(self):
        """ Sets QoS profile to ensure reliable and latched message delivery. """
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL  # Enables latching
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
                        self.create_subscription(String, topic_name,
                                                 lambda msg, f=file_path, ns=namespace: self.file_callback(msg, f, ns),
                                                 self.get_qos())
                    print(' ')

            else:
                self.forward_spawn_request(namespace, data)

        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON from {namespace}/spawn_entity topic.")






    def resolve_local_path(self, original_file_path):
        """
        Convert a 'file://...' or absolute path to a local path
        under 'gazebo_multisim_server' share directory,
        based on whether it's a mesh, config, or CSV, etc.
        """

        # Remove any leading "file://"
        local_path = original_file_path.replace('file://', '')

        # We'll store files in subfolders under this package.
        pkg = get_package_share_directory('gazebo_multisim_server')

        # -- Example heuristics below --
        # 1) If path includes '/meshes/' or ends with '.dae', place in pkg/meshes/
        # 2) If path includes '/config/' or ends with '.yaml', place in pkg/config/
        # 3) If it ends with '.csv' or has '/scan_mode/' for Livox, place in pkg/csv/

        if '/meshes/' in local_path or local_path.endswith('.dae'):
            # Take everything after '/meshes/' if it exists, otherwise just use basename
            if '/meshes/' in local_path:
                relative_part = local_path.split('/meshes/', 1)[1]
                local_path = os.path.join(pkg, 'meshes', relative_part)
            else:
                local_path = os.path.join(pkg, 'meshes', os.path.basename(local_path))

        elif '/config/' in local_path or local_path.endswith('.yaml'):
            # If there's a /config/, keep path structure after that
            if '/config/' in local_path:
                relative_part = local_path.split('/config/', 1)[1]
                local_path = os.path.join(pkg, 'config', relative_part)
            else:
                local_path = os.path.join(pkg, 'config', os.path.basename(local_path))

        elif local_path.endswith('.csv') or '/scan_mode/' in local_path:
            # CSV or a known subpath for Livox scans
            # Similar approach to keep sub-structure after '/scan_mode/', if you prefer
            if '/scan_mode/' in local_path:
                relative_part = local_path.split('/scan_mode/', 1)[1]
                local_path = os.path.join(pkg, 'csv', relative_part)
            else:
                local_path = os.path.join(pkg, 'csv', os.path.basename(local_path))

        else:
            # Fallback if none of the above patterns match:
            local_path = os.path.join(pkg, os.path.basename(local_path))

        return local_path

    def apply_namespace_to_paramater_yaml(self, data):
        import yaml
        print('---')
        print(data)
        print('---')
        data = yaml.dump({'/**': yaml.safe_load(data)})
        print('---')
        print(data)
        print('---')
        return data

    def file_callback(self, msg, file_path, namespace):
        # """ Saves received file content and checks if all required files are received. """
        # try:
            if namespace in self.pending_requests and file_path not in self.pending_requests[namespace]["remaining_files"]:
                return

            # Resolve local path based on our new logic
            local_path = self.resolve_local_path(file_path)

            file_dir = os.path.dirname(local_path)
            if file_dir and not os.path.exists(file_dir):
                os.makedirs(file_dir, exist_ok=True)

            # If msg.data might be a dict for CSV or other data, convert to string
            data_to_write = msg.data
            if not isinstance(data_to_write, str):
                import json
                data_to_write = json.dumps(data_to_write, indent=2)

            # If data should come from yaml file, apply namespace to the nodes in it
            if '.yaml' in file_path:
                data_to_write = self.apply_namespace_to_paramater_yaml(data_to_write)

            with open(local_path, 'w') as f:
                f.write(data_to_write)
                self.get_logger().info(f'{namespace} saved file: {local_path}')

            # Mark this file as received
            self.file_contents[local_path] = True

            # Check if all required files are received for this namespace
            if namespace in self.pending_requests:
                self.pending_requests[namespace]["remaining_files"].discard(file_path)

                if not self.pending_requests[namespace]["remaining_files"]:
                    self.forward_spawn_request(namespace, self.pending_requests[namespace]["data"])
                    del self.pending_requests[namespace]
                else:
                    pending = self.pending_requests[namespace]["remaining_files"]
                    self.get_logger().info(f"{namespace} still waiting for: {[p.split('/')[-1] for p in pending]}\n")


        # except Exception as e:
        #     self.get_logger().error(f'Error writing file {file_path}: {str(e)}')


    def rename_topics_to_be_local(self, xml, namespace):
        name = namespace.replace('/','')

        # topic (no qos)
        xml = xml.replace('name="/', 'name="')
        xml = xml.replace('<namespace/>', '') #remove dead namespace declarations
        xml = xml.replace('<ros>',
                         f'<ros>\n<namespace>{name}</namespace>') # apply namespaces to every ros node

	# explicit remappigns
        #xml = xml.replace('<plugin name="', f'<plugin name="{name}') # TODO: do this ia xml pkg

        xml = xml.replace('name=\"back_lidar_link_plugin\"',
                         f'name=\"{name}_back_lidar_link_plugin\"')

        xml = xml.replace('name=\"gazebo_ros2_control\">',
                         f'name=\"{name}_gazebo_ros2_control\">')


        # topic (with qos)
        # TODO: this

        # remappings
        xml = xml.replace('~/out:=/','~/out:=')
        return xml

    def rename_files_to_be_local(self, xml):
        """
        Rewrites file paths in XML to point to directories inside
        the local 'gazebo_multisim_server' package. Handles:
        1) Meshes    -> file://.../meshes/
        2) YAML      -> <parameters>/home/.../something.yaml</parameters>
        3) CSV       -> <csv_file_name>/home/.../something.csv</csv_file_name>
        """
        pkg = get_package_share_directory('gazebo_multisim_server')

        # ------------------------
        # 1) MESHES
        # ------------------------
        # Match file://ANYTHING/meshes/ and rewrite to file://PKG/meshes/
        # e.g. "file:///home/ros/XYZ/meshes/chassis.dae"
        # -> "file://.../gazebo_multisim_server/meshes/chassis.dae"
        replace_from_rgx = r'file://[^<"]*/meshes/'
        replace_with_str = f'file://{pkg}/meshes/'
        updated_xml = re.sub(replace_from_rgx, replace_with_str, xml)

        # ------------------------
        # 2) YAML files inside <parameters> tags
        # ------------------------
        # We look specifically for content inside <parameters>...</parameters> blocks
        # that ends with .yaml. We'll rewrite the path to something like:
        #      /home/ros/.../my_controller.yaml -> file://{pkg}/config/my_controller.yaml
        #
        # First define a regex capturing the inside of <parameters>...</parameters>.
        # We'll look for text that ends in ".yaml"
        param_pattern = r'(?<=<parameters>)([^<"]*\.yaml)(?=</parameters>)'
        # We'll define a function-based replacement so we can do partial rewriting.
        def replace_yaml_path(match):
            original_path = match.group(0)  # The entire matched string
            filename = os.path.basename(original_path)  # e.g. "ackermann_like_controller.yaml"
            return f'{pkg}/config/{filename}'    # or wherever you want to store YAML

        updated_xml = re.sub(param_pattern, replace_yaml_path, updated_xml)

        # ------------------------
        # 3) CSV files inside <csv_file_name> tags
        # ------------------------
        # Similar approach for CSV references. E.g.:
        #    <csv_file_name>/home/ros/.../mid360.csv</csv_file_name>
        #
        csv_pattern = r'(?<=<csv_file_name>)([^<"]*\.csv)(?=</csv_file_name>)'
        def replace_csv_path(match):
            original_path = match.group(0)
            filename = os.path.basename(original_path)
            return f'{pkg}/csv/{filename}'  # or whichever subdir you prefer

        updated_xml = re.sub(csv_pattern, replace_csv_path, updated_xml)

        return updated_xml


    def forward_spawn_request(self, namespace, data):
        """ Calls the correct `/spawn_entity` service with the reconstructed request. """

        # Clean up the XML and rewrite paths
        processed_xml = data.get("xml", "<sdf></sdf>")
        processed_xml = self.rename_files_to_be_local(processed_xml)
        processed_xml = self.rename_topics_to_be_local(processed_xml, namespace)

        # Form service request object
        request = SpawnEntity.Request()
        request.name = data.get("name", "default_name") + namespace
        request.xml = processed_xml
        request.robot_namespace = namespace
        request.initial_pose.position.x = data.get("x", 0.0)
        request.initial_pose.position.y = data.get("y", 0.0)
        request.initial_pose.position.z = data.get("z", 0.0)

        service_name = "/spawn_entity"
        client = self.create_client(SpawnEntity, service_name)

        self.check_installed_plugins(processed_xml)
        self.xml_pub.publish(String(data=request.xml))

        r2 = deepcopy(request)
        r2.xml = "removed"
        s = str(r2)
        self.meta_pub.publish(String(data=s))

        if client.wait_for_service(timeout_sec=2.0):
            future = client.call_async(request)
            future.add_done_callback(lambda fut: self.spawn_response_callback(fut, namespace))
            self.get_logger().info(f'{namespace} forwarded spawn request for {request.name} to {service_name}\n')
        else:
            self.get_logger().error(f'{namespace} SpawnEntity service unavailable at {service_name}')

    def spawn_response_callback(self, future, namespace):
        """ Logs the response from the spawn service. """
        try:
            response = future.result()
            self.get_logger().info(f'{namespace} SpawnEntity response from {namespace}: {response.success}')
        except Exception as e:
            self.get_logger().error(f'{namespace} SpawnEntity call failed in {namespace}: {str(e)}')

    # ADDED: Extended the file extensions to include .yaml and .csv
    #        and handle both quoted and unquoted references.
    def extract_file_paths(self, xml_string):
        """
        Extract file paths from XML content. Looks for:
          - Quoted references (e.g. filename="..."),
          - Unquoted references in tags (e.g. <parameters>...</parameters> or <csv_file_name>...</csv_file_name>).
        """
        pattern = (
            # 1) Match quoted filenames: filename="something.dae"
            r'(?:["\']([^"\'<>]*\.(?:dae|urdf|sdf|xacro|yaml|csv))["\'])'
            # 2) Or match unquoted references: <parameters>something.yaml</parameters>
            r'|>([^<>]*\.(?:dae|urdf|sdf|xacro|yaml|csv))<'
        )

        matches = re.findall(pattern, xml_string)
        file_paths = []
        for g1, g2 in matches:
            file_path = g1 if g1 else g2
            if file_path:
                file_paths.append(file_path)
        return file_paths

    def create_topic_name(self, file_path, namespace):
        """ Generates a topic name based on the file path. """
        file_name = os.path.basename(file_path).replace(".", "_")
        return f"{namespace}/spawn_entity/file_content/{file_name}"




######################################################


    def is_library_found(self, library_filename):
        # Given a library filename like "libgazebo_ros2_control.so",
        # strip the "lib" prefix and ".so" suffix, and use ctypes.util.find_library.
        base = library_filename.strip()
        if base.startswith("lib"):
            base = base[3:]
        if base.endswith(".so"):
            base = base[:-3]
        return ctypes.util.find_library(base) is not None

    def is_plugin_available_from_text(self, plugin_text):
        # Assume the text is in the form "package/Class". Use the package name as a rough check.
        parts = plugin_text.split('/')
        if len(parts) == 2:
            package = parts[0].strip()
            return ctypes.util.find_library(package) is not None
        return False

    def check_installed_plugins(self, xml, remove_unavailable=False):
        try:
            root = ET.fromstring(xml)
        except ET.ParseError as e:
            self.get_logger().error("XML ParseError: {}".format(e))
            return xml

        for parent in root.iter():
            # Use a copy of the child list for safe removal
            for child in list(parent):
                if child.tag == 'plugin':
                    available = False
                    plugin_id = ""
                    filename_attr = child.get("filename")
                    if filename_attr:
                        plugin_id = filename_attr.strip()
                        available = self.is_library_found(plugin_id)
                    else:
                        plugin_text = child.text.strip() if child.text else ""
                        plugin_id = plugin_text
                        available = self.is_plugin_available_from_text(plugin_text)
                    # Prefix the plugin identifier with "self." if not already present
                    if not plugin_id.startswith("self."):
                        plugin_id = "self." + plugin_id
                    diag = KeyValue()
                    diag.key = plugin_id
                    diag.value = "true" if available else "false"
                    self.all_installed_plugins.publish(diag)
                    self.get_logger().info("Plugin '{}' available: {}".format(plugin_id, diag.value))
                    if remove_unavailable and not available:
                        parent.remove(child)
        new_xml = ET.tostring(root, encoding='unicode')
        return new_xml

###############################################

def main():
    rclpy.init()
    node = TopicsToService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
