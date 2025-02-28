import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import xml.etree.ElementTree as ET
import ctypes.util



class RobotConfigChecker(Node):
    def __init__(self):
        super().__init__('robot_config_checker')
        # Publisher for diagnostics array
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', self.get_qos())
        # Subscribe to the spawn_entity service content topic
        self.create_subscription(String, '/spawn_entity_srv_content', self.xml_callback, self.get_qos())
        self.get_logger().info("RobotConfigChecker started; waiting for XML on /spawn_entity_srv_content")

    def get_qos(self):
        """ Sets QoS profile to ensure reliable and latched message delivery. """
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL  # Enables latching
        )

    def is_library_found(self, lib_filename):
        # Given a library filename (e.g. "libgazebo_ros2_control.so"),
        # strip "lib" and ".so" and try to find it
        base = lib_filename.strip()
        if base.startswith("lib"):
            base = base[3:]
        if base.endswith(".so"):
            base = base[:-3]
        return ctypes.util.find_library(base) is not None

    def is_plugin_available_from_text(self, plugin_text):
        # Assume plugin text is in the form "package/Class" and use the package name as a rough check.
        parts = plugin_text.split('/')
        if len(parts) == 2:
            package = parts[0].strip()
            return ctypes.util.find_library(package) is not None
        return False

    def check_robot_configuration(self, xml, remove_unavailable=False):
        diag_list = []

        # Check if XML parses correctly
        try:
            root = ET.fromstring(xml)
        except ET.ParseError as e:
            stat = DiagnosticStatus()
            stat.name = "Robot Description"
            stat.hardware_id = ""
            stat.level = DiagnosticStatus.ERROR
            stat.message = "XML ParseError: " + str(e)
            diag_list.append(stat)
            return xml, diag_list

        # Check root element
        if root.tag != 'robot':
            stat = DiagnosticStatus()
            stat.name = "Robot Description"
            stat.hardware_id = ""
            stat.level = DiagnosticStatus.ERROR
            stat.message = "Root element is not 'robot'"
            diag_list.append(stat)
        else:
            # Check for existence of <link> and <joint> elements
            links = list(root.iter('link'))
            joints = list(root.iter('joint'))
            if not links:
                stat = DiagnosticStatus()
                stat.name = "Robot Description"
                stat.hardware_id = ""
                stat.level = DiagnosticStatus.ERROR
                stat.message = "No <link> elements found"
                diag_list.append(stat)
            else:
                stat = DiagnosticStatus()
                stat.name = "Robot Description"
                stat.hardware_id = ""
                stat.level = DiagnosticStatus.OK
                stat.message = "Found {} links".format(len(links))
                diag_list.append(stat)
            if not joints:
                stat = DiagnosticStatus()
                stat.name = "Robot Description"
                stat.hardware_id = ""
                stat.level = DiagnosticStatus.ERROR
                stat.message = "No <joint> elements found"
                diag_list.append(stat)
            else:
                stat = DiagnosticStatus()
                stat.name = "Robot Description"
                stat.hardware_id = ""
                stat.level = DiagnosticStatus.OK
                stat.message = "Found {} joints".format(len(joints))
                diag_list.append(stat)

        # Check <ros2_control> sections
        for rc in root.iter('ros2_control'):
            rc_name = rc.get('name', 'unknown')
            # Check that there is a <hardware> child with a plugin element
            hardware = rc.find('hardware')
            if hardware is None:
                stat = DiagnosticStatus()
                stat.name = "ros2_control " + rc_name
                stat.hardware_id = ""
                stat.level = DiagnosticStatus.ERROR
                stat.message = "Missing <hardware> element"
                diag_list.append(stat)
            else:
                plugin_elem = hardware.find('plugin')
                if plugin_elem is None:
                    stat = DiagnosticStatus()
                    stat.name = "ros2_control " + rc_name
                    stat.hardware_id = ""
                    stat.level = DiagnosticStatus.ERROR
                    stat.message = "No plugin declared in <hardware>"
                    diag_list.append(stat)
                else:
                    # Check plugin availability (as in our previous function)
                    available = False
                    plugin_id = ""
                    filename_attr = plugin_elem.get("filename")
                    if filename_attr:
                        plugin_id = filename_attr.strip()
                        available = self.is_library_found(plugin_id)
                    else:
                        plugin_text = plugin_elem.text.strip() if plugin_elem.text else ""
                        plugin_id = plugin_text
                        available = self.is_plugin_available_from_text(plugin_text)
                    if not plugin_id.startswith("self."):
                        plugin_id = "self." + plugin_id
                    stat = DiagnosticStatus()
                    stat.name = "Plugin " + plugin_id
                    stat.hardware_id = plugin_id
                    if available:
                        stat.level = DiagnosticStatus.OK
                        stat.message = "Hardware plugin available"
                    else:
                        stat.level = DiagnosticStatus.ERROR
                        stat.message = "Hardware plugin not available"
                    kv = KeyValue()
                    kv.key = "Availability"
                    kv.value = "true" if available else "false"
                    stat.values.append(kv)
                    diag_list.append(stat)
                    if remove_unavailable and not available:
                        hardware.remove(plugin_elem)

                # Check each <joint> element inside this ros2_control element
                for joint in rc.iter('joint'):
                    joint_name = joint.get('name', 'unknown_joint')
                    cmd_intfs = list(joint.iter('command_interface'))
                    state_intfs = list(joint.iter('state_interface'))
                    if not cmd_intfs or not state_intfs:
                        stat = DiagnosticStatus()
                        stat.name = "ros2_control joint " + joint_name
                        stat.hardware_id = joint_name
                        stat.level = DiagnosticStatus.ERROR
                        msg = "Joint missing "
                        if not cmd_intfs:
                            msg += "command_interface; "
                        if not state_intfs:
                            msg += "state_interface; "
                        stat.message = msg
                        diag_list.append(stat)
                    else:
                        stat = DiagnosticStatus()
                        stat.name = "ros2_control joint " + joint_name
                        stat.hardware_id = joint_name
                        stat.level = DiagnosticStatus.OK
                        stat.message = "Interfaces OK ({} command, {} state)".format(len(cmd_intfs), len(state_intfs))
                        diag_list.append(stat)

                # Check for some expected parameters inside <hardware>
                expected_params = ["device", "baud_rate", "timeout", "loop_rate"]
                for param in expected_params:
                    found = False
                    for p in hardware.iter('param'):
                        if p.get('name', '').strip() == param:
                            found = True
                            break
                    stat = DiagnosticStatus()
                    stat.name = "ros2_control hardware param " + param
                    stat.hardware_id = rc_name
                    if found:
                        stat.level = DiagnosticStatus.OK
                        stat.message = "Parameter '{}' found".format(param)
                    else:
                        stat.level = DiagnosticStatus.ERROR
                        stat.message = "Parameter '{}' missing".format(param)
                    diag_list.append(stat)

        return ET.tostring(root, encoding='unicode'), diag_list

    def xml_callback(self, msg):
        xml_data = msg.data
        self.get_logger().info("Received XML data on /spawn_entity_srv_content")
        modified_xml, diag_list = self.check_robot_configuration(xml_data, remove_unavailable=False)
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        diag_array.header.frame_id = "robot_config_checker"
        diag_array.status = diag_list
        self.diagnostics_pub.publish(diag_array)
        self.get_logger().info("Published DiagnosticsArray with {} statuses.".format(len(diag_list)))
        # Optionally, you could log or save modified_xml if plugins were removed.
        # self.get_logger().info("Modified XML:\n{}".format(modified_xml))

def main(args=None):
    rclpy.init(args=args)
    node = RobotConfigChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
