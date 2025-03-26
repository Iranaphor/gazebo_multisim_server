#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import subprocess

class NavSatFixPublisherViaCLI(Node):
    def __init__(self):
        super().__init__('navsatfix_publisher_via_cli')

        # Create a publisher using the NavSatFix message type
        self.pub_navsat = self.create_publisher(NavSatFix, '/datum', 10)

        # Create a timer to call the CLI command every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info("Initialized. Will check 'datum' param every 5 seconds via CLI.")

    def timer_callback(self):
        """
        Timer callback that runs `ros2 param get /navsat_transform_base datum`
        to retrieve [latitude, longitude, altitude], then publishes as NavSatFix.
        """
        cmd = ["ros2", "param", "get", "/navsat_transform_base", "datum"]
        self.get_logger().info(f"Running command: {' '.join(cmd)}")

        # Run the command, capturing stdout
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode != 0:
            # If the command failed, log and return
            self.get_logger().error(f"CLI call failed: {result.stderr.strip()}")
            return

        # Example of output:
        # "Double values are: array('d', [53.268642038, -0.524509505881, 0.0])"
        output = result.stdout.strip()
        self.get_logger().info(f"CLI output: {output}")

        # Parse the output for the array of doubles
        prefix = "Double values are: array('d', ["
        suffix = "])"

        if prefix in output and suffix in output:
            # Extract just the content inside the brackets
            start_idx = output.find(prefix) + len(prefix)
            end_idx = output.rfind(suffix)
            if start_idx < end_idx:
                numeric_str = output[start_idx:end_idx].strip()
                # numeric_str might look like "53.268642038, -0.524509505881, 0.0"

                # Convert to a Python list of floats
                try:
                    values_list = [float(x.strip()) for x in numeric_str.split(",")]
                    if len(values_list) != 3:
                        self.get_logger().warning(
                            f"Expected 3 values [lat, lon, alt], but got {len(values_list)}."
                        )
                        return

                    # Create and populate a NavSatFix message
                    navsat_msg = NavSatFix()
                    navsat_msg.header.stamp = self.get_clock().now().to_msg()
                    navsat_msg.header.frame_id = "gps"  # frame ID, set as appropriate

                    # For a standard GPS fix: 
                    #   values_list[0] -> latitude
                    #   values_list[1] -> longitude
                    #   values_list[2] -> altitude
                    navsat_msg.latitude = values_list[0]
                    navsat_msg.longitude = values_list[1]
                    navsat_msg.altitude = values_list[2]

                    # Some fields you might want to set:
                    navsat_msg.status.status = NavSatStatus.STATUS_FIX
                    navsat_msg.status.service = NavSatStatus.SERVICE_GPS

                    # Covariance type can be UNKNOWN or any supported
                    navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    # If you have covariance data, you can set it here
                    # navsat_msg.position_covariance = [0.0]*9  

                    # Publish
                    self.pub_navsat.publish(navsat_msg)
                    self.get_logger().info(
                        f"Published NavSatFix -> lat={values_list[0]}, "
                        f"lon={values_list[1]}, alt={values_list[2]}"
                    )

                except ValueError:
                    self.get_logger().error("Failed converting CLI output to floats.")
            else:
                self.get_logger().warning("Unexpected format: could not slice output correctly.")
        else:
            self.get_logger().warning("Output not in the expected 'Double values are...' format.")

def main(args=None):
    rclpy.init(args=args)
    node = NavSatFixPublisherViaCLI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
