#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

import argparse
import os

class SpawnEntityFromXML(Node):
    def __init__(self, namespace: str, name: str, xml_filepath: str):
        super().__init__('spawn_entity_from_xml')

        self.xml_filepath = xml_filepath
        self.namespace = namespace
        self.entity_name = name

        # Attempt to load XML content
        self.xml_content = None
        try:
            with open(self.xml_filepath, 'r') as f:
                self.xml_content = f.read()
        except Exception as e:
            self.get_logger().error(f"Could not read file '{self.xml_filepath}': {e}")
            # If we can't read the file, no point continuing
            rclpy.shutdown()
            return

        self.get_logger().info(f"Successfully read XML from: {self.xml_filepath}")

        # Create a client for the '/spawn_entity' service
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        # Prepare the SpawnEntity request
        self.req = SpawnEntity.Request()
        self.req.xml = self.xml_content
        self.req.name = self.entity_name
        self.req.robot_namespace = self.namespace
        self.req.reference_frame = "world"

        # Call the service once
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.spawn_entity_response_callback)

    def spawn_entity_response_callback(self, future):
        """Handle the response from the SpawnEntity service."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Entity spawned successfully!")
            else:
                self.get_logger().error(
                    f"Failed to spawn entity: {response.status_message}"
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        # Shutdown after we receive the response
        self.get_logger().info("Shutting down node after service call.")
        rclpy.shutdown()

def main():
    rclpy.init()

    parser = argparse.ArgumentParser(
        description='Spawn an entity in Gazebo from an XML description.')

    parser.add_argument(
        '-ns', '--namespace',
        type=str,
        default='',
        help='Namespace for the spawned entity.'
    )

    parser.add_argument(
        '-n', '--name',
        type=str,
        default='spawned_entity',
        help='Name of the entity being spawned.'
    )

    parser.add_argument(
        '-f', '--filepath',
        type=str,
        required=True,
        help='Path to the XML file describing the entity.'
    )

    args = parser.parse_args()
    node = SpawnEntityFromXML(args.namespace, args.name, args.filepath)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
