import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty  # Adjust the service type as needed

class DynamicServiceListener(Node):
    def __init__(self):
        super().__init__('dynamic_service_listener')
        self.clients = {}
        # Timer to periodically check for new services
        self.timer = self.create_timer(2.0, self.discover_services)  # Check every 2 seconds

    def discover_services(self):
        # Get list of all services
        all_services = self.get_service_names_and_types()
        for service_name, service_types in all_services:
            if service_name.endswith('/spawn_entity') and service_name not in self.clients:
                # Check if the service type is correct; adjust 'std_srvs/srv/Empty' as needed
                if 'std_srvs/srv/Empty' in service_types:
                    self.create_client_for_service(service_name)

    def create_client_for_service(self, service_name):
        # Create client for the discovered service
        client = self.create_client(Empty, service_name)
        self.clients[service_name] = client
        self.get_logger().info(f'Created client for service: {service_name}')

        # Optional: You can also implement a waiting mechanism here if the client needs to wait for the service to be available

    def handle_spawn_request(self, service_name, request):
        # Process the XML part of the service message
        processed_xml = self.process_xml(request.xml)

        # Construct and send a new service request
        service_request = Empty.Request()  # Adjust based on actual data structure
        future = self.clients[service_name].call_async(service_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result received from {service_name}')
        else:
            self.get_logger().error(f'Exception while calling {service_name}')

    def process_xml(self, xml):
        # Placeholder for XML processing logic
        return xml

def main(args=None):
    rclpy.init(args=args)
    dynamic_service_listener = DynamicServiceListener()
    rclpy.spin(dynamic_service_listener)
    dynamic_service_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
