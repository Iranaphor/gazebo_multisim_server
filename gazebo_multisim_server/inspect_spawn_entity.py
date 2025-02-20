import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # Replace with your service type

class ServiceMonitor(Node):
    def __init__(self):
        super().__init__('service_monitor')
        self.srv = self.create_service(Trigger, 'monitored_service', self.service_callback)
        self.client = self.create_client(Trigger, 'original_service')

    def service_callback(self, request, response):
        self.get_logger().info('Service called!')
        if self.client.wait_for_service(timeout_sec=1.0):
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            return future.result()
        else:
            self.get_logger().error('Original service unavailable!')
            return response

def main():
    rclpy.init()
    node = ServiceMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
