import rclpy
from rclpy.node import Node
from rclpy import Timer

class repub(Node):

    def __init__(self):
        self.init_node('repub')
        self.subscribers = dict()
        self.timer = Timer(self.timer_cb, 10)

    def timer_cb(self):
        # Get list of all topics
        topic_list = getalltopics()
        topic_list = [t for t in topic_list if 'spawn_entity' in t and not t != '/spawn_entity']

        # Identify each namespace and loop through the new ones
        namespaces = [t.replace('/spawn_entity', '') for t in topic_list]
        namespaces = [n for n in namespaces if n not in self.subscribers.keys()]

        # Create subscription for each subscriber
        for n in namespaces:
            self.subscribers[n] = self.create_subscription(f'{n}/spawn_entity', lambda msg, ns=n : self.spawn_cb(msg, ns), 10)

    def spawn_cb(data, ns):
        self.spawn_pub.publish(data, namespace=ns)
        # this wont work because its a service but good try from memory ;)



def main(args):
    rclpy.init()
    RP = repub()

    RP.spin()
    rclpy.destroy_all()


if __name__=='__main__':
    main()
