import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, Int32


class FizzBuzzPubSub(Node):
    def __init__(self):
        super().__init__('fizzbuzz_pub_sub')
        self.pub = self.create_publisher(String, 'fizzbuzz_msg', 10)
        self.sub = self.create_subscription(Int32, 'number', self.callback, 10)        

    def callback(self, sub_msg):
        pub_msg = String()
        self.get_logger().info(f'サブスクライブ:{sub_msg.data}')
        if sub_msg.data % 3 == 0 and sub_msg.data % 5 == 0:
            pub_msg.data = f'FizzBuzz'
        elif sub_msg.data % 3 == 0:
            pub_msg.data = f'Fizz'
        elif sub_msg.data % 5 == 0:
            pub_msg.data = f'Buzz'
        else:
            pub_msg.data = f'{sub_msg.data}'

        self.pub.publish(pub_msg)
        self.get_logger().info(f'パブリッシュ:{pub_msg.data}')


def main():
    rclpy.init()
    node = FizzBuzzPubSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
