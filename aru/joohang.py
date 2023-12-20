import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotCommandSubscriber(Node):
    def __init__(self):
        super().__init__('robot_command_subscriber')
        self.subscription = self.create_subscription(
            String, 
            'robot_command', 
            self.command_callback,
            10)
        self.subscription

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

def main(args=None):
    rclpy.init(args=args)
    robot_command_subscriber = RobotCommandSubscriber()

    try:
        rclpy.spin(robot_command_subscriber)
    except KeyboardInterrupt:
        pass

    robot_command_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
