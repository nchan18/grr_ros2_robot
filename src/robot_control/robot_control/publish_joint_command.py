import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class PublishJointCmd(Node):

    def __init__(self):
        super().__init__('publish_joint_commands')
        self.publisher_ = self.create_publisher(JointState, 'isaac_joint_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        velocity_cmds = JointState()
        
        velocity_cmds.name = [
            'front_left_mecanum_joint', 
            'front_right_mecanum_joint',
            'rear_left_mecanum_joint',
            'rear_right_mecanum_joint']
        velocity_cmds.velocity = [ 15.0, 15.0, 15.0, 15.0 ]

        self.publisher_.publish(velocity_cmds)
        self.get_logger().info('Publishing: ...')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = PublishJointCmd()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()