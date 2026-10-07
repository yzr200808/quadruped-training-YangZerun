import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32


class XboxSendNode(Node):

    def __init__(self):
        super().__init__("xbox_send_node")
        key_topic = self.declare_parameter("key_topic", "/key_input").value
        joy_topic = self.declare_parameter("joy_topic", "/joy").value
        self.cmd_pub = self.create_publisher(Int32, key_topic, 1)
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.callback, 10)
        self.last_a = 0
        self.last_b = 0
        self.get_logger().info(
            "手柄节点已启动：订阅 %s，发布 %s（A 键=松弛 9，B 键=站姿 8）"
            % (joy_topic, key_topic)
        )

    def callback(self, msg: Joy):
        if len(msg.buttons) < 2:
            return
        a_now = msg.buttons[0]
        b_now = msg.buttons[1]
        if a_now == 1 and self.last_a == 0:
            self.cmd_pub.publish(Int32(data=9))
            self.get_logger().info("A 键按下：发送 9（电机松弛）")
        elif b_now == 1 and self.last_b == 0:
            self.cmd_pub.publish(Int32(data=8))
            self.get_logger().info("B 键按下：发送 8（站姿）")
        self.last_a = a_now
        self.last_b = b_now

def main(args=None):
    rclpy.init(args=args)
    node = XboxSendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
