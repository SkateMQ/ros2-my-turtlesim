#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from example_interfaces.msg import Int64

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.declare_parameter("number", 2)
        self.declare_parameter("timer_period", 1.0)
        self.numer_ = self.get_parameter("number").value
        self.timer_period_ = self.get_parameter("timer_period").get_parameter_value().double_value
        self.add_post_set_parameters_callback(self.parameters_callback)

        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(self.timer_period_, self.publish_numbers)
        self.get_logger().info("Number Publisher has been started.")

    def publish_numbers(self):
        msg = Int64()
        msg.data = self.numer_
        self.publisher_.publish(msg)

    def parameters_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == "number":
                self.numer_ = param.value


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
