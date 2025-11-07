#!/usr/bin/env python3

import rclpy
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl

class PerceptionController(BaseController):
    def __init__(self, node_name: str = "perception_controller") -> None:
        super().__init__(node_name)
        self.declare_parameter("active", True)
        self.declare_parameter("stop_time", 0)
        self.flag = 1
        self.end_time = 0

    @property
    def active(self) -> bool:
        return self.get_parameter("active").value

    def compute_control(self):     
        new_control = TurtleBotControl()
        if self.active:
            new_control.omega = 0.5
        else:
            if self.flag == 1: # Initial Stop
                new_control.omega = 0.0

                initial_time = self.get_clock().now().nanoseconds / 1e9
                self.end_time = initial_time + 500000000  # 5 seconds in nanoseconds
                self.flag = 0
                return new_control
            
            if self.flag == 0:
                current_time = self.get_clock().now().nanoseconds / 1e9
                difference = self.end_time - current_time
                if difference <= 5:
                    self.set_parameters([rclpy.Parameter("active", True)])
                    self.flag = 1
                    new_control.omega = 0.5
                    return new_control
        return new_control
    
if __name__ == "__main__":
    rclpy.init()
    node = PerceptionController()
    rclpy.spin(node)
    rclpy.shutdown()

    
