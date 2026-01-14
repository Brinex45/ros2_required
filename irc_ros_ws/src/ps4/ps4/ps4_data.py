import rclpy 
import pygame
from rclpy.node import Node 
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from irc_custom_interfaces.msg import Ps4 

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class IrcPs4(Node): 
    def __init__(self):
        super().__init__("ps4_data_node")
        self.get_logger().info("IrcPs4 node has been started.")

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.ps4_publishers = {
            0: self.create_publisher(Ps4, "ps4_data_rover", qos),
            1: self.create_publisher(Ps4, "ps4_data_arm", qos)
        }

        pygame.init()
        pygame.joystick.init()

        self.joysticks = {}
        self.detect_joysticks()

        self.timer = self.create_timer(0.05, self.loop)

    def detect_joysticks(self):
        self.joysticks.clear()
        count = pygame.joystick.get_count()

        for i in range(count):
            js = pygame.joystick.Joystick(i)
            js.init()
            self.joysticks[i] = js
            self.get_logger().info(f"Joystick {i} connected: {js.get_name()}")


    def read_ps4(self, js):
        pygame.event.pump()

        axes = [0] * 7
        buttons = [False] * 17

        axes[0] = int(js.get_axis(0) * 127) if abs(js.get_axis(0)) > 0.24 else 0
        axes[1] = int(js.get_axis(1) * -127) if abs(js.get_axis(1)) > 0.24 else 0
        axes[2] = int(js.get_axis(2) * 127) + 127
        axes[3] = int(js.get_axis(3) * 127) if abs(js.get_axis(3)) > 0.24 else 0
        axes[4] = int(js.get_axis(4) * -127) if abs(js.get_axis(4)) > 0.24 else 0
        axes[5] = int(js.get_axis(5) * 127) + 127

        for i in range(min(js.get_numbuttons(), 16)):
            buttons[i] = bool(js.get_button(i))

        for i in range(min(js.get_numhats(), 1)):
            hat_x, hat_y = js.get_hat(i)
            if hat_x == -1:
                buttons[15] = True  # Left
            elif hat_x == 1:
                buttons[16] = True  # Right
            if hat_y == 1:
                buttons[13] = True  # Up
            elif hat_y == -1:
                buttons[14] = True  # Down

        return axes, buttons
        
        
    def loop(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED or event.type == pygame.JOYDEVICEREMOVED:
                self.get_logger().warn("Joystick change detected, reloading")
                self.detect_joysticks()
                return

        for idx, js in self.joysticks.items():
            if idx not in self.ps4_publishers:
                continue  # ignore extra controllers

            axes, buttons = self.read_ps4(js)

            msg = Ps4()
            msg.ps4_data_analog = axes
            msg.ps4_data_buttons = buttons

            self.ps4_publishers[idx].publish(msg)
                
def main(args=None): 
    rclpy.init(args=args) 
    node = IrcPs4() 
    rclpy.spin(node) 
    rclpy.shutdown() 
        
if __name__ == "__main__": 
    main()