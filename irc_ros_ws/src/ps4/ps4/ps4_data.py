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

        self.data = self.create_publisher(Ps4, "ps4_data_", 10)
        self.data_1 = self.create_publisher(Ps4, "ps4_data_arm", 10)

        self.declare_parameter("timer_period", 0.05)
        self.timer_period_ = self.get_parameter("timer_period").value
        self.number_timer_ = self.create_timer(self.timer_period_, self.publish_ps_data)

        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        while count < 1:
            # self.get_logger().warn("No joystick found! Please connect your PS4 controller.")
            pygame.time.wait(1000)
            count = pygame.joystick.get_count()

        self.joysticks = [pygame.joystick.Joystick(i) for i in range(count)]
        for js in self.joysticks:
            js.init()
            self.get_logger().info(f"Using joystick: {js.get_name()}")


    def ps_data(self, joystick_id):
        pygame.event.pump()

        js = self.joysticks[joystick_id]

        axes = [0] * 7
        buttons = [0] * 16

        axes[0] = (int(round(js.get_axis(0)*127))) if abs(js.get_axis(0)) > 0.24 else 0
        axes[1] = (int(round(js.get_axis(1)*(-127)))) if abs(js.get_axis(1)) > 0.24 else 0

        axes[2] = (int(round(js.get_axis(2)*127))) + 127 if abs((int(round(js.get_axis(2)*127))) + 127) > 20 else 0
        
        axes[3] = (int(round(js.get_axis(3)*127))) if abs(js.get_axis(3)) > 0.24 else 0
        axes[4] = (int(round(js.get_axis(4)*(-127)))) if abs(js.get_axis(4)) > 0.24 else 0
        
        axes[5] = (int(round(js.get_axis(5)*127))) + 127 if abs((int(round(js.get_axis(5)*127))) + 127) > 20 else 0

        buttons = [bool(js.get_button(i)) for i in range(js.get_numbuttons())]

        return axes, buttons
        
        
    def publish_ps_data(self): 
        axes_c, buttons_c = self.ps_data(0)
        axes_a, buttons_a = self.ps_data(1)

        msg_c = Ps4()
        msg_a = Ps4()

        msg_c.ps4_data_analog = axes_c[:7] + [0.0] * (7 - len(axes_c))   # pad if fewer than 7
        msg_c.ps4_data_buttons = buttons_c[:16] + [False] * (16 - len(buttons_c))  # pad if fewer than 16

        msg_a.ps4_data_analog = axes_a[:7] + [0.0] * (7 - len(axes_a))   # pad if fewer than 7
        msg_a.ps4_data_buttons = buttons_a[:16] + [False] * (16 - len(buttons_a))  # pad if fewer than 16

        self.data.publish(msg_c) 
        self.data_1.publish(msg_a)
        # self.get_logger().info(f'Published Ps4 message: {msg_a}')

                
def main(args=None): 
    rclpy.init(args=args) 
    node = IrcPs4() 
    rclpy.spin(node) 
    rclpy.shutdown() 
        
if __name__ == "__main__": 
    main()