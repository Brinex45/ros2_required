import rclpy 
from rclpy.node import Node 
from rclpy.parameter import Parameter
from irc_interfaces.msg import Ps4 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist

class DataToTwist(Node): 
    def __init__(self):
        super().__init__("ps4_data_to_twist")
        self.get_logger().info("IrcPs4 node has been started.")

        self.ps4_msg = Ps4()

        qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.sub = self.create_subscription(
            Ps4,
            "ps4_data_rover",
            self.listener_callback,
            qos
        )
        self.pub = self.create_publisher(Twist, "/unstamped_vel", 10) 

        self.declare_parameter("timer_period", 0.05)
        self.declare_parameter("multiplier", 1.0)

        self.timer_period_ = self.get_parameter("timer_period").value
        self.multiplier_ = self.get_parameter("multiplier").value

        self.number_timer_ = self.create_timer(self.timer_period_, self.publish_ps_data)

    def listener_callback(self, msg: Ps4): 
        self.ps4_msg = msg

    def publish_ps_data(self): 
        twist = Twist()
        
        twist.linear.x = (self.ps4_msg.ps4_data_analog[1] / 127.0) * self.multiplier_
        twist.angular.z = ( - self.ps4_msg.ps4_data_analog[0] / 127.0) * self.multiplier_

        self.pub.publish(twist)

                
def main(args=None): 
    rclpy.init(args=args) 
    node = DataToTwist() 
    rclpy.spin(node) 
    rclpy.shutdown() 
        
if __name__ == "__main__": 
    main()