import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from bluedot import BlueDot
import time

class BlueDotPIDPublisher(Node):
    def __init__(self):
        super().__init__('bluedot_pid_publisher')
        self.get_logger().info('Starting BlueDot PID Publisher...')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'pid_params', 10)

        # Initialize BlueDot with specific layout
        self.get_logger().info('Initializing BlueDot. Waiting for connection...')
        self.bd = BlueDot()
        self.bd.resize(2, 8)

        # Set up connection callbacks
        self.bd.when_client_connects = self.handle_connect
        self.bd.when_client_disconnects = self.handle_disconnect

        # Initialize values
        self.p = 0.5
        self.i = 0.05
        self.alpha = 0.2
        self.rate = 1.6

        self.po = 2.0
        self.io = 1.8
        self.d = 0.9
        self.e = 0.002

        # Wait for the grid to be ready
        self.get_logger().info('Setting up button grid...')
        time.sleep(1)  # Give BlueDot time to set up the grid

        try:
            # Assign button press handlers
            self.bd[1, 0].when_pressed = self.increase_p
            self.bd[1, 1].when_pressed = self.increase_i
            self.bd[1, 2].when_pressed = self.increase_alpha
            self.bd[1, 3].when_pressed = self.increase_rate
            self.bd[0, 0].when_pressed = self.decrease_p
            self.bd[0, 1].when_pressed = self.decrease_i
            self.bd[0, 2].when_pressed = self.decrease_alpha
            self.bd[0, 3].when_pressed = self.decrease_rate

            self.bd[1, 4].when_pressed = self.increase_po
            self.bd[0, 4].when_pressed = self.decrease_po
            self.bd[1, 5].when_pressed = self.increase_d
            self.bd[0, 5].when_pressed = self.decrease_d
            self.bd[1, 6].when_pressed = self.increase_e
            self.bd[0, 6].when_pressed = self.decrease_e
            self.bd[1, 7].when_pressed = self.increase_io
            self.bd[0, 7].when_pressed = self.decrease_io

            # Set button colors
            self.bd[0, 0].color = "red" 
            self.bd[0, 1].color = "green"  
            self.bd[0, 2].color = "blue" 
            self.bd[0, 3].color = "black"   
            self.bd[0, 4].color = "red"   
            self.bd[0, 5].color = "green"  
            self.bd[0, 6].color = "blue"
            self.bd[0, 7].color = "black"

            self.bd[1, 0].color = "red" 
            self.bd[1, 1].color = "green" 
            self.bd[1, 2].color = "blue" 
            self.bd[1, 3].color = "black" 
            self.bd[1, 4].color = "red" 
            self.bd[1, 5].color = "green" 
            self.bd[1, 6].color = "blue" 
            self.bd[1, 7].color = "black" 

            self.get_logger().info('Grid setup complete')
            self.get_logger().info(f'Initial values: P={self.p}, I={self.i}, ALPHA={self.alpha}, RATE={self.rate}, PO={self.po}, D={self.d}, E={self.e}, IO={self.io}')

        except Exception as e:
            self.get_logger().error(f'Failed to set up button grid: {str(e)}')
            raise

    def handle_connect(self):
        """Callback when a client connects"""
        self.get_logger().info('Bluetooth client connected!')

    def handle_disconnect(self):
        """Callback when a client disconnects"""
        self.get_logger().warn('Bluetooth client disconnected!')

    def increase_p(self):
        self.p += 0.1
        self.publish_pid()

    def decrease_p(self):
        self.p = round(max(0, self.p - 0.1), 2)
        self.publish_pid()

    def increase_i(self):
        self.i += 0.01
        self.publish_pid()

    def decrease_i(self):
        self.i = round(max(0, self.i - 0.01), 2)
        self.publish_pid()

    def increase_alpha(self):
        self.alpha += 0.01
        self.publish_pid()

    def decrease_alpha(self):
        self.alpha = round(max(0, self.alpha - 0.011), 2)
        self.publish_pid()

    def increase_rate(self):
        self.rate += 0.1
        self.publish_pid()

    def decrease_rate(self):
        self.rate = round(max(0, self.rate - 0.1), 2)
        self.publish_pid()

    def increase_po(self):
        self.po += 0.1
        self.publish_pid()

    def decrease_po(self):
        self.po = round(max(0, self.po - 0.1), 2)
        self.publish_pid()

    def increase_d(self):
        self.d += 0.01
        self.publish_pid()

    def decrease_d(self):
        self.d = round(max(0, self.d - 0.01), 2)
        self.publish_pid()

    def increase_e(self):
        self.e += 0.001
        self.publish_pid()

    def decrease_e(self):   
        self.e = round(max(0, self.e - 0.001), 3)
        self.publish_pid()

    def increase_io(self):
        self.io += 0.1
        self.publish_pid()

    def decrease_io(self):
        self.io = round(max(0, self.io - 0.1), 2)
        self.publish_pid()

    def publish_pid(self):
        """Publish the updated PID values and update the label."""
        msg = Float32MultiArray()
        msg.data = [self.p, self.i, self.alpha, self.rate, self.po, self.d, self.e, self.io]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Updated PARAMS: P={round(self.p, 2)}, I={round(self.i, 2)}, ALPHA={round(self.alpha, 2)}, RATE={round(self.rate, 2)}, PO={round(self.po, 2)}, D={round(self.d, 2)}, E={round(self.e, 3)}, IO={round(self.io, 2)}')

def main(args=None):
    rclpy.init(args=args)
    node = BlueDotPIDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
