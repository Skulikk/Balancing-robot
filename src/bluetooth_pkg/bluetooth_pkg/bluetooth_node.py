import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from bluedot import BlueDot
from signal import pause
import time
import threading


class BlueDotJoystickPublisher(Node):
    def __init__(self):
        super().__init__('bluedot_joystick_publisher')
        self.get_logger().info('Starting BlueDot Joystick Publisher...')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'bluetooth_data', 10)

        # Subscriptions to status topics
        self.create_subscription(Bool, 'IMU_status', self.imu_status_callback, 10)
        self.create_subscription(Bool, 'encoder_status', self.encoder_status_callback, 10)
        self.create_subscription(Bool, 'ultra_s_status', self.ultra_s_status_callback, 10)

        self.mode = "dpad"
        self.bd = None

        self.imu_status = False
        self.encoder_status = False
        self.ultra_s_status = False
        self.balancing_status = False
        self.auto_status = False

        self.setup_dpad()
        self.get_logger().info("Joystick ready. Waiting for Bluetooth client...")

    def setup_dpad(self):
        self.mode = "dpad"

        self.cleanup_bluedot()
        self.bd = BlueDot(cols=3, rows=5)
        self.bd.visible = False

        self.bd[0, 0].visible = True
        self.bd[0, 0].color = 'green' if self.imu_status else 'red'

        self.bd[1, 0].visible = True
        self.bd[1, 0].color = 'green' if self.encoder_status else 'red'

        self.bd[2, 0].visible = True
        self.bd[2, 0].color = 'green' if self.ultra_s_status else 'red'

        self.bd[1, 1].visible = True
        self.bd[1, 1].color = "black"
        self.bd[1, 1].when_pressed = lambda: self.restart()

        self.bd[1, 2].visible = True
        self.bd[1, 2].color = "green" if self.balancing_status else 'red'
        self.bd[1, 2].when_pressed = lambda: self.balance()

        self.bd[1, 3].visible = True
        self.bd[1, 3].color = "green" if self.auto_status else 'yellow'
        self.bd[1, 3].when_pressed = lambda: self.auto()

        self.bd[1, 4].visible = True
        self.bd[1, 4].color = "black"
        self.bd[1, 4].when_pressed = lambda: self.switch_to_joystick()

    def imu_status_callback(self, msg):
        self.imu_status = msg.data
        if self.bd and self.mode == "dpad":
           self.bd[0,0].color = 'green' if msg.data else 'red'

    def encoder_status_callback(self, msg):
        self.encoder_status = msg.data
        if self.bd and self.mode == "dpad":
            self.bd[1,0].color = 'green' if msg.data else 'red'

    def ultra_s_status_callback(self, msg):
        self.ultra_s_status = msg.data
        if self.bd and self.mode == "dpad":
            self.bd[2,0].color = 'green' if msg.data else 'red'

    def setup_joystick(self):
        self.mode = "joystick"

        self.cleanup_bluedot()
        self.bd = BlueDot()

        self.bd.when_moved = self.handle_joystick
        self.bd.when_released = self.joystick_released
        self.bd.when_double_pressed = lambda: self.switch_to_dpad()

    def switch_to_joystick(self):
        threading.Timer(0.5, self.setup_joystick).start()

    def restart(self):
        self.get_logger().info("Restart")

    def balance(self):
        msg = Float32MultiArray()
        if self.balancing_status:
            msg.data = [250.0, 0.0]
            self.balancing_status = False
            self.bd[1,2].color = 'red'
        else:
            msg.data = [250.0, 1.0]
            self.balancing_status = True
            self.bd[1,2].color = 'green'
        self.publisher_.publish(msg)

    def auto(self):
        msg = Float32MultiArray()
        if self.auto_status:
            msg.data = [260.0, 0.0]
            self.auto_status = False
            self.bd[1,3].color = 'yellow'
        else:
            msg.data = [260.0, 1.0]
            self.auto_status = True
            self.bd[1,3].color = 'green'
        self.publisher_.publish(msg)

    def switch_to_dpad(self):
        threading.Timer(0.5, self.setup_dpad).start()

    def cleanup_bluedot(self):
        if self.bd is not None:
            try:
                self.bd.stop()
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().warn(f"Error stopping BlueDot: {e}")
            finally:
                self.bd = None

    def handle_joystick(self, pos):
        angle = float(round(pos.angle, 2)) if pos.angle is not None else 0.0
        distance = float(round(pos.distance, 2)) if pos.distance is not None else 0.0
        msg = Float32MultiArray()
        msg.data = [angle, distance]
        self.publisher_.publish(msg)

    def joystick_released(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BlueDotJoystickPublisher()

    executor_thread = threading.Thread(target=rclpy.spin, args=(node,))
    executor_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.cleanup_bluedot()
        rclpy.shutdown()
        executor_thread.join()
        node.destroy_node()


if __name__ == '__main__':
    main()
