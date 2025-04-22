#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from bluedot import BlueDot
from signal import pause
import time
import threading

class BlueDotJoystickPublisher(Node):
    def __init__(self):
        super().__init__('bluedot_joystick_publisher')
        self.get_logger().info('Starting BlueDot Joystick Publisher...')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'bluetooth_data', 10)
        
        # Initialize state variables
        self.mode = "dpad"
        self.bd = None

        self.p = 65.0
        self.d = 0.5
        self.i = 36.5
        self.rate = 0.0
        
        # Set up the initial D-pad interface
        self.setup_dpad()
        
        self.get_logger().info("Joystick ready. Waiting for Bluetooth client...")
    
    def setup_dpad(self):
        self.get_logger().info("Setting up D-pad mode...")
        self.mode = "dpad"
        
        # Clean up existing BlueDot instance if any
        self.cleanup_bluedot()
        
        # Create a fresh BlueDot instance with grid layout
        self.bd = BlueDot(cols=3, rows=5)
        self.bd.visible = False
        
        # Configure D-pad buttons
        self.bd[0,0].visible = True
        self.bd[0,0].when_pressed = lambda: self.update_prms("p-")

        self.bd[0,1].visible = True
        self.bd[0,1].when_pressed = lambda: self.update_prms("i-")
        
        self.bd[0,2].visible = True
        self.bd[0,2].when_pressed = lambda: self.update_prms("d-")

        self.bd[2,0].visible = True
        self.bd[2,0].when_pressed = lambda: self.update_prms("p+")

        self.bd[2,1].visible = True
        self.bd[2,1].when_pressed = lambda: self.update_prms("i+")
        
        self.bd[2,2].visible = True
        self.bd[2,2].when_pressed = lambda: self.update_prms("d+")

        self.bd[1,3].visible = True
        self.bd[1,3].when_pressed = lambda: self.update_prms("r+")

        
        
        # Add a switch mode button
        self.bd[1,4].visible = True
        self.bd[1,4].color = "red"
        self.bd[1,4].when_pressed = lambda: self.switch_to_joystick()
        
        # Configure Bluetooth events
        self.bd.when_client_connects = lambda: self.get_logger().info("Client connected to D-pad")
        self.bd.when_client_disconnects = lambda: self.get_logger().info("Client disconnected from D-pad")
    
    def setup_joystick(self):
        self.get_logger().info("Setting up joystick mode...")
        self.mode = "joystick"
        
        # Clean up existing BlueDot instance if any
        self.cleanup_bluedot()
        
        # Create a new BlueDot instance
        self.bd = BlueDot()
        
        # Attach joystick handlers
        self.bd.when_moved = self.handle_joystick
        self.bd.when_released = self.joystick_released
        self.bd.when_pressed = lambda pos: self.get_logger().info("Joystick pressed")
        
        # Add a double-press handler to switch back to D-pad
        self.bd.when_double_pressed = lambda: self.switch_to_dpad()
        
        # Configure Bluetooth events
        self.bd.when_client_connects = lambda: self.get_logger().info("Client connected to joystick")
        self.bd.when_client_disconnects = lambda: self.get_logger().info("Client disconnected from joystick")
    
    def switch_to_joystick(self):
        self.get_logger().info("Switching to joystick mode")
        # Delay to allow button release to complete
        threading.Timer(0.5, self.setup_joystick).start()
    
    def switch_to_dpad(self):
        self.get_logger().info("Switching to D-pad mode")
        # Delay to allow double-press detection to complete
        threading.Timer(0.5, self.setup_dpad).start()
    
    def cleanup_bluedot(self):
        if self.bd is not None:
            try:
                self.get_logger().info("Cleaning up existing BlueDot instance")
                self.bd.stop()
                time.sleep(0.5)  # Give more time for cleanup
            except Exception as e:
                self.get_logger().warn(f"Error stopping BlueDot: {e}")
            finally:
                self.bd = None
    
    def update_prms(self, p_type):
        #self.get_logger().info(f"D-pad: {type}")
        
        if p_type == "p+":
            self.p += 1.0
            self.p = round(self.p, 2)
        elif p_type == "p-":
            self.p -= 1.0
            self.p = round(self.p, 2)
        elif p_type == "i+":
            self.i += 0.5
            self.i = round(self.i, 2)
        elif p_type == "i-":
            self.i -= 0.5
            self.i = round(self.i, 2)
        elif p_type == "d+":
            self.d += 0.1
            self.d = round(self.d, 2)
        elif p_type == "d-":
            self.d -= 0.1
            self.d = round(self.d, 2)
        elif p_type == "r+":
            self.rate += 1.0
            self.rate = round(self.rate, 2)

        msg = Float32MultiArray()
        msg.data = [self.p, self.i, self.d, self.rate]
        self.get_logger().info(f"Updated parameters: p={self.p}, i={self.i}, d={self.d}, rate={self.rate}")
        self.publisher_.publish(msg)
    
    def handle_joystick(self, pos):
        angle = float(round(pos.angle, 2)) if pos.angle is not None else 0.0
        distance = float(round(pos.distance, 2)) if pos.distance is not None else 0.0
        
        #self.get_logger().info(f"Joystick moved: angle={angle}, dist={distance}")
        
        msg = Float32MultiArray()
        msg.data = [angle, distance]
        self.publisher_.publish(msg)
    
    def joystick_released(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0]
        self.publisher_.publish(msg)
        #self.get_logger().info("Joystick released")

def main(args=None):
    rclpy.init(args=args)
    node = BlueDotJoystickPublisher()
    
    # Run ROS2 in a separate thread so BlueDot can run alongside
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()
    
    try:
        # Keep the main thread alive
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    
    # Clean up
    node.cleanup_bluedot()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()