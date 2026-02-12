import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class GpioBrain(Node):
    def __init__(self):
        super().__init__('scara_gpio_brain')

        # --- CONFIGURATION ---
        self.input_pin = "20"  # The button
        self.output_pin = "16" # The LED
        self.timeout = 1.5     # Seconds to wait before "playing back"

        # --- STATE ---
        self.click_count = 0
        self.last_click_time = 0
        self.processing = False

        # --- ROS INTERFACE ---
        # 1. Listen to the Bridge (Sensor Data)
        self.sub = self.create_subscription(
            String, 
            '/scara/gpio/in', 
            self.handle_input, 
            10
        )

        # 2. Talk to the Bridge (Commands)
        self.pub = self.create_publisher(
            String, 
            '/scara/gpio/out', 
            10
        )

        # 3. Timer to check for "silence" (End of sequence)
        self.timer = self.create_timer(0.1, self.check_sequence_end)

        self.get_logger().info("🧠 Logic Node Ready: Click GP20 to queue blinks on GP16.")

    def handle_input(self, msg):
        """
        Receives events like "20:0" (Pressed) or "20:1" (Released)
        """
        data = msg.data.strip()
        try:
            pin, state = data.split(":")
            
            # We only care about OUR input pin
            if pin == self.input_pin:
                # We only care about the PRESS event (0 = Pressed, usually)
                # Adjust '0' to '1' depending on if your switch is Normally Open/Closed
                if state == "0": 
                    self.click_count += 1
                    self.last_click_time = time.time()
                    self.processing = True
                    self.get_logger().info(f"Click! Count: {self.click_count}")

        except ValueError:
            pass

    def check_sequence_end(self):
        """
        Runs 10 times a second. Checks if the user stopped clicking.
        """
        if not self.processing: return
        if self.click_count == 0: return

        # How long has it been since the last click?
        elapsed = time.time() - self.last_click_time

        if elapsed > self.timeout:
            # User finished entering the sequence. Execute Logic!
            self.trigger_blink_sequence()
            
            # Reset
            self.click_count = 0
            self.processing = False

    def trigger_blink_sequence(self):
        """
        Constructs the command to Blink GP16 n times
        """
        n = self.click_count
        self.get_logger().info(f"🚀 Sequence Complete. Blinking {n} times...")
        
        # Command Format: "16:B:<Count>:<Speed>"
        # Example: "16:B:3:0.2" (Blink 3 times, fast)
        cmd_str = f"{self.output_pin}:B:{n}:0.2"
        
        msg = String()
        msg.data = cmd_str
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpioBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()