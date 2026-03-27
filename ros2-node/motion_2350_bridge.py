import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String

class Motion2350Bridge(Node):
    def __init__(self):
        super().__init__('motion_2350_bridge')
        
        # --- SETTINGS ---
        self.port = '/dev/tty.usbmodem1101' # Update as needed
        self.baud = 115200
        
        # --- TOPICS ---
        # 1. Output to Board (e.g. "16:1", "16:0")
        self.sub_out = self.create_subscription(
            String, '/scara/gpio/out', self.cb_gpio_out, 10)

        # 2. Input from Board (e.g. "16:1")
        self.pub_in = self.create_publisher(
            String, '/scara/gpio/in', 10)

        # 3. Config (e.g. "list", "16:in")
        self.sub_config = self.create_subscription(
            String, '/scara/gpio/config', self.cb_gpio_config, 10)

        self.ser = None
        self.connect_serial()
        
        # Timer for reading serial input from board
        self.timer = self.create_timer(0.01, self.serial_read_loop)
        
        self.get_logger().info("SCARA GPIO Node Ready.")

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            time.sleep(1.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Connected to {self.port}")
        except Exception as e:
            self.get_logger().error(f"Serial Connect Error: {e}")

    # --- CALLBACK: /scara/gpio/out ---
    def cb_gpio_out(self, msg):
        """
        Controls outputs.
        Formats:
          "16:1"         -> Turn GP16 ON
          "16:0"         -> Turn GP16 OFF
          "16:B:5:0.2"   -> Blink GP16, 5 times, 0.2s interval
          "7:T:1.0:0.5"  -> Timed Servo on GP7, Throttle 1.0, 0.5 sec
          "7:S:0.0"      -> Continuous Servo on GP7, Stop (Throttle 0.0)
        """
        data = msg.data.strip()
        parts = data.split(":")
        
        if not self.ser: return

        try:
            pin = parts[0]
            
            # Check for Blink command (B:Pin:Count:Speed)
            if len(parts) >= 4 and parts[1].upper() == 'B':
                cmd = f"B:{pin}:{parts[2]}:{parts[3]}\n"
                
            # Check for Timed Servo command (T:Pin:Throttle:Duration)
            elif len(parts) >= 4 and parts[1].upper() == 'T':
                cmd = f"T:{pin}:{parts[2]}:{parts[3]}\n"
                
            # Check for Continuous Servo command (S:Pin:Throttle)
            elif len(parts) >= 3 and parts[1].upper() == 'S':
                cmd = f"S:{pin}:{parts[2]}\n"
                
            # Standard On/Off (O:Pin:Value)
            else:
                val = int(parts[1])
                cmd = f"O:{pin}:{val}\n"
            
            self.ser.write(cmd.encode())
            self.get_logger().info(f"Tx Output: {cmd.strip()}")
            
        except Exception as e:
            self.get_logger().error(f"Invalid Output Format '{data}': {e}")

    # --- CALLBACK: /scara/gpio/config ---
    def cb_gpio_config(self, msg):
        """
        Configures modes.
        Formats:
          "list"   -> Scans available pins
          "16:in"  -> Sets GP16 as Input (Sensor)
        """
        cmd = msg.data.lower().strip()
        if not self.ser: return

        if cmd == "list":
            self.ser.write(b"L\n")
            self.get_logger().info("Tx Config: Requested Pin List...")
        
        elif ":in" in cmd:
            # Format "16:in"
            try:
                pin = cmd.split(":")[0]
                self.ser.write(f"I:{pin}\n".encode())
                self.get_logger().info(f"Tx Config: Set GP{pin} to INPUT")
            except:
                self.get_logger().error("Bad Config Format. Use '16:in'")

    # --- LOOP: Read Serial ---
    def serial_read_loop(self):
        if not self.ser: return
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue
                
                # 1. Hardware Event (Switch changed)
                if line.startswith("E:"):
                    # Protocol: E:Pin:Value (0 or 1)
                    parts = line.split(":")
                    if len(parts) == 3:
                        pin = parts[1]
                        val = parts[2] # "1" or "0"
                        
                        # Publish to /scara/gpio/in
                        msg = String()
                        msg.data = f"{pin}:{val}"
                        self.pub_in.publish(msg)
                        
                        # Log it
                        state = "RELEASED" if val == "1" else "PRESSED"
                        self.get_logger().info(f"Rx Input: GP{pin} is {state} (Pub: {msg.data})")

                # 2. Pin List Response
                elif line.startswith("PINS:"):
                    self.get_logger().info(f"✅ BOARD INFO: {line}")

                # 3. Acks/Errors
                elif line.startswith("ACK") or line.startswith("ERR"):
                    self.get_logger().debug(f"Board: {line}")

        except Exception as e:
            self.get_logger().error(f"Serial Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Motion2350Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node.ser: node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()