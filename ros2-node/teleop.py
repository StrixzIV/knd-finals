#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import termios
import tty
import time

# --- KINEMATIC CONFIGURATION ---
# Format: [PUL_PIN, DIR_PIN, STEPS_PER_UNIT, LIMIT_MIN, LIMIT_MAX, CURRENT_POS, READY_TIME]
# Unit is Degrees for Revolute, Centimeters for Prismatic.
MOTORS = {
    'M1': {'pul': 0, 'dir': 1, 'steps_per_unit': 1000 / 45.0, 'limit': 80.0, 'pos': 0.0, 'name': 'Base (J1)', 'ready_time': 0.0},
    'M2': {'pul': 2, 'dir': 3, 'steps_per_unit': 1000 / 2.0,  'limit': 13.0, 'pos': 0.0, 'name': 'Z-Axis (J2)', 'ready_time': 0.0},
    'M3': {'pul': 4, 'dir': 5, 'steps_per_unit': 1000 / 60.0, 'limit': 80.0, 'pos': 0.0, 'name': 'Elbow (J3)', 'ready_time': 0.0},
    'M4': {'pul': 6, 'dir': 7, 'steps_per_unit': 100 / 15.0,  'limit': 80.0, 'pos': 0.0, 'name': 'Wrist (J4)', 'ready_time': 0.0}
}

# --- KEYBOARD MAPPING ---
# Key: (Motor, Direction_Multiplier)
KEY_BINDINGS = {
    'q': ('M1', 1),  'a': ('M1', -1),
    'w': ('M2', 1),  's': ('M2', -1),
    'e': ('M3', 1),  'd': ('M3', -1),
    'r': ('M4', 1),  'f': ('M4', -1),
}

INSTRUCTIONS = """
SCARA RPRR Teleop Controller
---------------------------
Make sure the arm is manually homed (Straight out on Y-axis).

Moving Joints (Degrees / cm):
   Q / A : Base J1  (+ / -)
   W / S : Z-Axis   (+ / -)
   E / D : Elbow J3 (+ / -)
   R / F : Wrist J4 (+ / -)

Settings:
   + / - : Increase/Decrease Jog Step Size
   CTRL-C to quit.
"""

class ScaraTeleop(Node):
    def __init__(self):
        super().__init__('scara_teleop')
        self.pub_out = self.create_publisher(String, '/scara/gpio/out', 10)
        
        # Default movement step sizes per keystroke
        self.jog_deg = 5.0  # Move 5 degrees per keystroke
        self.jog_cm = 0.5   # Move 0.5 cm per keystroke
        self.pulse_speed = 0.001 # 1ms half-period (500 steps/sec)

        self.get_logger().info("SCARA Teleop Node Started.")
        print(INSTRUCTIONS)
        self.print_status()

    def print_status(self):
        status = (f"\rJog Size: {self.jog_deg:.1f}° / {self.jog_cm:.1f}cm | "
                  f"Pos -> M1:{MOTORS['M1']['pos']:.1f}° "
                  f"M2:{MOTORS['M2']['pos']:.1f}cm "
                  f"M3:{MOTORS['M3']['pos']:.1f}° "
                  f"M4:{MOTORS['M4']['pos']:.1f}°")
        # Flush standard out so the terminal updates smoothly, padding with spaces clears artifact text
        print(f"{status:<85}", end='', flush=True)

    def move_motor(self, motor_key, direction):
        m = MOTORS[motor_key]
        now = time.time()
        
        # 1. Rate Limiter: Drop OS auto-repeat keystrokes if the motor is still physically executing the last command.
        if now < m['ready_time']:
            return
            
        # Determine movement amount based on unit type
        is_linear = (motor_key == 'M2')
        move_amt = self.jog_cm if is_linear else self.jog_deg
        move_amt *= direction
        
        # Calculate new position and check limits
        new_pos = m['pos'] + move_amt
        if abs(new_pos) > m['limit']:
            print(f"\n[!] LIMIT REACHED: {m['name']} cannot exceed ±{m['limit']}")
            self.print_status()
            return
            
        m['pos'] = new_pos
        
        # Calculate Steps
        steps = int(abs(move_amt) * m['steps_per_unit'])
        dir_val = 1 if direction > 0 else 0
        
        # 2. Update ready_time lock for this motor
        # A full pulse cycle takes (pulse_speed * 2) seconds. 
        move_duration = steps * (self.pulse_speed * 2.0)
        m['ready_time'] = now + move_duration
        
        # 3. Publish Direction Command (O:pin:val)
        dir_msg = String()
        dir_msg.data = f"{m['dir']}:{dir_val}"
        self.pub_out.publish(dir_msg)
        
        # 4. Publish Step Pulses via Blink (pin:B:count:speed)
        pulse_msg = String()
        pulse_msg.data = f"{m['pul']}:B:{steps}:{self.pulse_speed}"
        self.pub_out.publish(pulse_msg)
        
        self.print_status()

    def adjust_jog_size(self, multiplier):
        self.jog_deg *= multiplier
        self.jog_cm *= multiplier
        print(f"\nJog size updated.")
        self.print_status()

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = ScaraTeleop()

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key in KEY_BINDINGS:
                motor, direction = KEY_BINDINGS[key]
                node.move_motor(motor, direction)
            elif key == '+':
                node.adjust_jog_size(1.5)
            elif key == '-':
                node.adjust_jog_size(0.66)
            elif key == '\x03': # CTRL-C
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()