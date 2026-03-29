import pwmio
import board
import time
import digitalio

from adafruit_motor import servo

class AsyncPin:
    def __init__(self, pin_id):
        self.pin_id = pin_id
        self.pin = self._setup_pin(pin_id)
        
        # State
        self.toggles_remaining = 0  
        self.delay_ns = 500_000_000 
        self.next_event_time = 0
        self.state = False          

    def _setup_pin(self, pin_id):
        pin_name = "GP{}".format(pin_id)
        if hasattr(board, pin_name):
            p = getattr(board, pin_name)
            d = digitalio.DigitalInOut(p)
            d.direction = digitalio.Direction.OUTPUT
            return d
        return None

    def set_value(self, value):
        """Turn pin SOLID on (True) or SOLID off (False)"""
        if not self.pin: return
        self.toggles_remaining = 0 # Stop any active blinking
        self.state = bool(value)
        self.pin.value = self.state

    def blink(self, count, duration_s):
        if not self.pin: return
        self.toggles_remaining = count * 2 
        self.delay_ns = int(duration_s * 1_000_000_000)
        self.state = True
        self.pin.value = True
        self.toggles_remaining -= 1
        self.next_event_time = time.monotonic_ns() + self.delay_ns

    def update(self):
        if self.toggles_remaining <= 0:
            return

        now = time.monotonic_ns()
        if now >= self.next_event_time:
            self.state = not self.state
            self.pin.value = self.state
            self.toggles_remaining -= 1
            self.next_event_time = now + self.delay_ns

class AsyncInput:
    def __init__(self, pin_id, pull_up=True):
        self.pin_id = pin_id
        self.pin = self._setup_pin(pin_id, pull_up)
        self.last_value = self.pin.value if self.pin else False
        self.last_change_time = 0
        self.debounce_ms = 50 * 1_000_000 
        self.enabled = True

    def _setup_pin(self, pin_id, pull_up):
        pin_name = "GP{}".format(pin_id)
        if hasattr(board, pin_name):
            p = getattr(board, pin_name)
            d = digitalio.DigitalInOut(p)
            d.direction = digitalio.Direction.INPUT
            d.pull = digitalio.Pull.UP if pull_up else digitalio.Pull.DOWN
            return d
        return None

    def update(self):
        if not self.pin or not self.enabled: return None
        current_value = self.pin.value
        now = time.monotonic_ns()
        if current_value != self.last_value:
            if now - self.last_change_time > self.debounce_ms:
                self.last_value = current_value
                self.last_change_time = now
                # Return 0 or 1. (PullUp: False=0=Pressed)
                val_int = 1 if current_value else 0
                return f"E:{self.pin_id}:{val_int}" 
        return None
    

class AsyncContinuousServo:
    
    def __init__(self, pin_id):
        self.pin_id = pin_id
        pin_name = "GP{}".format(pin_id)
        if hasattr(board, pin_name):
            p = getattr(board, pin_name)
            self.pwm = pwmio.PWMOut(p, frequency=50)
            self.servo = servo.ContinuousServo(self.pwm)
        else:
            self.servo = None
            
        self.target_time_ns = 0
        self.is_moving_timed = False

    def set_throttle(self, value):
        """Manual override: Set infinite continuous throttle"""
        if not self.servo: return
        self.is_moving_timed = False # Cancel any timed moves
        clamped_value = max(-1.0, min(1.0, float(value)))
        self.servo.throttle = clamped_value

    def move_timed(self, throttle, duration_s):
        """Move at a specific throttle for a set duration in seconds"""
        if not self.servo: return
        clamped_value = max(-1.0, min(1.0, float(throttle)))
        self.servo.throttle = clamped_value
        self.target_time_ns = time.monotonic_ns() + int(duration_s * 1_000_000_000)
        self.is_moving_timed = True

    def update(self):
        """Check if a timed move has finished"""
        if self.is_moving_timed:
            if time.monotonic_ns() >= self.target_time_ns:
                self.servo.throttle = 0.0 # Stop the servo
                self.is_moving_timed = False