import supervisor
import sys
import board
from async_pin import AsyncPin, AsyncInput, AsyncContinuousServo

active_outputs = {}
active_inputs = {}
active_servos = {}

def get_output(pin_id):
    if pin_id not in active_outputs:
        active_outputs[pin_id] = AsyncPin(pin_id)
    return active_outputs[pin_id]

def get_input(pin_id):
    if pin_id not in active_inputs:
        active_inputs[pin_id] = AsyncInput(pin_id, pull_up=True)
    return active_inputs[pin_id]

def get_servo(pin_id):
    if pin_id not in active_servos:
        active_servos[pin_id] = AsyncContinuousServo(pin_id)
    return active_servos[pin_id]

# Pre-config input pins
# SW_NO1 -> GP16
# SW_NO2 -> GP17
# SW_NO3 -> GP18
# SW_NO4 -> GP19
for limit_sw_pin in [16, 17, 18, 19]:
    get_input(limit_sw_pin)

# Pre-config output pins
# GP0 -> PUL+ #1
# GP1 -> DIR+ #1
# GP2 -> PUL+ #2
# GP3 -> DIR+ #2
# GP4 -> PUL+ #3
# GP5 -> DIR+ #3
# GP6 -> PUL+ #4
# GP26 -> DIR+ #4
for steppers_pin in [0, 1, 2, 3, 4, 5, 6, 26]:
    get_output(steppers_pin)

# GP7 -> SIG (SERVO)
get_servo(7)

print("MOTION 2350: SCARA GPIO Bridge Ready.")

while True:
    if supervisor.runtime.serial_bytes_available:
        try:
            line = sys.stdin.readline().strip()
            if line:
                parts = line.split(":")
                cmd = parts[0].upper()
                
                # --- L: LIST PINS ---
                if cmd == "L":
                    available = [p for p in dir(board) if p.startswith("GP")]
                    available.sort(key=lambda x: int(x[2:]))
                    print(f"PINS:{','.join(available)}")
                
                # --- I: SET INPUT ---
                elif cmd == "I" and len(parts) >= 2:
                    pin = int(parts[1])
                    get_input(pin) 
                    print(f"ACK: Monitor GP{pin}")

                # --- O: SET OUTPUT ---
                elif cmd == "O" and len(parts) >= 3:
                    pin = int(parts[1])
                    val = int(parts[2])
                    get_output(pin).set_value(val)
                    print(f"ACK: GP{pin}={'HIGH' if val else 'LOW'}")

                # --- B: BLINK ---
                elif cmd == "B" and len(parts) >= 4:
                    pin = int(parts[1])
                    cnt = int(parts[2])
                    spd = float(parts[3])
                    get_output(pin).blink(cnt, spd)
                    print(f"ACK: Blink GP{pin}")

                # --- S: SERVO THROTTLE (Infinite) ---
                elif cmd == "S" and len(parts) >= 3:
                    pin = int(parts[1])
                    throttle = float(parts[2])
                    get_servo(pin).set_throttle(throttle)
                    print(f"ACK: Servo GP{pin}={throttle}")
                    
                # --- T: TIMED SERVO MOVE ---
                elif cmd == "T" and len(parts) >= 4:
                    pin = int(parts[1])
                    throttle = float(parts[2])
                    duration = float(parts[3])
                    get_servo(pin).move_timed(throttle, duration)
                    print(f"ACK: Timed Move GP{pin} Spd={throttle} Dur={duration}s")

        except ValueError:
            print("ERR: Parse")

    # Update Loops
    for ctrl in active_outputs.values():
        ctrl.update()
    for sens in active_inputs.values():
        event = sens.update()
        if event: print(event)