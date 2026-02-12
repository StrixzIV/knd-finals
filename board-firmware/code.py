import supervisor
import sys
import board
from async_pin import AsyncPin, AsyncInput

active_outputs = {}
active_inputs = {}

def get_output(pin_id):
    if pin_id not in active_outputs:
        active_outputs[pin_id] = AsyncPin(pin_id)
    return active_outputs[pin_id]

def get_input(pin_id):
    if pin_id not in active_inputs:
        active_inputs[pin_id] = AsyncInput(pin_id, pull_up=True)
    return active_inputs[pin_id]

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
                
                # --- I: INPUT MODE ---
                elif cmd == "I" and len(parts) >= 2:
                    pin = int(parts[1])
                    get_input(pin) 
                    print(f"ACK: Monitor GP{pin}")

                # --- O: OUTPUT STATIC (NEW) ---
                # Format: O:<Pin>:<1/0>
                elif cmd == "O" and len(parts) >= 3:
                    pin = int(parts[1])
                    val = int(parts[2])
                    get_output(pin).set_value(val)
                    print(f"ACK: GP{pin}={'HIGH' if val else 'LOW'}")

                # --- B: BLINK (Optional) ---
                elif cmd == "B" and len(parts) >= 4:
                    pin = int(parts[1])
                    cnt = int(parts[2])
                    spd = float(parts[3])
                    get_output(pin).blink(cnt, spd)
                    print(f"ACK: Blink GP{pin}")

        except ValueError:
            print("ERR: Parse")

    # Update Loops
    for ctrl in active_outputs.values():
        ctrl.update()
    for sens in active_inputs.values():
        event = sens.update()
        if event: print(event)