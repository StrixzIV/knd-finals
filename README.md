# Motion 2350 Pro - ROS 2 SCARA Bridge

This project provides a modular bridge between **ROS 2** (running on a host Mac/PC) and a **Motion 2350 Pro** (RP2350) microcontroller running **CircuitPython**.

It enables the robot to:
1.  **Control GPIOs** (Grippers, Relays, LEDs) from ROS topics.
2.  **Read Sensors** (Limit Switches, Digital Inputs) and publish events to ROS.
3.  **Auto-Discover** available hardware pins.

---

## 📂 Project Structure

```text
.
├── board-firmware/       # Code running on the Motion 2350 Pro
│   ├── code.py           # Main loop & Serial Protocol Handler
│   └── async_pin.py      # Non-blocking GPIO & Input Debouncing classes
├── ros2-node/            # ROS 2 Nodes running on the Host (Mac/Linux)
│   ├── motion_2350_bridge.py # The Serial <-> ROS Bridge
│   └── scara_gpio_brain.py   # Example logic (Blink counter)
└── README.md

```

---

## 1. Firmware (Motion 2350 Pro)

### Installation

1. Connect the board via USB. It usually mounts as `CIRCUITPY`.
2. Copy the contents of `board-firmware/` to the drive.
3. The board will soft-reboot automatically.

### Serial Protocol

The firmware listens on the USB Serial port (115200 baud).

| Command | Syntax | Description | Example |
| --- | --- | --- | --- |
| **List Pins** | `L` | Ask board to list all valid GPIO pins. | `L`  `PINS:GP0,GP1...` |
| **Set Output** | `O:<Pin>:<Val>` | Set a pin High (1) or Low (0). | `O:16:1` (Turn GP16 ON) |
| **Blink** | `B:<Pin>:<Cnt>:<Spd>` | Blink a pin `Cnt` times at `Spd` seconds. | `B:25:5:0.2` |
| **Set Input** | `I:<Pin>` | Configure pin as Input (Pull-Up). | `I:20` |

### Events (Board -> Host)

When a configured Input pin changes state (debounced), the board sends:

* `E:<Pin>:<0|1>`
* `0` = **Pressed** (Circuit closed to GND)
* `1` = **Released** (Open circuit)



---

## 2. ROS 2 Nodes (Host Side)

### Dependencies

* ROS 2 (Humble/Iron/Jazzy/Rolling)
* Python `pyserial`: `pip install pyserial`

### A. The Bridge (`motion_2350_bridge.py`)

This node handles the serial communication with the board and translates it to ROS 2 topics.

#### Configuration

Edit `ros2-node/motion_2350_bridge.py` to match your USB port:

```python
self.port = '/dev/tty.usbmodem1101'  # macOS example
# self.port = '/dev/ttyACM0'         # Linux example
```

#### Running the Bridge

```bash
python3 ros2-node/motion_2350_bridge.py
```

### B. How to Configure Pins

Pins are configured dynamically via ROS topics. You do not need to reboot the board to change a pin's function.

#### 1. Set a Pin as INPUT (e.g., Limit Switch, Button)
To monitor a sensor, you must tell the board to start watching that pin.
*   **Topic**: `/scara/gpio/config`
*   **Format**: `"<PinNumber>:in"`
*   **Command**:
    ```bash
    ros2 topic pub --once /scara/gpio/config std_msgs/msg/String "data: '20:in'"
    ```
    *Once configured, the board will publish state changes (0=Pressed, 1=Released) to `/scara/gpio/in`.*

#### 2. Set a Pin as OUTPUT (e.g., LED, Relay, Gripper)
Outputs are initialized automatically the first time you send a command to them.
*   **Topic**: `/scara/gpio/out`
*   **Formats**:
    *   `"<PinNumber>:1"` (ON)
    *   `"<PinNumber>:0"` (OFF)
    *   `"<PinNumber>:B:<Count>:<Speed>"` (Blink)
*   **Example (Turn on GP16)**:
    ```bash
    ros2 topic pub --once /scara/gpio/out std_msgs/msg/String "data: '16:1'"
    ```

### C. The Logic Brain (`scara_gpio_brain.py`)

An example node that listens for button presses on **GP20** and triggers a blink sequence on **GP16** after a short timeout.

> [!IMPORTANT]
> Before using the brain, you **must** configure GP20 as an input. Run this command while the Bridge is active:
> ```bash
> ros2 topic pub --once /scara/gpio/config std_msgs/msg/String "data: '20:in'"
> ```

#### Running the Brain

```bash
python3 ros2-node/scara_gpio_brain.py
```

#### How it works:
1.  Ensure the Bridge is running and GP20 is configured as an input.
2.  Press the button on GP20 multiple times.
3.  Wait 1.5 seconds.
4.  The Brain sends a command to the Bridge to blink the LED on GP16 the same number of times you pressed the button.

---

## ⚡ Wiring Guide (Common)

* **Outputs (LEDs/Relays):**
* Signal  GPx
* Ground  GND

* **Inputs (Limit Switches):**
* Signal  GPx
* Ground  GND
* *Note: Firmware uses internal Pull-Up resistors. No external resistors needed.*
