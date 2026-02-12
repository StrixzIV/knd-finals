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
├── ros2_bridge/          # ROS 2 Node running on the Host (Mac/Linux)
│   └── controller.py     # The Python Node
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

### Events (Board  Host)

When a configured Input pin changes state (debounced), the board sends:

* `E:<Pin>:<0|1>`
* `0` = **Pressed** (Circuit closed to GND)
* `1` = **Released** (Open circuit)



---

## 2. ROS 2 Bridge (Host Side)

### Dependencies

* ROS 2 (Humble/Iron/Jazzy)
* Python `pyserial`: `pip install pyserial`

### Configuration

Edit `ros2_bridge/controller.py` to match your USB port:

```python
self.port = '/dev/tty.usbmodem1101'  # macOS example
# self.port = '/dev/ttyACM0'         # Linux example

```

### Running the Node

```bash
python3 ros2_bridge/controller.py

```

### Topics & API

#### 1. Configure Hardware (`/scara/gpio/config`)

* **Type:** `std_msgs/msg/String`
* **Usage:**
* `"list"`: Asks the board to print available pins to the node logger.
* `"16:in"`: Configures GP16 as a digital input (limit switch).



```bash
ros2 topic pub --once /scara/gpio/config std_msgs/msg/String "data: '20:in'"

```

#### 2. Control Outputs (`/scara/gpio/out`)

* **Type:** `std_msgs/msg/String`
* **Usage:**
* `"16:1"`: Turn GP16 **ON**.
* `"16:0"`: Turn GP16 **OFF**.
* `"16:B:5:0.5"`: Blink GP16, 5 times, 0.5s interval.



```bash
ros2 topic pub --once /scara/gpio/out std_msgs/msg/String "data: '16:1'"

```

#### 3. Read Inputs (`/scara/gpio/in`)

* **Type:** `std_msgs/msg/String`
* **Data Format:** `"<Pin>:<0|1>"`
* **Usage:** Subscribe to this topic to react to limit switches.

```bash
ros2 topic echo /scara/gpio/in
# Output:
# data: "20:0"  <-- Switch Pressed
# data: "20:1"  <-- Switch Released

```

---

## ⚡ Wiring Guide (Common)

* **Outputs (LEDs/Relays):**
* Signal  GPx
* Ground  GND

* **Inputs (Limit Switches):**
* Signal  GPx
* Ground  GND
* *Note: Firmware uses internal Pull-Up resistors. No external resistors needed.*
