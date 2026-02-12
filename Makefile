# --- CONFIGURATION ---
BOARD_VOLUME := /Volumes/CIRCUITPY
FW_DIR       := board-firmware
ROS_DIR      := ros2-node
PYTHON       := python3

# Auto-detect the USB Serial device (takes the first one found)
SERIAL_DEV   := $(shell ls /dev/tty.usbmodem* 2>/dev/null | head -n 1)

.PHONY: help upload serial clean bridge brain deps

help:
	@echo "🤖 Motion 2350 Pro - SCARA Control"
	@echo "----------------------------------"
	@echo "Hardware:"
	@echo "  make upload   : Copy firmware to $(BOARD_VOLUME)"
	@echo "  make serial   : Open USB serial monitor (Ctrl+A, K, Y to exit)"
	@echo ""
	@echo "ROS 2 Nodes (Run in separate terminals):"
	@echo "  make bridge   : Run the Hardware Driver (connects to USB)"
	@echo "  make brain    : Run the Logic Node (processes clicks)"
	@echo ""
	@echo "Maintenance:"
	@echo "  make clean    : Remove __pycache__ and temp files"
	@echo "  make deps     : Install Python dependencies (Mac/PC side)"

# --- HARDWARE TARGETS ---

upload:
	@echo "🔎 Checking for board at $(BOARD_VOLUME)..."
	@test -d $(BOARD_VOLUME) || (echo "❌ Error: $(BOARD_VOLUME) not found. Is board plugged in?"; exit 1)
	
	@echo "🚀 Deploying firmware..."
	@# cp -X strips macOS metadata (._ files) which confuse CircuitPython
	cp -X $(FW_DIR)/*.py $(BOARD_VOLUME)/
	
	@echo "💾 Syncing disks..."
	@sync
	@echo "✅ Done! Board is soft-reloading."

serial:
	@if [ -z "$(SERIAL_DEV)" ]; then \
		echo "❌ No USB device found matching /dev/tty.usbmodem*"; \
		exit 1; \
	fi
	@echo "🔌 Connecting to $(SERIAL_DEV)..."
	@echo "   (Press Ctrl+A, then K, then Y to exit 'screen')"
	screen $(SERIAL_DEV) 115200

# --- ROS 2 TARGETS ---

bridge:
	@echo "🌉 Starting Hardware Bridge..."
	$(PYTHON) $(ROS_DIR)/motion_2350_bridge.py

brain:
	@echo "🧠 Starting Logic Brain..."
	$(PYTHON) $(ROS_DIR)/scara_gpio_brain.py

deps:
	$(PYTHON) -m pip install pyserial

# --- UTILS ---

clean:
	rm -rf __pycache__
	rm -rf $(FW_DIR)/__pycache__
	rm -rf $(ROS_DIR)/__pycache__
	@echo "🧹 Cleaned."