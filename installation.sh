#!/bin/bash

# Exit on any error
set -e

# Define paths and variables
VENV_PATH="/home/mdt/navibox"
PYTHON_SCRIPT="/home/mdt/navibox/main.py"
LOG_DIR="/var/log/navibox"
USER="mdt"

# Ensure the script is run as root
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run as root (use sudo)." >&2
    exit 1
fi

# Step 1: Update and upgrade the system
echo "Updating and upgrading the system..."
apt-get update && apt-get upgrade -y

# Install required system packages
echo "Installing system dependencies.It can take a little longer plz wait ........"
apt-get install -y python3 python3-pip python3-venv libatlas-base-dev

# Step 2: Create and activate virtual environment
echo "Creating virtual environment at $VENV_PATH..."
mkdir -p "$VENV_PATH"
python3 -m venv "$VENV_PATH/venv"
source "$VENV_PATH/venv/bin/activate"

# Step 3: Install Python dependencies
echo "Installing Python dependencies..."
pip3 install --no-cache-dir \
    pyserial \
    python-dotenv \
    websocket-client \
    pynmea2

# Deactivate virtual environment
deactivate

# Ensure log directory exists and has correct permissions
echo "Setting up log directory..."
mkdir -p "$LOG_DIR"
chown "$USER:$USER" "$LOG_DIR"
chmod 755 "$LOG_DIR"

# Ensure the Python script is in place (assuming it will be placed manually or exists)
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "Error: Python script $PYTHON_SCRIPT not found. Please place it there manually." >&2
    exit 1
fi
chown "$USER:$USER" "$PYTHON_SCRIPT"
chmod 644 "$PYTHON_SCRIPT"

# Step 4: Prompt for systemd service setup
read -p "Do you want to set up a systemd service to run the GPS data collection system automatically? (y/n): " answer
if [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
    echo "Creating systemd service..."
    cat > /etc/systemd/system/navibox.service <<EOF
[Unit]
Description=Navibox GPS Data Collection Service
After=network-online.target
Wants=network-online.target

[Service]
User=$USER
WorkingDirectory=$VENV_PATH
ExecStart=$VENV_PATH/venv/bin/python3 $PYTHON_SCRIPT
Restart=always
RestartSec=10
EnvironmentFile=/home/mdt/.env

[Install]
WantedBy=multi-user.target
EOF

    # Reload systemd, enable and start the service
    systemctl daemon-reload
    systemctl enable navibox.service
    systemctl start navibox.service
    echo "Systemd service 'navibox' has been created and started."
else
    echo "Skipping systemd service setup."
fi

echo "Installation completed successfully!"
exit 0