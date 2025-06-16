#!/bin/bash

set -e

echo "[STEP 1] Restarting ModemManager..."

echo "[INFO] Stopping ModemManager..."
sudo systemctl stop ModemManager
sleep 2

echo "[INFO] Starting ModemManager..."
sudo systemctl start ModemManager

echo "[INFO] Enabling ModemManager to start on boot..."
sudo systemctl enable ModemManager

echo "[INFO] Waiting 5 seconds for ModemManager to detect modem..."
sleep 5

echo "[STEP 1 COMPLETE] ModemManager restarted successfully."
echo "--------------------------------------------------"

echo "[STEP 2] Detecting Quectel modem via mmcli..."

for i in {1..10}; do
  MODEM_LIST=$(mmcli -L 2>/dev/null)
  echo "[DEBUG] Attempt $i - mmcli output:"
  echo "$MODEM_LIST"

  if echo "$MODEM_LIST" | grep -q Quectel; then
    echo "[INFO] Quectel modem found."
    break
  fi

  echo "[WAIT] Modem not detected yet. Retrying in 2 seconds..."
  sleep 2
done

if ! echo "$MODEM_LIST" | grep -q Quectel; then
  echo "[ERROR] No Quectel modem found via ModemManager after multiple attempts."
  exit 1
fi

MODEM_INDEX=$(echo "$MODEM_LIST" | grep Quectel | awk -F'/' '{print $NF}' | awk '{print $1}')
echo "[INFO] Using modem index: $MODEM_INDEX"
echo "--------------------------------------------------"

echo "[STEP 3] Connecting and configuring modem $MODEM_INDEX..."

sudo mmcli -m "$MODEM_INDEX" -e
sudo mmcli -m "$MODEM_INDEX" --simple-connect="apn=srsapn"

# Assuming interface is wwan0 and IP is fixed; adjust as needed
sudo ip link set wwan0 up
sudo ip addr add 10.45.0.2/16 dev wwan0

sudo iptables -F

echo "[STEP 3 COMPLETE] Modem is connected and network interface is configured."
echo "--------------------------------------------------"

echo "[STEP 4] Starting UDP server..."

python3 - <<EOF
import socket
import json

UDP_IP = "0.0.0.0"
UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"[UDP SERVER] Listening on {UDP_IP}:{UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)
    try:
        message = json.loads(data.decode())
        print(f"[UDP SERVER] Received from {addr}:\\n{json.dumps(message, indent=2)}")
    except json.JSONDecodeError:
        print(f"[UDP SERVER] Invalid JSON from {addr}: {data}")
EOF

