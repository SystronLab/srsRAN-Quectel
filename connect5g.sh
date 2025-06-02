#!/bin/bash

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

# Retry mmcli -L until Quectel modem is detected
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

# Extract modem index
MODEM_INDEX=$(echo "$MODEM_LIST" | grep Quectel | awk -F'/' '{print $NF}' | awk '{print $1}')
echo "[INFO] Using modem index: $MODEM_INDEX"
echo "--------------------------------------------------"

echo "[STEP 3] Identifying AT port for modem $MODEM_INDEX..."

MODEM_INFO=$(mmcli -m "$MODEM_INDEX" 2>/dev/null)
echo "[DEBUG] Output from 'mmcli -m $MODEM_INDEX':"
echo "$MODEM_INFO"

echo "[DEBUG] Found AT ports:"
echo "$MODEM_INFO" | grep -o 'ttyUSB[0-9]\+ (at)'

# Use first AT port found
AT_PORT=$(echo "$MODEM_INFO" | grep -o 'ttyUSB[0-9]\+ (at)' | head -n 1 | awk '{print $1}')

if [[ -z "$AT_PORT" ]]; then
  echo "[ERROR] AT port not found for modem $MODEM_INDEX."
  exit 1
fi

AT_PORT="/dev/$AT_PORT"
echo "[INFO] Using AT port: $AT_PORT"
echo "--------------------------------------------------"

echo "[STEP 4] Checking readiness of $AT_PORT..."

for i in {1..15}; do
  if [[ -e "$AT_PORT" ]]; then
    echo "[INFO] AT port is ready: $AT_PORT"
    break
  fi
  echo "[WAIT] Attempt $i: Waiting for $AT_PORT to appear..."
  sleep 1
done

if [[ ! -e "$AT_PORT" ]]; then
  echo "[ERROR] AT port not found after 15 seconds."
  exit 1
fi

echo "[STEP 4 COMPLETE] AT port is ready."
echo "--------------------------------------------------"

echo "[STEP 5] Sending AT commands to $AT_PORT..."

{
  echo -e "AT\r"
  echo "[SEND] AT"
  sleep 1

  echo -e "AT+CGDCONT=1,\"IP\",\"srsapn\"\r"
  echo "[SEND] AT+CGDCONT=1,\"IP\",\"srsapn\""
  sleep 1

  echo -e "AT+CGATT=1\r"
  echo "[SEND] AT+CGATT=1"
  sleep 1

  echo -e "AT+CGACT=1,1\r"
  echo "[SEND] AT+CGACT=1,1"
  sleep 2
} > "$AT_PORT"

echo "[STEP 5 COMPLETE] AT commands sent to $AT_PORT successfully."
echo "--------------------------------------------------"
echo "[DONE] Modem initialization script completed successfully."

