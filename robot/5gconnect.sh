#!/bin/bash

set -e

log="/home/wheeltec/Projects/OranDemos/Demo1/log"


echo "[INFO] Waiting 5 seconds for ModemManager to detect modem..."
echo "[INFO] Waiting 5 seconds for ModemManager to detect modem..." >> $log
sleep 5

echo "[STEP 2] Detecting Quectel modem via mmcli..."
echo "[STEP 2] Detecting Quectel modem via mmcli..." >> $log

for i in {1..30}; do
  MODEM_LIST=$(mmcli -L 2>/dev/null)
  echo "[DEBUG] Attempt $i - mmcli output:"
    echo "[DEBUG] Attempt $i - mmcli output:" >> $log
  echo "$MODEM_LIST"

  if echo "$MODEM_LIST" | grep -q Quectel; then
    echo "[INFO] Quectel modem found."
    echo "[INFO] Quectel modem found." >> $log
    break
  fi

  echo "[WAIT] Modem not detected yet. Retrying in 2 seconds..."
  echo "[WAIT] Modem not detected yet. Retrying in 2 seconds..." >> $log
  sleep 2
done

if ! echo "$MODEM_LIST" | grep -q Quectel; then
  echo "[ERROR] No Quectel modem found via ModemManager after multiple attempts."
  echo "[ERROR] No Quectel modem found via ModemManager after multiple attempts." >> $log
  exit 1
fi

MODEM_INDEX=$(echo "$MODEM_LIST" | grep Quectel | awk -F'/' '{print $NF}' | awk '{print $1}')
echo "[INFO] Using modem index: $MODEM_INDEX"
echo "--------------------------------------------------"
echo "[INFO] Using modem index: $MODEM_INDEX" >> $log
echo "--------------------------------------------------" >> $log

echo "[STEP 3] Connecting and configuring modem $MODEM_INDEX..."
echo "[STEP 3] Connecting and configuring modem $MODEM_INDEX..." >> $log

sudo mmcli -m "$MODEM_INDEX" -e
sudo mmcli -m "$MODEM_INDEX" --simple-connect="apn=srsapn"

# Assuming interface is wwan0 and IP is fixed; adjust as needed
sudo ip link set wwan0 up
sudo ip addr add 10.45.0.2/16 dev wwan0

sudo iptables -F

echo "[STEP 3 COMPLETE] Modem is connected and network interface is configured."
echo "--------------------------------------------------"
echo "[STEP 3 COMPLETE] Modem is connected and network interface is configured." >> $log
echo "--------------------------------------------------" >> $log

exit 0
EOF

