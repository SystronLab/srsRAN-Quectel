#!/bin/bash
set -e

# Wait until ModemManager detects a modem
while true; do
  MODEM_INDEX=$(mmcli -L 2>/dev/null | grep -i quec | awk -F'/' '{print $NF}' | awk '{print $1}')
  if [ -n "$MODEM_INDEX" ]; then
    break
  fi
  sleep 1
done

# Enable modem
sudo mmcli -m "$MODEM_INDEX" -e

# Connect using APN
sudo mmcli -m "$MODEM_INDEX" --simple-connect="apn=srsapn"

echo "Modem connected (index $MODEM_INDEX)"
