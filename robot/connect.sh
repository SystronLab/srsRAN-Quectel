#!/bin/bash
set -e

# Get modem index detected by ModemManager
MODEM_INDEX=$(mmcli -L | grep -i quec | awk -F'/' '{print $NF}' | awk '{print $1}')

# Enable modem
sudo mmcli -m "$MODEM_INDEX" -e

# Connect using APN
sudo mmcli -m "$MODEM_INDEX" --simple-connect="apn=srsapn"

echo "Modem connected."
