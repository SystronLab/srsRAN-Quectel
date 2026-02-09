#!/bin/bash
set -e

MODEM_INDEX=$(mmcli -L | grep -i quec | awk -F'/' '{print $NF}' | awk '{print $1}')
sudo mmcli -m "$MODEM_INDEX" --simple-connect="apn=srsapn"
