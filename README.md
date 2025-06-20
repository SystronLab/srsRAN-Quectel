# srsRAN-Quectel

Guide to setup srsRAN with USRP x310 and Quectel RMU500-EK

## Connect the modem with the type c cord

```bash
lsusb
sudo dmesg | grep ttyUSB
```

## Install and Manage ModemManager

```bash
sudo apt install modemmanager

sudo systemctl stop ModemManager
sudo systemctl start ModemManager
sudo systemctl enable ModemManager

systemctl status ModemManager
```

### List Modems

```bash
sudo mmcli -L
```

---

## Access AT Command Console

```bash
lsusb
sudo dmesg | grep ttyUSB
ls /dev/ttyUSB*
```

### Install Picocom

```bash
sudo apt install picocom
```

### Identify AT Port

```bash
sudo mmcli -m 0 | grep ports
```

### Start AT Console

```bash
sudo picocom -b 115200 /dev/ttyUSB2
```

---

## Useful AT Commands

- Check AT connection:

  ```
  AT
  ```

- Currently registered PLMN:

  ```
  AT+COPS?
  ```

- Returns the IMSI:

  ```
  AT+CIMI
  ```

- Returns APNs:
  ```
  AT+CGDCONT?
  ```

Example output:

```
+CGDCONT: 1,"IP","srsapn",...
+CGDCONT: 2,"IPV4V6","ims",...
+CGDCONT: 3,"IPV4V6","sos",...
```

> ⚠️ If you see multiple APNs (ims, sos), you'll get `DNN Not Supported OR Not Subscribed in the Slice` error in AMF logs. Remove extra APNs with:

```bash
AT+CGDCONT=2
AT+CGDCONT=3
```

### Set APN with IPv4 Only

```bash
AT+CGDCONT=1,"IP","srsapn"
```

### Attach to the Packet Domain

```bash
AT+CGATT=1
```

### Activate PDP Context 1

```bash
AT+CGACT=1,1
```

### Check if IP Address is Assigned

```bash
AT+CGPADDR=1
```

### Ping Test

```bash
AT+QPING=1,"8.8.8.8"
```

# Using the script

You can use the script to automatically connect the 5g modem to the srsRAN.
Make it executable using

```bash
chmod +x ./connect5g.sh
```

Check the data interface
AT+QCFG="data_interface"

Set it to usb
AT+QCFG="data_interface",0,0


AT+QCFG="usbnet"

AT+QCFG="usbnet",2


Do ip link
you'll see wwan0 as an interface


List all connected modem
sudo mmcli -L

take note of modem number i.e. in my case its 0. Then, I run following commands

sudo mmcli -m 0 -e
sudo mmcli -m 0 --simple-connect="apn=srsapn"


sudo ip link set wwan0 up
sudo ip a add 10.45.0.2/16 dev wwan0

sudo iptables -F

then run it with

```bash
sudo ./connect5g.sh
```
