#!/bin/sh

loc="/home/wheeltec/Projects/OranDemos/Demo1"
log=$loc"/log"

echo "ORAN Demo - Demo1: Gamepad Control of RosBot over 5G Private Network"
echo "ORAN Demo - Demo1: Gamepad Control of RosBot over 5G Private Network" > $log
echo "running oran_demo1_startup.sh script" >> $log
echo "attempting to run 5gconnect.sh..." >> $log
$loc"/5gconnect.sh"
retVal=$?
echo "5gconnect.sh returned" $retVal >> $log

if [ $retVal -ne 0 ]; then
    echo "error so quitting..."
    echo "error so quitting..." >> $log
    exit $retVal 
fi

bash $loc"/oran_demo1_robot_startup.sh" 

exit 0

