#!/bin/bash      
echo ************Configure modem**************** 
#First switch usb mode from mass storage to modem
usb_modeswitch -v 2357 -p f000 -V 2357 -P 9000 -W -I -n -M'5553424312345678000000000000061e000000000000000000000000000000' -2 '5553424312345678000000000000061b000000020000000000000000000000'
#Second configure the modem using sagis3g
./sakis3g "connect" USBMODEM="2357:9000" APN="online.telia.se" USBINTERFACE="3" USBDRIVER="option" OTHER="USBMODEM"

        
