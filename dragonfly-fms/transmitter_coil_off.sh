#!/bin/bash      
echo ************transmitter coil OFF**************** 
curl --request POST 'https://api.thingspeak.com/update?key=4CMYM4V0ZC24DYCU&field1=0'
