#!/bin/bash      
echo ************transmitter coil ON**************** 
curl --request POST 'https://api.thingspeak.com/update?key=4CMYM4V0ZC24DYCU&field1=1'
