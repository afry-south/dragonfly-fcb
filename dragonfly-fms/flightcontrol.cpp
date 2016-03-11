/*****************************************************************************
 * @file    flightcontrol.cpp
 * @brief	Flightcontrol handles the communication with the flight control
 * 			board, FCB.
 * 			It handles high level tasks concerning the flight.
 ******************************************************************************/

#include <iostream>
#include <fcntl.h>
#include <stropts.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdio>
#include <fstream>
#include <cstring>
#include <cstdlib>

#include "flightcontrol.h"
#include "com.h"

using namespace std;

/**
 * @brief	Constructor of flightcontrol.
 * 			Opens the communication to the flight control board.
 * @param	None.
 * @retval	None.
 */
flightcontrol::flightcontrol()
{
	fcbCom.openCom("/dev/ttyS2", B115200);
}

/**
 * @brief	Gets the sensor values from the flight control board
 * 			in nonserialized format
 * @param	None.
 * @retval	None.
 */
//TODO Use serialized format, protobuffer
//TODO Extract the data and save in variables.
int flightcontrol::getReceiver()
{
	char buffer[MAX_BUFFER_SIZE];
	//empty first
	fcbCom.readCom(&buffer[0], sizeof(buffer));
	//send message
	fcbCom.writeCom("get-receiver n\r", MAX_BUFFER_SIZE);
	sleep(1);
	//receive message
	fcbCom.readCom(&buffer[0], sizeof(buffer));
	
	return 0;
}



