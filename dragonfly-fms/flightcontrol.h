/*****************************************************************************
 * @file    flightcontrol.h
 * @brief	Header file for flightcontrol
 ******************************************************************************/

#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

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

#include "com.h"

class flightcontrol
{
private:
	com fcbCom;
	
public:
	flightcontrol();
	int getReceiver();
};

#endif
