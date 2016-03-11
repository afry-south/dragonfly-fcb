/*****************************************************************************
 * @file    com.cpp
 * @brief	Com contains functionality for communication between the
 * 			flight manager other and other units in the system.
 ******************************************************************************/

#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <stropts.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdio>
#include <fstream>
#include <cstring>
#include <cstdlib>

#include "com.h"

using namespace std;

//Constructor
com::com()
{
	
}

/**
 * @brief	Open a serial communication port.
 * @param	const char *port: path to the file to open
 * 			speed_t speed: baudrate
 * @retval	0: if open is OK, 1:if not OK
 */
int com::openCom(const char *port, speed_t speed)
{
	struct termios tty;
	//Open
	descriptor = open( port, O_RDWR| O_NOCTTY | O_NDELAY);

	//Get current settings of tty
	if ( tcgetattr ( descriptor, &tty ) != 0 )
	{
	   std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
	   return 1;
	}

	//Set baudrate
	cfsetospeed (&tty, (speed_t)speed);
	cfsetispeed (&tty, (speed_t)speed);
	
	tty.c_cflag     &=  ~PARENB;	//No parity
	tty.c_cflag     &=  ~CSTOPB;	//Only one stopbit
	tty.c_cflag     &=  ~CSIZE;		//Clear character size
	tty.c_cflag     |=  CS8;		//8 bit character
	tty.c_cflag     &=  ~CRTSCTS;	//No flow control
	
	//Flush
	tcflush( descriptor, TCIFLUSH );

	//set new settings
	if ( tcsetattr ( descriptor, TCSANOW, &tty ) != 0)
	{
	   std::cout << "Error " << errno << " from tcsetattr" << std::endl;
	   return 1;
	}

	std::cout << port << "is open" << endl;
	return 0;
}

/**
 * @brief	Read serial communication port.
 * 			Reads until a CR is read or nbrBytes is read.
 * @param	*bufferRead: buffer to put read characters in.
 * 			nbrBytes: max nbr of char to read.
 * @retval	0: OK, 1: not OK
 */
int com::readCom(char *bufferRead, int nbrBytes)
{
	char charRead;
	int	bufferIndex=0;
	int nbrRead;
	
	do
	{
		nbrRead = read(descriptor, &charRead, 1);
		sprintf(&bufferRead[bufferIndex], "%c", charRead);
		bufferIndex++;
		std::cout << charRead;
	}
	while((charRead != '\r') && (nbrRead > 0) && (bufferIndex<nbrBytes));
	
	cout << bufferIndex << "bytes read" << endl;
	
	if(nbrRead <= 0)
	{
		return 1;
	}
	
	return 0;
}

/**
 * @brief	Write to serial communication port.
 * 			Write until CR in buffer or nbrBytes.
 * @param	*bufferWrite: buffer to be written.
 * 			nbrBytes: max nbr of char to write.
 * @retval	0: OK, 1: not OK
 */
int com::writeCom(const char *bufferWrite, unsigned int nbrBytes)
{
	unsigned int nbrWritten;
	unsigned int bufferIndex=0;
	
	do
	{
		nbrWritten = write(descriptor, &bufferWrite[bufferIndex], 1);
		bufferIndex++;
	}
	while((bufferWrite[bufferIndex-1] != '\r') && (nbrWritten > 0) && (bufferIndex < nbrBytes));
	
	cout << bufferIndex << "bytes written" << endl;
	
	if(nbrWritten <= 0)
	{
		return 1;
	}
	
	return 0;
}



