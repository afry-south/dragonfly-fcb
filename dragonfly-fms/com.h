/*****************************************************************************
 * @file    com.h
 * @brief	Header file for com
 ******************************************************************************/

#ifndef COM_H
#define COM_H

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

#define MAX_BUFFER_SIZE 256

class com
{
public:
	com();
	int openCom(const char *port, speed_t speed);
	int readCom(char *bufferRead, int nbrBytes);
	int writeCom(const char *bufferWrite, unsigned int nbrBytes);
private:
	int descriptor;
};

#endif
