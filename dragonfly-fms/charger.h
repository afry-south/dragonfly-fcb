/*****************************************************************************
 * @file    charger.h
 * @brief	Header file for charger
 ******************************************************************************/

#ifndef CHARGER_H
#define CHARGER_H

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
#include <thread>

using namespace std;
#define BUFFER_SIZE 60
#define PARAMETER_SIZE 10

class charger
{
	public: 
	charger();
	void startCharger();
	void setFinishedVoltage(float voltage);
	int openCom();
	
	private:
	void transmitterCoilOn();
	void transmitterCoilOff();
	//float voltage_level_finished;
	//bool isCharging;
};

#endif 
