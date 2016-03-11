/*****************************************************************************
 * @file    charger.cpp
 * @brief	Charger contains functionality to monitor the onboard balanced
 *			charger in the dragonfly and to control the transmitter coil in
 *			the inductive charger.
 *
 *			To communicate with the inductive charger in the landing pad
 *			thingspeak.com is used.
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
#include <thread>
//#include <curl.h>

#include "charger.h"

using namespace std;
#define BUFFER_SIZE 60
#define PARAMETER_SIZE 10

std::string writeAPIKey = "4CMYM4V0ZC24DYCU";
std::string readAPIKey = "NGDJHL37B3IL5LAK";
std::string baseUpdateURL = "https://api.thingspeak.com/update?key=";
std::string coilOn = "&field1=1";
std::string coilOff = "&field1=0";

//char port_string[] = "/dev/ttyUSB0";
char port_string[] = "voltage_data";
void charging();

//TODO Now it is using a file with faked voltage data, use /dev/ttyUSB instead.
//TODO The theading is not working OK, try phread instead.
//TODO move into class
bool isCharging;
float voltage_level_finished;

/**
 * @brief	Constructor of charger
 * 			Initializes variables
 * @param	None.
 * @retval	None.
 */
charger::charger()
{
	isCharging = false;
	voltage_level_finished = 16.800; //16.8 Volt
}

/**
 * @brief	Sets the voltage when the charging is considered finished
 * @param	voltage
 * @retval	None
 */
void charger::setFinishedVoltage(float voltage)
{
	voltage_level_finished = voltage;
}

/**
 * @brief	Open the communication to the onboard charger
 * @param	None
 * @retval	0: if OK, 1:if NOT OK
 */
int charger::openCom()
{
	//std::ifstream infile(port_string);
	return 0;
}

/**
 * @brief	Starts charging the battery.
 * 			Opens communication with the charger, turns the transmitter coil current on.
 * 			Starts a new thread that monitors the status of the onboard charger.
 * @param	None
 * @retval	None
 */
void charger::startCharger()
{
	//Open port
	this->openCom();
	
	//Tell transmitter coil to start
	transmitterCoilOn();
	
	//Start charging in the background
	std::thread chargeThread(charging);
}

/**
 * @brief	Turns transmitter coil on.
 * @param	None
 * @retval	None
 */
void charger::transmitterCoilOn()
{
	//call script
	//script uses cURL to make a post request to thingspeak.com
	//TODO error handling
	system("./transmitter_coil_on.sh");
	
	//or use libcurl
	/*
	string writeURL;
	//Make url
	writeURL = baseUpdateURL + writeAPIKey + coilOn;
	
	//Send url
	CURL * myHandle;
	CURLcode result; 
	
	curl_global_init( CURL_GLOBAL_ALL );
	myHandle = curl_easy_init ( ) ;
	curl_easy_setopt(myHandle, CURLOPT_URL, writeURL);
	result = curl_easy_perform( myHandle );
	curl_easy_cleanup( myHandle );
	*/ 
}

/**
 * @brief	Turns transmitter coil off.
 * @param	None
 * @retval	None
 */
void charger::transmitterCoilOff()
{
	//call script
	system("./transmitter_coil_off.sh");
}

/**
 * @brief	Function to be started by thread.
 * 			The function monitors the charger and exits when the
 * 			charging is finished.
 * @param	None
 * @retval	None
 */
void charging()
{
/*	float parameters[PARAMETER_SIZE];	//parameters on each line is delimeted by ";"
	std::string line;	
	std::ifstream infile(port_string);
	//TODO test if port is open
	
	cout << "start charging\n" << endl;
	isCharging = true;
	while(isCharging)
	{
		std::getline(cin, line);
		std::getline(infile, line);
		cout << "line:" << line << endl;
			
		//parse each line and find the total voltage
		//$A;B;;C;total voltage;D;cell1 voltage; cell2 voltage; cell3 voltage, cell4 voltage;E;F;G;H;I;J \n 
		int i=0;
		char *charArray = (char*)line.c_str();
		char *token = std::strtok(charArray, ";");
		while ((token != NULL) && (i < PARAMETER_SIZE)) 
		{
			std::cout << "Parameter" << i << ": " << token << '\n';
			parameters[i]=atof(token);
			token = std::strtok(NULL, ";");
			i++;
		}
		 
		//Test if total voltage has reached level
		if(parameters[3] >= voltage_level_finished)
		{
			isCharging = false;	
		}
		
		//TODO Time out charging is needed
	}
*/	
	cout << "finished charging\n" << endl;
}
