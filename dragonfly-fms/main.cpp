/*****************************************************************************
 * @file    main.cpp
 * @brief	Main file of the Flight Manager program.
 * 			Is intended to run on a linux system (armbian)
 ******************************************************************************/
#include <iostream>
#include <stdlib.h>
#include "charger.h"
#include "flightcontrol.h"

using namespace std;

int main () {
	cout << "Starting Flight Manager" << endl;
	charger iCharger;
	flightcontrol fController;
	
	while(true)
	{
		//iCharger.startCharger();
		//fController.getReceiver();
		//getchar();
	}
    return 0;
}
