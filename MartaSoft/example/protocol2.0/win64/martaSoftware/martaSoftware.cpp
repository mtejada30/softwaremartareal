/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

//https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sample_code/cpp_bulk_read_write_protocol_2_0/#cpp-bulk-read-write-protocol-20

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <vector>
#include <time.h>
#include <string>
#include <iomanip>

#include "Robot.h"
#include "motorDynamixel.h"                                  // Uses Dynamixel SDK library


// Protocol version
#define PROTOCOL_VERSION                2.0         // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID0                         1           // Dynamixel ID: 1
#define DXL_ID1                         2           // Dynamixel ID: 1
#define DXL_ID2                         3           // Dynamixel ID: 1
//#define DEVICENAME                      "COM5"      // Check which port is being used on your controller// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


std::vector<int> DXL_ID;
std::vector<int> MOVE_POS = { 0, 2000, 0 };





int main()
{	
	Robot Marta("martaAll.yaml");
	std::vector<int> IDs = { 1, 2, 3, 4 };
	std::vector<int> pos1 = { 1000, 1000, 500, 500};
	std::vector<int> pos2 = { 0, 0, 0, 0};

	bool aux = true;
	Marta.readFileMove("initParts.txt");
	/*
	do{
		aux = Marta.moveMotors(IDs, pos1);
		std::cout << "WHAT " << aux << std::endl;
		Marta.writeFile("TestWrite.txt");
	} while (aux);
	
	
	do {
		std::cout << "WHAT1 " << aux << std::endl;
		aux = Marta.moveMotors(IDs, pos2);
		Marta.writeFile("TestWrite.txt");
	} while (aux);
	*/
	/*
	float t = 0;
	auto start = std::chrono::high_resolution_clock::now();

	std::ofstream currentfile;
	std::cout << "UHMMMM " << std::endl;
	std::vector<std::vector<int> > info;
	std::vector<int> aux;
	currentfile.open("currentWrite.txt");
	currentfile << "Time";
	for (int i = 0; i < DXL_ID.size(); i++)
		currentfile << "," << DXL_ID[i] << "IDPosition," << DXL_ID[i] << "IDCurrent";
	currentfile << "\n";

	std::cout << "INIT " << std::endl;
	initialize(DEVICENAME);
	std::cout << "PING" << std::endl;
	for (int i = 0; i < DXL_ID.size(); i++) {
		initializeBulkRead(DXL_ID[i], DEVICENAME);
		ping(DXL_ID[i], DEVICENAME);
		set_torque(DXL_ID[i], DEVICENAME, 1);
	}

	
	int pos = -99999;
	int posF =-9999;
	int auxt = -99999;
	for (int i = 0; i < MOVE_POS.size(); i++) {
		for (int j = 0; j < DXL_ID.size(); j++) {
			set_position(DXL_ID[j], DEVICENAME, MOVE_POS[i]);
			//std::this_thread::sleep_for(std::chrono::milliseconds(TIME_SLEEP));
		}
		while (abs(MOVE_POS[i] - posF) > STATUS_THRESHOLD){
			auxt = -9999;
			posF = -9999;
			currentfile << std::setprecision(3) << t << ",";
			auto start = std::chrono::high_resolution_clock::now();
			//std::this_thread::sleep_for(std::chrono::milliseconds(TIME_SLEEP));

			for (int j = 0; j < DXL_ID.size(); j++) {
				pos = get_present_position(DXL_ID[j], DEVICENAME);
				//pos = get_bulk_present_position(DXL_ID[j], DEVICENAME);
				if (abs(MOVE_POS[i] - pos) > auxt) {
					posF = pos;
					auxt = abs(MOVE_POS[i] - pos);
				}
				currentfile << pos << "," << get_present_current(DXL_ID[j], DEVICENAME) << ",";
			}

			currentfile << "\n";
			auto end = std::chrono::high_resolution_clock::now();
			auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
			t += elapsed.count() * 1e-9;
			std::cout << "posFinal " << posF << std::endl;
			//t++;
		}
		std::cout << "MOVE " << MOVE_POS[i] << " " << posF << std::endl;
	}
	*/
	return 0;
}
