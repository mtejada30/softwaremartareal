#pragma once

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
#include <map>

#include "motorDynamixel.h"                                  // Uses Dynamixel SDK library


#define TIME_SLEEP						100			//100 MS
#define THRESHOLD						15			//Error of position maximum.

// Protocol version Dynamixel
#define PROTOCOL_VERSION                2.0         // See which protocol version is used in the Dynamixel

#define MAX_POSITION_MOTOR				4095		// 
#define MIN_POSITION_MOTOR				0			


class Robot
{
	public:
		std::string RobotName;
		std::string fileNameWrite;
		bool newFileNameWrite = true;
		std::chrono::high_resolution_clock::time_point startTime;

		std::vector<std::string> portMotor;
		std::map<std::string, std::string> robotSensors;		// sensor name, port
		std::map<int, std::vector<std::string> > robotMotors;	// motor ID, [PORT, motor name]
		std::map<int, std::vector<int> > motorRange;			// motor ID, [min, max, init]
		std::map<std::string, std::vector<int> > bodyMotor;		// body name, [id motor, id motor2, ...]

		Robot(std::string);

		bool moveMotors(std::vector<int>, std::vector<int>);
		bool readFileMove(std::string);
		bool writeFile(std::string);

	private:
		std::vector<std::string> split(const std::string& s, char delimiter);
		bool readFileConfig(std::string filename);
};

