#include "Robot.h"




Robot::Robot(std::string file) {
	readFileConfig(file);

	for(int i = 0; i < portMotor.size(); i++)
		initialize(portMotor[i]);

	std::map<int, std::vector<std::string> >::iterator it;
	std::map<int, std::vector<int> >::iterator itRange;
	std::vector<int> IDxErase;

	for (it = robotMotors.begin(); it != robotMotors.end(); ++it) {
		std::cout << it->first << " " << it->second[0] << std::endl;
		if (ping(it->first, it->second[0]) == -1) {
			IDxErase.push_back(it->first);
		}
		else {
			set_torque(it->first, it->second[0], 1);
		}
	}

	for (int i = 0; i < IDxErase.size(); i++) {
		robotMotors.erase(IDxErase[i]);
		motorRange.erase(IDxErase[i]);
	}

	std::cout << "Motor Initialize \n";
	
	for (itRange = motorRange.begin(); itRange != motorRange.end(); ++itRange) {
		std::cout << itRange->first << "\t " << itRange->second[2] << std::endl;
		set_position(itRange->first, robotMotors[itRange->first][0], itRange->second[2]);
		std::this_thread::sleep_for(std::chrono::milliseconds(TIME_SLEEP));
	}
	
	startTime = std::chrono::high_resolution_clock::now();
	
}

std::vector<std::string> Robot::split(const std::string& s, char delimiter)
{
	std::string aux;
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream(s);
	while (std::getline(tokenStream, aux, '\t'));

	std::istringstream tokenStream2(aux);
	while (std::getline(tokenStream2, aux, ' '));

	std::istringstream tokenStream3(aux);
	while (std::getline(tokenStream3, token, delimiter))
	{
		tokens.push_back(token);
	}
	return tokens;
}


bool Robot::readFileConfig(std::string filename) {
	std::string line;
	std::string segment;
	std::string DEVICENAME;
	std::vector<int> IDs;
	std::string aux;
	std::vector<std::string> seglist;

	std::ifstream fileConf;
	fileConf.open(filename);
	if (!fileConf.is_open())
	{
		return false;
	}

	//std::getline(fileConf, line);
	std::getline(fileConf, line, '\n');
	seglist = split(line, ':');
	RobotName = seglist[1];

	std::getline(fileConf, line, '\n');
	while (line.compare("SENSOR:") == 0) {
		std::getline(fileConf, line, '\n');
		seglist = split(line, ':');			// Sensor Port
		aux = seglist[1];

		std::getline(fileConf, line, '\n');
		seglist = split(line, ':');			// Sensor Name
		
		robotSensors.insert(std::pair<std::string, std::string>(seglist[1], aux));

		std::getline(fileConf, line, '\n');
	}

	if (line.compare("MOTOR:") == 0) {
		std::getline(fileConf, line, '\n');
		seglist = split(line, ':');
		DEVICENAME = seglist[1];		// Motor PORT
		portMotor.push_back(DEVICENAME);

		std::getline(fileConf, line, '\n');
		seglist = split(line, ':');
		while (seglist.size() == 1) {
			aux = seglist[0];			// Body 

			std::getline(fileConf, line, '\n');
			seglist = split(line, ':');		
			
			while (seglist.size() > 1) {
				int DXL_ID = std::stoi(seglist[0]);		// ID MOTOR
				std::cout << DXL_ID << " lendo " << std::endl;

				int pos_min = std::stoi(seglist[2]);
				int pos_max = std::stoi(seglist[3]);
				int pos		= std::stoi(seglist[4]);

				if (!(pos_min >= MIN_POSITION_MOTOR && pos_min <= MAX_POSITION_MOTOR && pos_min < pos_max)){
					std::cout << DXL_ID << " ERROR POS_MIN" << std::endl;
					pos_min = MIN_POSITION_MOTOR;
				}
				if (!(pos_max >= MIN_POSITION_MOTOR && pos_max <= MAX_POSITION_MOTOR)) {
					std::cout << DXL_ID << " ERROR POS_MAX" << std::endl;
					pos_max = MAX_POSITION_MOTOR;
				}
				if (!(pos >= pos_min && pos <= pos_max)) {
					std::cout << DXL_ID << " ERROR POS_INIT" << std::endl;
					pos = pos_min;
				}

				std::vector<int> range = { pos_min, pos_max, pos };
				std::vector<std::string>  portMotorName = { DEVICENAME, seglist[1] };
				IDs.push_back(DXL_ID);

				robotMotors.insert(std::pair<int, std::vector<std::string> >(DXL_ID, portMotorName));
				motorRange.insert(std::pair<int, std::vector<int> >(DXL_ID, range));

				std::getline(fileConf, line, '\n');
				seglist = split(line, ':');
			}

			bodyMotor.insert(std::pair<std::string, std::vector<int> >(aux, IDs));
			IDs.clear();
		}

	}


	fileConf.close();
	return true;
}

bool Robot::writeFile(std::string fileee) {
	std::ofstream currentFile;
	if (fileNameWrite.compare(fileee) != 0) {
		currentFile.open(fileee);
		currentFile << "time,motor,position,current\n";
		fileNameWrite = fileee;
	}
	else {
		currentFile.open(fileee, std::ios_base::app);
	}

	std::map<int, std::vector<std::string> >::iterator it;
	float tt;

	for (it = robotMotors.begin(); it != robotMotors.end(); ++it) {
		auto t = std::chrono::high_resolution_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t - startTime);
		tt = elapsed.count() * 1e-9;
		currentFile << tt << "," << it->first << ",";
		currentFile << get_present_position(it->first, it->second[0]) << ",";
		currentFile << get_present_current(it->first, it->second[0]) << "\n";
	}

	currentFile.close();
	return true;

}

bool Robot::moveMotors(std::vector<int> vecId, std::vector<int> vecPos) {
	if (vecId.size() != vecPos.size())
		return true;

	bool resultant = false;
	for (int i = 0; i < vecId.size(); i++) {
		if (robotMotors.count(vecId[i])) {
			if (vecPos[i] < motorRange[vecId[i]][0]) {
				vecPos[i] = motorRange[vecId[i]][0];
			}
			if (vecPos[i] > motorRange[vecId[i]][1]) {
				vecPos[i] = motorRange[vecId[i]][1];
			}
			set_position(vecId[i], robotMotors[vecId[i]][0], vecPos[i]);

			resultant = resultant || (abs(vecPos[i] - get_present_position(vecId[i], robotMotors[vecId[i]][0])) > THRESHOLD);
			//std::cout << vecId[i] << " " << vecPos[i] << " " << resultant << std::endl;
		}
	}

	/*
	if (abs(vecPos[0] - get_present_position(vecId[0], robotMotors[vecId[0]][0])) > 15) {
		return false;
	}
	else {
		return true;
	}*/
	return resultant;
}

bool Robot::readFileMove(std::string filename) {
	std::string line;
	std::string segment;
	std::string fileWrite = "Write_" + filename;
	std::vector<std::string> seglist;
	std::vector<std::string> seglistAux;

	std::ifstream fileMove;
	fileMove.open(filename);
	if (!fileMove.is_open())
	{
		return false;
	}

	//std::getline(fileConf, line);
	bool resultant = false;
	std::vector<int> vecIDs;
	std::vector<int> vecPos;


	while (std::getline(fileMove, line, '\n')) {
		seglist = split(line, '|');
		for (int i = 0; i < seglist.size(); i++) {
			seglistAux = split(seglist[i], '=');
			vecIDs.push_back(std::stoi(seglistAux[0]));
			vecPos.push_back(std::stoi(seglistAux[1]));
		}

		bool aux = false;
		do {
			aux = moveMotors(vecIDs, vecPos);
			writeFile(fileWrite);
		} while (aux);

		vecIDs.clear();
		vecPos.clear();
			/*
			if (robotMotors.count(DXL_ID)) {
				if (pos < motorRange[DXL_ID][0]) {
					pos = motorRange[DXL_ID][0];
				}
				if (pos > motorRange[DXL_ID][1]) {
					pos = motorRange[DXL_ID][1];
				}
				set_position(DXL_ID, robotMotors[DXL_ID][0], pos);

				resultant = resultant && (abs(pos - get_present_position(DXL_ID, robotMotors[DXL_ID][0])) > 15);
			}*/

		
	}

	fileMove.close();
	return true;
}

