#pragma once


/************************************************************/
/*                                                          */
/*              Initializer & Deinitializer                 */
/*                                                          */
/************************************************************/

int initialize(std::string deviceName);
void terminate(std::string deviceName);
bool initializeBulkRead(int DXL_ID, std::string deviceName);



/************************************************************/
/*                                                          */
/*                     Get functions                        */
/*                                                          */
/************************************************************/
int ping(int DXL_ID, std::string deviceName);
int get_bulk_present_position(int DXL_ID, std::string deviceName);
int get_bulk_present_current(int DXL_ID, std::string deviceName);

int get_present_position(int DXL_ID, std::string deviceName);
int get_firmware_version(int DXL_ID, std::string deviceName);
int get_led_status(int DXL_ID, std::string deviceName);
int get_present_current(int DXL_ID, std::string deviceName);
int get_present_velocity(int DXL_ID, std::string deviceName);
int get_input_voltage(int DXL_ID, std::string deviceName);
int get_present_temperature(int DXL_ID, std::string deviceName);
int get_present_torque(int DXL_ID, std::string deviceName);
int get_profile_acceleration(int DXL_ID, std::string deviceName);
int get_profile_velocity(int DXL_ID, std::string deviceName);
int get_moving_status(int DXL_ID, std::string deviceName);
int get_present_pwm(int DXL_ID, std::string deviceName);


/************************************************************/
/*                                                          */
/*                     Set functions                        */
/*                                                          */
/************************************************************/

void set_position(int DXL_ID, std::string deviceName, int goal_position);
void set_led_status(int DXL_ID, std::string deviceName, int led_status);
void set_torque(int DXL_ID, std::string deviceName, int torque_status);
void set_profile_acceleration(int DXL_ID, std::string deviceName, int profile_acceleration);
void set_profile_velocity(int DXL_ID, std::string deviceName, int profile_velocity);
void set_goal_pwm(int DXL_ID, std::string deviceName, int goal_pwm);


