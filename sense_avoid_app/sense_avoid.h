/************************************************************************************************************************
 * Author: Julian Torres
 * File: sense_avoid.h
 * Purpose:
 *      Header file for sense_avoid_app. Includes function prototypes for a PID controller, a feedback system plant, 
 *      a state machine, and an obstacle simulation as a virtual input.
 ***********************************************************************************************************************/
#ifndef SENSE_AVOID_H
#define SENSE_AVOID_H

#define _USE_MATH_DEFINES

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <ctime> // for calculating state machine frequency
#include <mutex> // For threading

#include <dronecode_sdk/dronecode_sdk.h>
#include <dronecode_sdk/plugins/action/action.h>
#include <dronecode_sdk/plugins/offboard/offboard.h>
#include <dronecode_sdk/plugins/telemetry/telemetry.h>

using namespace dronecode_sdk;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::seconds;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour


enum state_t {
    NORMAL,
    STOPPING,
    RISING, 
    FINISH
};

struct Obstacle {
    double latitude_deg;
    double longitude_deg;
    double bottomAltitude;
    double topAltitude;
};

// Handles Action's result (was inline)
void action_error_exit(Action::Result result, const std::string &message);

// Handles Offboard's result
void offboard_error_exit(Offboard::Result result, const std::string &message);

// Handles connection result
void connection_error_exit(ConnectionResult result, const std::string &message);

// Logs during Offboard control
void offboard_log(const std::string &offb_mode, const std::string msg);

void usage(std::string bin_name);

/**
 * Does Offboard control using NED co-ordinates.
 *
 * returns true if everything went well in Offboard control, exits with a log otherwise.
 */
bool offb_normal_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard, std::string offb_mode);

bool offb_stopping_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard, double present_value, std::string offb_mode);

bool offb_rising_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard, std::string offb_mode);



/*void StoppingController(std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
                        std::shared_ptr<dronecode_sdk::Offboard> offboard,
                            Obstacle obstacle, bool zero_velocity, bool &ret);*/

//int PID(int pid_input);

//int Plant(int plant_input);

//void GenerateClock();

/*void StateMachineInit(enum main_state_t &present_state, enum main_state_t &next_state, bool exit_condition, 
                bool object_detected, bool zero_velocity, bool clk);*/

/*void NextStateLogic(state_t present_state, state_t &next_state, 
                bool is_finished, bool object_detected, bool zero_velocity);

void OutputLogic(state_t present_state, bool &is_finished,
                std::shared_ptr<dronecode_sdk::Offboard> offboard);*/

double CalculateObstacleDistance(std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
                            Obstacle obstacle);

double CalculateDestinationDistance(std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
                            dronecode_sdk::Telemetry::Position destination);

// Make obstacle class with coordinates and height in altitude

double haversin(double angle);


#endif  /* SENSE_AVOID_H */
