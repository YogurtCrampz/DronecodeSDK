/************************************************************************************************************************
 * Author: Julian Torres
 * File: sense_avoid.h
 * Purpose:
 *      Header file for sense_avoid_app. Includes function prototypes for a PID controller, a feedback system plant, 
 *      a state machine, and an obstacle simulation as a virtual input.
 ***********************************************************************************************************************/
#ifndef SENSE_AVOID_H
#define SENSE_AVOID_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <ctime> // for calculating state machine frequency

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


enum main_state_t {
    NORMAL,
    STOPPING,
    RISING, 
    FINISH
};

int PID(int pid_input);

int Plant(int plant_input);

//void GenerateClock();

/*void StateMachineInit(enum main_state_t &present_state, enum main_state_t &next_state, bool exit_condition, 
                bool object_detected, bool zero_velocity, bool clk);*/

int SimulateObstacle();


#endif  /* SENSE_AVOID_H */
