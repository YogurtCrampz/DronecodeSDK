/************************************************************************************************************************
 * Author: Julian Torres
 * File: sense_avoid.h
 * Purpose:
 *      Header file for sense_avoid_app. Includes function prototypes for a PID controller, a feedback system plant, 
 *      a state machine, and an obstacle simulation as a virtual input.
 ***********************************************************************************************************************/

#include <chrono>
#include <cstdint>
#include <dronecode_sdk/dronecode_sdk.h>
#include <dronecode_sdk/plugins/action/action.h>
#include <dronecode_sdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <thread>

using namespace dronecode_sdk;
using namespace std::this_thread;
using namespace std::chrono;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour


enum state {
    NORMAL,
    STOPPING,
    RISING
};

//int PID(int pid_input);

//int Plant(int plant_input);

//enum state StateMachine(enum state present_state)



