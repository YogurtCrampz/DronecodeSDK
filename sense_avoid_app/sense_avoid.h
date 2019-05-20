/************************************************************************************************************************
 * Author: Julian Torres
 * File: sense_avoid.h
 * Purpose:
 *      Header file for sense_avoid_app. Includes function prototypes for a PID controller, a feedback system plant, 
 *      a state machine, and an obstacle simulation as a virtual input.
 ***********************************************************************************************************************/

#include <chrono>
#include <cmath>
#include <dronecode_sdk/dronecode_sdk.h>
#include <dronecode_sdk/plugins/action/action.h>
#include <dronecode_sdk/plugins/mission/mission.h>
#include <dronecode_sdk/plugins/offboard/offboard.h>
#include <dronecode_sdk/plugins/telemetry/telemetry.h>

#include <functional>
#include <future>
#include <iostream>
#include <thread>
#include <memory>

using namespace dronecode_sdk;
using namespace std::placeholders; // for `_1`
using namespace std::chrono; // for seconds(), milliseconds()
using namespace std::this_thread; // for sleep_for()

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour


enum state {
    NORMAL,
    STOPPING,
    RISING
};


int PID(int pid_input);

int Plant(int plant_input);

enum state StateMachine(enum state present_state);

int SimulateObstacle();



