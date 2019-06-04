/**
 * @file offboard_velocity.cpp
 * @brief Example that demonstrates offboard velocity control in local NED and body coordinates
 *
 * @authors Author: Julian Oes <julian@oes.ch>,
 *                  Shakthi Prashanth <shakthi.prashanth.m@intel.com>
 * @date 2017-10-17
 * 
 * 
 * Author: Julian Torres
 * File Name: sense_avoid_main.cpp
 * Purpose:
 *      Implementation of a sense and avoid feature with simulated distance sensor input.
 *      Uses basic structure of:
 *          // Output Commands
 *          // Get New Data
 *          // Next Input
 *          // Wait for delay
 */

#ifndef SENSE_AVOID_H
#include "sense_avoid.h"
#endif

#ifndef PID_H
#include "PID.h"
#endif


int main(int argc, char **argv)
{
    DronecodeSDK dc;
    std::string connection_url;
    ConnectionResult connection_result;

    if (argc == 2) {
        connection_url = argv[1];
        connection_result = dc.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Connection failed: " << connection_result_str(connection_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Wait for the system to connect via heartbeat
    while (!dc.is_connected()) {
        std::cout << "Wait for system to connect via heartbeat" << std::endl;
        sleep_for(seconds(1));
    }

    // System got discovered.
    System &system = dc.system();
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);

    while (!telemetry->health_all_ok()) {
        std::cout << "Waiting for system to be ready" << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "System is ready" << std::endl;

    Action::Result arm_result = action->arm();
    action_error_exit(arm_result, "Arming failed");
    std::cout << "Armed" << std::endl;

    Action::Result takeoff_result = action->takeoff();
    action_error_exit(takeoff_result, "Takeoff failed");
    std::cout << "In Air..." << std::endl;
    sleep_for(seconds(5));


    // Start Offboard Mode
    const std::string offb_mode = "NED";
    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});

    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");





    // Start operation --------------------------------------------------------------------------------------------------------------------------------

    // Variable Declarations
    state_t present_state = NORMAL, next_state = NORMAL;
    bool is_finished = false, object_detected = false, is_settled = false, is_destination_reached = false;
    bool normal_is_on = false, avoidance_is_on = false, settle_is_on = false;
    bool ret = true;

    dronecode_sdk::Telemetry::Position start = telemetry->position();
    dronecode_sdk::Telemetry::Position destination = start;
    destination.longitude_deg = start.longitude_deg + ArcLengthToAngleDeg(250);

    double copter_longitude_deg = telemetry->position().longitude_deg; 
    double copter_relative_altitude_m = double(telemetry->position().relative_altitude_m);

    // Simulated Obstacles to simulate sensor data
    double obstacle1_longitude_deg = start.longitude_deg + ArcLengthToAngleDeg(50);
    Obstacle obstacle1 = {start.latitude_deg, obstacle1_longitude_deg, 0, 10, 150};
    double obstacle2_longitude_deg = obstacle1.longitude_deg + ArcLengthToAngleDeg(100);
    Obstacle obstacle2 = {start.latitude_deg, obstacle2_longitude_deg, 10, 20, 50};
    std::array<Obstacle, 2> obstacle_list = {obstacle1, obstacle2}; 
    
    // PID information
    PID pid_front_avoidance(0.020, 10, -10, 0.2, 5, 0.01, 2); //For front facing distance sensor
    PID pid_down_normal(0.020, 5, -5, 0.2, 5, 0.01, 2); // For downward facing distance sensor 0.2
    PID pid_down_settle(0.020, 5, -5, 0.2, 5, 0.01, 2); // For downward facing distance sensor 0.5

    double avoidance_pres_val, avoidance_set_val, avoidance_pid_output;
    double normal_pres_val, normal_set_val, normal_pid_output;
    double settle_pres_val, settle_set_val, settle_pid_output;

    //double predicted_obst_long;

    std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> timeStart, timeEnd;
    std::chrono::duration<double> elapsed_seconds;



    while(!is_finished) {
        // Start timer for delay
        timeStart = std::chrono::system_clock::now();
        timeEnd = std::chrono::system_clock::now();

        // Output Logic (Output Commands based on current state) ************************************************************************
        switch(present_state) {
            case NORMAL: // Use both controllers
                avoidance_is_on = false;
                settle_is_on = false;
                if(!normal_is_on) {
                    std::cout << TELEMETRY_CONSOLE_TEXT
                            << "In NORMAL state" 
                            << NORMAL_CONSOLE_TEXT << std::endl;
                    // Initialize controller
                    normal_is_on = true;
                    normal_set_val = 3.0;
                } 
                else { 
                    std::cout << "ground distance: " << CalculateGroundDistance(copter_longitude_deg, copter_relative_altitude_m, obstacle_list) << std::endl
                            << "copter altitude: " << copter_relative_altitude_m << std::endl 
                            << "Obstacle Distance: " << CalculateObstacleDistance(copter_longitude_deg, copter_relative_altitude_m, obstacle_list) << std::endl;
                    ret = offb_normal_ctrl_ned(offboard, offb_mode, normal_pid_output);
                }
                break;
            case AVOIDANCE:
                normal_is_on = false;
                settle_is_on = false;
                if(!avoidance_is_on) {
                    std::cout << TELEMETRY_CONSOLE_TEXT
                            << "In AVOIDANCE state" 
                            << NORMAL_CONSOLE_TEXT << std::endl;
                    // initialize controller
                    avoidance_is_on = true;
                    avoidance_set_val = 3.0;
                }
                else {
                    std::cout << "ground distance: " << CalculateGroundDistance(copter_longitude_deg, copter_relative_altitude_m, obstacle_list) << std::endl
                            << "copter altitude: " << copter_relative_altitude_m << std::endl
                            << "Obstacle Distance: " << CalculateObstacleDistance(copter_longitude_deg, copter_relative_altitude_m, obstacle_list) << std::endl;
                    ret = offb_avoidance_ctrl_ned(offboard, offb_mode, avoidance_pid_output);
                }   
                break;
            case SETTLE: // Second part of AVOIDANCE
                normal_is_on = false;
                avoidance_is_on = false;
                if(!settle_is_on) {
                    std::cout << TELEMETRY_CONSOLE_TEXT
                            << "In SETTLE state" 
                            << NORMAL_CONSOLE_TEXT << std::endl;
                    // Initialize settling
                    settle_is_on = true;
                    settle_set_val = double(copter_relative_altitude_m) + 3.0;
                    // grab current longitude and add 3m, have a boolean flag that detects when current longitude is greater than or equal to that value.
                    //predicted_obst_long = copter_longitude_deg + ArcLengthToAngleDeg(3.0);
                }
                else  {
                    std::cout << "ground distance: " << CalculateGroundDistance(copter_longitude_deg, copter_relative_altitude_m, obstacle_list) << std::endl
                            << "copter altitude: " << copter_relative_altitude_m << std::endl
                            << "Obstacle Distance: " << CalculateObstacleDistance(copter_longitude_deg, copter_relative_altitude_m, obstacle_list) << std::endl;
                    ret = offb_settle_ctrl_ned(offboard, offb_mode, settle_pid_output);
                }
                break;
            case FINISH:
                is_finished = true;
                break;
        }

        if (ret == false) // throw error if there is an issue with sending velocity commands
            return EXIT_FAILURE;
        

        // New Data ********************************************************************************************************************************
            // New copter data
        copter_longitude_deg = telemetry->position().longitude_deg;
        //copter_latitude_deg = telemetry->position().latitude_deg;
        copter_relative_altitude_m = double(telemetry->position().relative_altitude_m);

            // New PID Data
        avoidance_pres_val = CalculateObstacleDistance(copter_longitude_deg, copter_relative_altitude_m, obstacle_list);
        avoidance_pid_output = pid_front_avoidance.Calculate(avoidance_set_val, avoidance_pres_val);

        normal_pres_val = CalculateGroundDistance(copter_longitude_deg, copter_relative_altitude_m, obstacle_list);
        normal_pid_output = pid_down_normal.Calculate(normal_set_val, normal_pres_val);

        settle_pres_val = copter_relative_altitude_m;
        settle_pid_output = pid_down_settle.Calculate(settle_set_val, settle_pres_val);

    
            // Update Variables
        if( IsObstacleDetected(copter_longitude_deg, copter_relative_altitude_m, obstacle_list) )
            object_detected = true; // Detects if there is an object directly in front of copter picked up by distance sensor
        else
            object_detected = false;
        
        if( (fabs(settle_set_val - settle_pres_val) <= 0.2) && (fabs(normal_set_val - normal_pres_val) <= 0.2) ) 
            is_settled = true;
        else
            is_settled = false;

        if(copter_longitude_deg >= destination.longitude_deg)
            is_destination_reached = true;
        else
            is_destination_reached = false;


        // Next State Logic ***********************************************************************************************************************
        next_state = present_state; // default next state
        switch(present_state) {
            case NORMAL:
                if(is_destination_reached) // if current longitude is greater than destination longitude 
                    next_state = FINISH;                                            
                else if(object_detected)
                    next_state = AVOIDANCE;   
                break;
            case AVOIDANCE:
                if(is_destination_reached)
                    next_state = FINISH;
                else if(!object_detected)  
                    next_state = SETTLE;
                break;
            case SETTLE: // Need to check for obstacle again in case another obstacle is there immediately before settling finishes
                if(is_destination_reached)
                    next_state = FINISH;
                else if(object_detected)
                    next_state = AVOIDANCE;
                else if(is_settled) {
                    next_state = NORMAL;
                    /*std::cout << TELEMETRY_CONSOLE_TEXT 
                            << "relative_altitude_m: " << copter_relative_altitude_m << std::endl
                            << "settle_set_value: " << settle_set_val << std::endl
                            << NORMAL_CONSOLE_TEXT << std::endl;*/
                }    
                break;
            case FINISH:
                next_state = FINISH;
                break;
        }
        
        // Memory (Update present state) *********************************************************************************************************
        present_state = next_state;

        // Delay *********************************************************************************************************************************
        elapsed_seconds = timeEnd-timeStart;
        while(elapsed_seconds.count() <= 0.020) {
            timeEnd = std::chrono::system_clock::now();
            elapsed_seconds = timeEnd-timeStart;
        }

        std::cout << "Distance from Start: " << AngleDegToArcLength(copter_longitude_deg) - AngleDegToArcLength(start.longitude_deg) << std::endl;  

    }


    // Now, stop offboard mode.
    offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
    offboard_log(offb_mode, "Offboard stopped");


    const Action::Result land_result = action->land();
    action_error_exit(land_result, "Landing failed");

    // Check if vehicle is still in air
    while (telemetry->in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished..." << std::endl;

    return EXIT_SUCCESS;
}

