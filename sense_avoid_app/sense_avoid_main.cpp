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


    // Start operation
    state_t present_state = NORMAL, next_state = NORMAL;
    bool is_finished = false, object_detected = false, zero_velocity = false, height_is_reached = false, object_is_cleared = false;
    bool normal_is_on = false, stopping_is_on = false, rising_is_on = false, traversing_is_on = false;
    bool ret = false;

    dronecode_sdk::Telemetry::Position start = telemetry->position();
    dronecode_sdk::Telemetry::Position destination = start;
    destination.longitude_deg = start.longitude_deg + 0.0017986; // about 100m west of start position, 200m = 0.0017986 degrees 100 m = 0.0008993 degrees
    double obstacle_longitude_deg = start.longitude_deg + (destination.longitude_deg - start.longitude_deg)/3;
    Obstacle obstacle = {start.latitude_deg, obstacle_longitude_deg, 10, 20, 10};
    
    PID pid_front_facing(0.0220, 10, -10, 0.1, 5, 0.01, 2);
    //PID pid_down_facing(0.0220, 10, -10, 0.1, 5, 0.01, 2);
    double front_pres_val, front_set_val, front_pid_output;
    //double down_pres_val, down_set_val, down_pid_output;

    std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> timeStart, timeEnd;
    std::chrono::duration<double> elapsed_seconds;


    // Clock / Memory (main) thread
    while(!is_finished) {
        // Start timer for delay
        timeStart = std::chrono::system_clock::now();
        timeEnd = std::chrono::system_clock::now();

        // Output Logic (Output Commands based on current state)
        switch(present_state) {
            case NORMAL:
                stopping_is_on = false;
                rising_is_on = false;
                traversing_is_on = false;
                if(!normal_is_on) {
                    ret = offb_normal_ctrl_ned(offboard, offb_mode);
                    normal_is_on = true;
                } 
                break;
            case STOPPING:
                normal_is_on = false;
                rising_is_on = false;
                traversing_is_on = false;
                if(!stopping_is_on) {
                    // initialize controller
                    stopping_is_on = true;
                    front_pres_val = CalculateObstacleDistance(telemetry, obstacle);
                    front_set_val = 3;
                }
                else if(stopping_is_on && !zero_velocity) {
                    // velocity command
                    ret = offb_stopping_ctrl_ned(offboard, front_pid_output, offb_mode);
                    
                    // Present Value is given by obstacle distance 
                    front_pres_val = CalculateObstacleDistance(telemetry, obstacle);

                    // update controller
                    front_pid_output = pid_front_facing.Calculate(front_set_val, front_pres_val); 
                }
                else if(stopping_is_on && zero_velocity) {
                    // end controller
                        // do nothing
                }
                // Temporary Debug Info
                std::cout << "Obstacle Distance: " << CalculateObstacleDistance(telemetry, obstacle) << std::endl 
                            << "Front PID Output Value: " << front_pid_output << std::endl;
                    
                break;
            case RISING:
                normal_is_on = false;
                stopping_is_on = false;
                traversing_is_on = false;
                if(!rising_is_on) {
                    ret = offb_rising_ctrl_ned(offboard, offb_mode);
                    rising_is_on = true;
                }
                break;
            case TRAVERSE:
                normal_is_on = false;
                stopping_is_on = false;
                rising_is_on = false;
                if(!traversing_is_on) {
                    ret = offb_traversing_ctrl_ned(offboard, offb_mode);
                    traversing_is_on = true;
                }
                break;
            case FINISH:
                is_finished = true;
                break;
        }

        if (ret == false) // throw error if there is an issue with sending velocity commands
            return EXIT_FAILURE;
        
        // Get New Data (Update variables/Inputs to Next state logic)
        if((CalculateObstacleDistance(telemetry,obstacle) <= 40.0) && (telemetry->position().longitude_deg <= destination.longitude_deg))
            object_detected = true;
        else
            object_detected = false;
        if((fabs(telemetry->ground_speed_ned().velocity_east_m_s) <= 0.1f) && (fabs(CalculateObstacleDistance(telemetry, obstacle) - front_set_val) <= 0.01))
            zero_velocity = true;
        else
            zero_velocity = false;
        if(telemetry->position().relative_altitude_m >= 5.0f + float(obstacle.topAltitude))
            height_is_reached = true;
        else 
            height_is_reached = false;
        if(ObstacleCleared(telemetry, obstacle))
            object_is_cleared = true;
        else 
            object_is_cleared = false;

        // Next State Logic
        next_state = present_state; // default next state
        switch(present_state) {
            case NORMAL:
                if(telemetry->position().longitude_deg >= destination.longitude_deg) // if destination is reached or if current longitude is 
                    next_state = FINISH;                                            // greater than destination longitude
                else if(object_detected)    // object within 40 meters
                    next_state = STOPPING;
                break;
            case STOPPING:
                if(telemetry->position().longitude_deg >= destination.longitude_deg)
                    next_state = FINISH;
                else if(zero_velocity)
                    next_state = RISING;
                break;
            case RISING:
                if(telemetry->position().longitude_deg >= destination.longitude_deg)
                    next_state = FINISH;
                else if(height_is_reached)
                    next_state = TRAVERSE;
                break;
            case TRAVERSE:
                if(telemetry->position().longitude_deg >= destination.longitude_deg)
                    next_state = FINISH;
                else if(object_is_cleared)
                    next_state = NORMAL;
                break;
            case FINISH:
                next_state = FINISH;
                break;
        }
        
        // Memory (Update present state)
        present_state = next_state;

        // Delay
        elapsed_seconds = timeEnd-timeStart;
        while(elapsed_seconds.count() <= 0.020) {
            timeEnd = std::chrono::system_clock::now();
            elapsed_seconds = timeEnd-timeStart;
        }

        //std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

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

