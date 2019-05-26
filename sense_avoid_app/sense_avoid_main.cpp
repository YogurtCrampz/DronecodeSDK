/**
 * @file offboard_velocity.cpp
 * @brief Example that demonstrates offboard velocity control in local NED and body coordinates
 *
 * @authors Author: Julian Oes <julian@oes.ch>,
 *                  Shakthi Prashanth <shakthi.prashanth.m@intel.com>
 * @date 2017-10-17
 */

#ifndef SENSE_AVOID_H
#include "sense_avoid.h"
#endif

// Handles Action's result
inline void action_error_exit(Action::Result result, const std::string &message)
{
    if (result != Action::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << Action::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result, const std::string &message)
{
    if (result != Offboard::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << Offboard::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void connection_error_exit(ConnectionResult result, const std::string &message)
{
    if (result != ConnectionResult::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << connection_result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Logs during Offboard control
inline void offboard_log(const std::string &offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

/**
 * Does Offboard control using NED co-ordinates.
 *
 * returns true if everything went well in Offboard control, exits with a log otherwise.
 */
bool offb_normal_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard);

bool offb_stopping_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard);

bool offb_rising_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard);


void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

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



    // Start operation
    main_state_t present_state, next_state;
    bool object_detected = false, zero_velocity = false;
    uint16_t count = 0;
    //bool clk = false;
    bool exit_condition = false;
    bool ret = false;

    present_state = NORMAL; // Initialize state machine

    // Start state machine, which is 3 separate threads running continuously?
    //StateMachineInit(present_state, next_state, exit_condition,
      //                  object_detected, zero_velocity, clk);

    // Start clock thread(s)
    while(1) {
        if(count == 999) {
            present_state = next_state;
            count = 0;

            // Need to measure time
            /*auto start = std::chrono::system_clock::now();
            // Some computation here
            auto end = std::chrono::system_clock::now();

            std::chrono::duration<double> elapsed_seconds = end-start;
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);

            std::cout << "finished computation at " << std::ctime(&end_time)
                    << "elapsed time: " << elapsed_seconds.count() << "s\n";*/
        }
        
        // next state logic
        switch(present_state) {
            case NORMAL:
                // If end is nigh
                if(false)
                    next_state = FINISH;
                else if(object_detected)
                    next_state = STOPPING;
                break;
            case STOPPING:
                if(false)
                    next_state = FINISH;
                else if(zero_velocity)
                    next_state = RISING;
                break;
            case RISING:
                if(false)
                    next_state = FINISH;
                else if(!object_detected)
                    next_state = NORMAL;
                break;
            case FINISH:
                next_state = FINISH;
                break;
        }

        // Output Logic
        switch(present_state) {
            case NORMAL:
                // call normal operation ned func.
                ret = offb_normal_ctrl_ned(offboard);
                if (ret == false)
                    return EXIT_FAILURE;
                break;
            case STOPPING:
                // call stopping operation ned func.
                ret = offb_stopping_ctrl_ned(offboard);
                if (ret == false)
                    return EXIT_FAILURE;
                break;
            case RISING:
                // call rising operation ned func.
                ret = offb_rising_ctrl_ned(offboard);
                if (ret == false)
                    return EXIT_FAILURE;
                break;
            case FINISH:
                exit_condition = true;
                break;
        }


        if(exit_condition)
            break;
        count++;
    }

    // join threads

    //  using local NED co-ordinates is output of state machine
    /*ret = offb_normal_ctrl_ned(offboard);
    if (ret == false) {
        return EXIT_FAILURE;
    }*/



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

bool offb_normal_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard)
{
    const std::string offb_mode = "NED";
    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});

    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");

    /*offboard_log(offb_mode, "Turn to face East");
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 90.0f});
    sleep_for(seconds(1)); // Let yaw settle.*/

    /*{
        const float step_size = 0.01f;
        const float one_cycle = 2.0f * (float)M_PI;
        const unsigned steps = 2 * unsigned(one_cycle / step_size);

        offboard_log(offb_mode, "Go North and back South");
        for (unsigned i = 0; i < steps; ++i) {
            float vx = 5.0f * sinf(i * step_size);
            offboard->set_velocity_ned({vx, 0.0f, 0.0f, 90.0f});
            sleep_for(milliseconds(10));
        }
    }*/

    /*offboard_log(offb_mode, "Turn to face West");
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 270.0f});
    sleep_for(seconds(2));*/

    /*offboard_log(offb_mode, "Go up 2 m/s, turn to face South");
    offboard->set_velocity_ned({0.0f, 0.0f, -2.0f, 180.0f});
    sleep_for(seconds(4));*/

    /*offboard_log(offb_mode, "Go down 1 m/s, turn to face North");
    offboard->set_velocity_ned({0.0f, 0.0f, 1.0f, 0.0f});
    sleep_for(seconds(4));*/

    // Go North 100m 
    offboard_log(offb_mode, "Go north 100m");
    offboard->set_velocity_ned({5.0f, 0.0f, 0.0f, 0.0f});
    sleep_for(seconds(4)); // or while loop?

    // Now, stop offboard mode.
    offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
    offboard_log(offb_mode, "Offboard stopped");

    return true;
}

bool offb_stopping_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard) {
    const std::string offb_mode = "NED";
    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});

    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");


    // Go North 100m 
    offboard_log(offb_mode, "Go north 100m");
    offboard->set_velocity_ned({5.0f, 0.0f, 0.0f, 0.0f});
    sleep_for(seconds(4)); // or while loop?

    // Now, stop offboard mode.
    offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
    offboard_log(offb_mode, "Offboard stopped");

    return true;
};

bool offb_rising_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard) {
    const std::string offb_mode = "NED";
    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});

    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");


    // Go North 100m 
    offboard_log(offb_mode, "Go north 100m");
    offboard->set_velocity_ned({5.0f, 0.0f, 0.0f, 0.0f});
    sleep_for(seconds(4)); // or while loop?

    // Now, stop offboard mode.
    offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
    offboard_log(offb_mode, "Offboard stopped");

    return true;
};
