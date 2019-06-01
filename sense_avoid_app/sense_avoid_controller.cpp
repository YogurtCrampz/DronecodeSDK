#ifndef SENSE_AVOID_H
#include "sense_avoid.h"
#endif

#ifndef PID_H
#include "PID.h"
#endif


void action_error_exit(Action::Result result, const std::string &message)
{
    if (result != Action::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << Action::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

void offboard_error_exit(Offboard::Result result, const std::string &message)
{
    if (result != Offboard::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << Offboard::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

void connection_error_exit(ConnectionResult result, const std::string &message)
{
    if (result != ConnectionResult::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << connection_result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

void offboard_log(const std::string &offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

bool offb_normal_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard, std::string offb_mode)
{
    /*offboard_log(offb_mode, "Turn to face East");
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 90.0f});
    sleep_for(seconds(2)); // Let yaw settle*/  // add another state for settling yaw?

    // Go West 100m 
    offboard_log(offb_mode, "Normal Sequence Velocity Command");
    //offboard->set_velocity_ned({0.0f, 5.0f, 0.0f, 90.0f});
    offboard->set_velocity_ned({0.0f, 10.0f, 0.0f, 90.0f});
    //sleep_for(seconds(4)); // it'll do other stuff in the mean time

    return true;
}

bool offb_stopping_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard, double present_value, std::string offb_mode) {
    float vel_eastf = float(present_value);

    // Implement velocity command for stopping sequence
    offboard_log(offb_mode, "Stopping Sequence Velocity Command");
    offboard->set_velocity_ned({0.0f, vel_eastf, 0.0f, 0.0f});
    //sleep_for(seconds(4)); 

    return true;
};

bool offb_rising_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard, std::string offb_mode) {
    float vel_risef = float(-1) * 5.0f;

    // Raise altitude until object is no longer seen
    offboard_log(offb_mode, "Rising Sequence Velocity Command");
    offboard->set_velocity_ned({0.0f, 0.0f, vel_risef, 0.0f});
    //sleep_for(seconds(4));

    return true;
};

bool offb_traversing_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard, std::string offb_mode) {
    float vel_forwardf = 5.0f;

    // Go forward until down sensor picks up something within maybe 5 meters then turn on altitude controller
    // Then have semi-normal state?
    offboard_log(offb_mode, "Traversing Sequence Velocity Command");
    offboard->set_velocity_ned({0.0f, vel_forwardf, 0.0f, 0.0f});
    //sleep_for(seconds(4));

    return true;
}



bool ObstacleCleared(std::shared_ptr<dronecode_sdk::Telemetry> telemetry, Obstacle obstacle) {
    // Calculate longitude of obstacle ending point: angle_deg = (180/pi) * s / R
    double earth_radius = 6.371e+6; // in meters
    double obst_end_angle_deg = obstacle.longitude_deg + (180 / M_PI) * obstacle.length / earth_radius;
    double obst_end_buffer_angle_deg = obstacle.longitude_deg + (180 / M_PI) * 2 / earth_radius; // Provides 2 m of clearance.
    if(telemetry->position().longitude_deg >= (obst_end_angle_deg + obst_end_buffer_angle_deg))
        return true;
    else 
        return false;
}

/********************************************************************************/

double CalculateObstacleDistance(std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
                            Obstacle obstacle) { 
    // Need for Feedback Controller
    dronecode_sdk::Telemetry::Position currentPosition = telemetry->position();
    
    //double copterLatitude = currentPosition.latitude_deg * (M_PI/180); // radians
    double copterLongitude = currentPosition.longitude_deg * (M_PI/180);

    //double obstacleLatitude = obstacle.latitude_deg * (M_PI/180); // radians
    double obstacleLongitude = obstacle.longitude_deg * (M_PI/180);

    double earth_radius = 6.371e+6; // in meters
    /*double haversin_alpha = haversin(copterLatitude - obstacleLatitude) 
                    + cos(copterLatitude) * cos(obstacleLatitude) 
                    * haversin(copterLongitude - obstacleLongitude);
    // theta = lattitude, phi = longitude

    double obstacleDistance = 2*earth_radius*asin(sqrt(haversin_alpha));*/

    double obstacleDistance = earth_radius * (obstacleLongitude - copterLongitude);

    return obstacleDistance;
}

double CalculateDestinationDistance(std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
                                dronecode_sdk::Telemetry::Position destination) {
    // Need to tell when application should finish
    dronecode_sdk::Telemetry::Position currentPosition = telemetry->position();

    //double copterLatitude = currentPosition.latitude_deg * (M_PI/180); // radians
    double copterLongitude = currentPosition.longitude_deg * (M_PI/180);

    //double destinationLatitude = destination.latitude_deg * (M_PI/180);
    double destinationLongitude = destination.longitude_deg * (M_PI/180);

    double earth_radius = 6.371 * (10 ^ 6); // in meters
    /*double haversin_alpha = haversin(copterLatitude - destinationLatitude) 
                    + cos(copterLatitude) * cos(destinationLatitude) 
                    * haversin(copterLongitude - destinationLongitude);
    // theta = lattitude, phi = longitude

    double destinationDistance = 2*earth_radius*asin(sqrt(haversin_alpha));*/

    double destinationDistance = earth_radius * (destinationLongitude - copterLongitude);

    return destinationDistance;
}

double haversin(double angle) { // need exception?  // radians
    double result = (1-cos(angle)) / 2;
    // or pow(sin(angle/2), 2);
    return result;
}
