/*
 * Author: Julian Torres
 * File Name: sense_avoid_functions.cpp
 * Purpose:
 *      Necessary functions for Offboard mode control, copter movement, front-facing and down-facing distance sensor simulations, 
 *      and simple helper functions 
 * References:  Dronecode example offboard_velocity.cpp written by Julian Oes <julian@oes.ch>, Shakthi Prashanth <shakthi.prashanth.m@intel.com>
 */


#ifndef SENSE_AVOID_H
#include "sense_avoid.h"
#endif

#ifndef PID_H
#include "pid.h"
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

/********************************************* My Functions **********************************************************************/

// Copter Movement Control Functions

bool offb_normal_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard, std::string offb_mode, double down_present_value)
{
    float vel_downf = float(down_present_value);

    // Go Eest 100m 
    offboard_log(offb_mode, "Normal Sequence Velocity Command");
    
    offboard->set_velocity_ned({0.0f, 5.0f, vel_downf, 90.0f});

    return true;
}

bool offb_avoidance_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard, std::string offb_mode,
                            double front_present_value) {
    float vel_downf = float(-1) * 5.0f;
    float vel_eastf = float(front_present_value);
    
    // Implement velocity command for Avoidance sequence
    offboard_log(offb_mode, "Avoidance Sequence Velocity Command");
    offboard->set_velocity_ned({0.0f, vel_eastf, vel_downf, 90.0f}); 

    return true;
};

bool offb_settle_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard, std::string offb_mode,
                            double down_present_value) {
    float vel_downf = float(down_present_value);
    float vel_eastf = 5.0f;

    // Implement velocity command for Settling sequence
    offboard_log(offb_mode, "Settle Sequence Velocity Command");
    offboard->set_velocity_ned({0.0f, vel_eastf, vel_downf, 90.0f});

    return true;
};


// Sensor Simulation Functions

    // Front Facing Sensor
double CalculateObstacleDistance(double copter_longitude_deg, double copter_altitude_m, std::array<Obstacle, 2> obstacle_list) {     
    double earth_radius = 6.371e+6; // in meters
    double copter_longitude_rad = copter_longitude_deg * (M_PI/180);
    double obstacle_longitude_rad;
    double obstacleDistance;
    Obstacle detectedObstacle;
    bool obstacleDetectionFlag = false;

    for (int i = 0; i < int(obstacle_list.size()); i++) {
        if( ( (copter_altitude_m >= obstacle_list[i].bottomAltitude) && (copter_altitude_m <= obstacle_list[i].topAltitude) ) 
            && (copter_longitude_deg <= obstacle_list[i].longitude_deg) ) {
            obstacleDetectionFlag = true;
            detectedObstacle = obstacle_list[i];
            obstacle_longitude_rad = detectedObstacle.longitude_deg * (M_PI/180);
            obstacleDistance = earth_radius * (obstacle_longitude_rad - copter_longitude_rad);
        }       
    }
    if(!obstacleDetectionFlag) // pointers convert to boolean false if they have a value of NULL and positive if they have something else
        obstacleDistance = -1; // IsObstacleDetected() is false if object distance is not between 0 and 40

    // Haversin is used for changes in latitude and longitude, I am just doing longitude so it's unecessary
    /*double haversin_alpha = haversin(copterLatitude - obstacleLatitude) 
                    + cos(copterLatitude) * cos(obstacleLatitude) 
                    * haversin(copterLongitude - obstacleLongitude);
    // theta = lattitude, phi = longitude

    double obstacleDistance = 2*earth_radius*asin(sqrt(haversin_alpha));*/

    return obstacleDistance;
}

    // Down Facing Sensor
double CalculateGroundDistance(double copter_longitude_deg, double copter_altitude_m, std::array<Obstacle, 2> obstacle_list) {
    /*  Notes:
            The physical distance sensor would simply return a distance, but to simulate it you need to check 
        whether the copter coordinates overlap with any obstacles. If they do overlap then, the ground distance
        is calculated with current altitude - obstacle height, where the current obstacle is the highest one
        it is trying to avoid that it overlaps with.
    */
    
    double maxHeight;
    double downDistance;
    std::vector<Obstacle> overlapped_obstacles;
    
    // Check if current coordinates overlap with obstacle's
    for (int i = 0; i < 2; i++) {
        // Filter obstacles that overlap with copter
        if( (copter_longitude_deg >= obstacle_list[i].longitude_deg) 
            && (copter_longitude_deg <= (obstacle_list[i].longitude_deg + ArcLengthToAngleDeg(obstacle_list[i].length) ) ) )
            overlapped_obstacles.push_back(obstacle_list[i]);
    }
    // Find obstacle with max altitude from remaining obstacles
    if(overlapped_obstacles.size() == 0)
        downDistance = copter_altitude_m;
    else {
        maxHeight = 0;
        for (int i = 0; i < int(overlapped_obstacles.size()); i++) {
            if (overlapped_obstacles.front().topAltitude >= maxHeight) {
                maxHeight = overlapped_obstacles.back().topAltitude;
                overlapped_obstacles.pop_back();
            }   
        }
        downDistance = copter_altitude_m - maxHeight;
    }
    
    return downDistance;
}

bool IsObstacleDetected(double copter_longitude_deg, double copter_altitude_m, std::array<Obstacle, 2> obstacle_list) {
    double obstacle_distance = CalculateObstacleDistance(copter_longitude_deg, copter_altitude_m, obstacle_list);
    bool within_distance = (obstacle_distance <= 40.0) && (obstacle_distance >= 0); 
    bool is_detected = within_distance;
    
    return is_detected;
}

// Conversions

double ArcLengthToAngleDeg(double length) {
    double earth_radius = 6.371e+6; // in meters
    double angle_deg = (length / earth_radius) * (180/M_PI);

    return angle_deg;
}

double ArcLengthToAngleRad(double length) {
    double earth_radius = 6.371e+6; // in meters
    double angle_deg = (length / earth_radius) * (180/M_PI);

    return angle_deg;
}

double AngleRadToArcLength(double angle_rad) {
    double earth_radius = 6.371e+6; // in meters
    double arc_length = earth_radius * angle_rad;

    return arc_length;
}

double AngleDegToArcLength(double angle_deg) {
    double earth_radius = 6.371e+6; // in meters
    double arc_length = earth_radius * angle_deg * (M_PI/180);

    return arc_length;
}

// Not used in this implementation
double haversin(double angle) { // need exception?  // radians
    double result = (1-cos(angle)) / 2;
    // or pow(sin(angle/2), 2);
    return result;
}
