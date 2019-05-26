#ifndef SENSE_AVOID_H
#include "sense_avoid.h"
#endif


int PID(int pid_input) {
    return pid_input;
}

int Plant(int plant_input) {
    return plant_input;
}

/*void GenerateClock() { // state machine
    // while loop is master clock? 
    // How to measure if clock is at correct frequency? 
        // use system time library to snapshot times?
    // Need to protect clk?

    enum clk_state_t present_state, next_state;
    uint16_t count;

    // set start state
    present_state = LOW;

    // next state logic
    switch(present_state) {
        case LOW:
            if(count == 1000)
                next_state = HI;
            break;
        case HI:
            if(count == 1000)
                next_state = LOW;
            break;
    }
    // memory
    while(1) {
        count++;
    }
    
    // output logic
    switch(present_state) {
        case LOW:
            derived_clk = HI
    }
}*/

/*void StateMachineInit(enum main_state_t present_state, enum main_state_t next_state, bool exit_condition, 
                    bool object_detected, bool zero_velocity, bool clk) {
    // Do I need to have a lock on present_state, next_state to keep track of when writing it and reading from it?
    // I should create 4 threads to run concurrently? Yes
    // How do I run clk at 50? Use separate state machine for clock generator.

    // Set start state
    present_state = NORMAL;
    
    // Next state logic
    next_state = present_state; // default next state
    switch (present_state) {
        case NORMAL:
            // If end is nigh
            // next_state = FINAL;
            if(object_detected)
                next_state = STOPPING;
            break;
        case STOPPING:
            if(zero_velocity)
                next_state = RISING;
            break;
        case RISING:
            if(!object_detected)
                next_state = NORMAL;
            break;
    }

    // Memory
    if(clk)
        present_state = next_state;

    // Output Logic
        // turn all off?
    switch(present_state) {
        case NORMAL:
            // call normal operation ned func.
            break;
        case STOPPING:
            // call stopping operation ned func.
            break;
        case RISING:
            // call rising operation ned func.
            break;
    }
    
}*/

int SimulateObstacle() {
    return 0;
}

