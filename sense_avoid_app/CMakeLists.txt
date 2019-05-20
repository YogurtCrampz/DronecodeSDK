cmake_minimum_required(VERSION 2.8.12)

# Specify your project's name
project(sense_avoid_app)

# Specify at least C++11
add_definitions("-std=c++11")

# Enable strict handling of warnings
add_definitions("-Wall -Wextra -Werror")

# In case /usr/local/include is not in your default includes
#include_directories(/usr/local/include)

# In case /usr/local/lib is not in your default linker path
#link_directories(/usr/local/lib)

# Specify your app's executable name, and list of source files used to create it.
add_executable(sense_avoid_app
    sense_avoid.h
    sense_avoid_main.cpp
    sense_avoid_misc_funcs.cpp
    sense_avoid_controller_funcs.cpp
    # ... any other source files
)

# Specify your app's executable name and a list of linked libraries
target_link_libraries(sense_avoid_app
    dronecode_sdk          #All apps link against dronecode_sdk library
    dronecode_sdk_action      # If action plugin used by app ...
    dronecode_sdk_mission
    dronecode_sdk_telemetry   #If telemetry plugin used by app ...
    dronecode_sdk_offboard
    # ... Any other linked libraries
)