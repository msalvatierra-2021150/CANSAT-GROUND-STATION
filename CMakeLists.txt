cmake_minimum_required(VERSION 3.16.0)

# ESP-IDF finds the 'main' folder by default. 
# Only use EXTRA_COMPONENT_DIRS if your code is in a weirdly named folder.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(CansatGroundStation)
