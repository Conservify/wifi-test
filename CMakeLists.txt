cmake_minimum_required(VERSION 3.5)
project(ARDUINO_PROJECTS)

set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})

include(Arduino)

enable_arduino_toolchain()

file(GLOB sources *.cpp)

add_arduino_library(wifi-test "${sources}")

# enable_small_bootloader()

find_package(WiFi101)
target_link_libraries(wifi-test WiFi101)

find_package(arduino-logging)
target_link_libraries(wifi-test arduino-logging)

find_package(SerialFlash)
target_link_libraries(wifi-test SerialFlash)

option(FK_CORE "Apply FieldKit Core configuration.")
if(FK_CORE)
  add_definitions(-DFK_CORE)
endif(FK_CORE)

option(FK_NATURALIST "Apply FieldKit Naturalist configuration.")
if(FK_NATURALIST)
  add_definitions(-DFK_NATURALIST)
endif(FK_NATURALIST)

option(ADAFRUIT_FEATHER "Apply Adafruit Feather configuration.")
if(ADAFRUIT_FEATHER)
  add_definitions(-DADAFRUIT_FEATHER)
endif(ADAFRUIT_FEATHER)

add_arduino_firmware(wifi-test)
