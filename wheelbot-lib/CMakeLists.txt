cmake_minimum_required(VERSION 3.21)

project(
  Wheelbot
  VERSION 1.0
  LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mcpu=native -mtune=native")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED system)


include(FetchContent)
FetchContent_Declare(
  readerwriterqueue
  GIT_REPOSITORY    https://github.com/cameron314/readerwriterqueue
  GIT_TAG           master
)
FetchContent_MakeAvailable(readerwriterqueue)


FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
)
FetchContent_MakeAvailable(argparse)


FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)
FetchContent_MakeAvailable(json)


include(FetchContent)
FetchContent_Declare(
  cactus_rt
  GIT_REPOSITORY https://github.com/cactusdynamics/cactus-rt.git
  GIT_TAG        79b81f33
)
FetchContent_MakeAvailable(cactus_rt)

option(HARDWARE_BUILD "Enable robot build" OFF)

if(HARDWARE_BUILD)
  add_definitions(-DHARDWARE_BUILD)
endif()

option(TESTS "Enable tests" ON)

if(TESTS)
  add_definitions(-DTESTS)
endif()

include_directories(
  include
)

add_executable(Main src/main.cpp)
target_link_libraries(Main PRIVATE Boost::system -liio atomic quill::quill cactus_rt readerwriterqueue argparse nlohmann_json::nlohmann_json)


if(TESTS)
  add_executable(TestCan test/can/main.cpp)
  target_link_libraries(TestCan PRIVATE -liio atomic quill::quill cactus_rt -lgpiod)

  add_executable(TestQueue test/queue/main.cpp)
  target_link_libraries(TestQueue PRIVATE quill::quill readerwriterqueue)

  add_executable(TestGyro test/gyro/main.cpp)
  target_link_libraries(TestGyro PRIVATE -liio atomic quill::quill cactus_rt readerwriterqueue)

  add_executable(TestAccel test/accel/main.cpp)
  target_link_libraries(TestAccel PRIVATE -liio atomic quill::quill cactus_rt readerwriterqueue)

  add_executable(TestImuThread test/imu_thread/main.cpp)
  target_link_libraries(TestImuThread PRIVATE -liio atomic quill::quill cactus_rt readerwriterqueue)

  add_executable(TestLogging test/logging/main.cpp)
  target_link_libraries(TestLogging PRIVATE -liio atomic quill::quill cactus_rt readerwriterqueue)

  add_executable(TestConfig test/configuration/main.cpp)
  target_link_libraries(TestConfig PRIVATE nlohmann_json::nlohmann_json)

  add_executable(TestInputserver test/inputserver/main.cpp)
  target_link_libraries(TestInputserver PRIVATE Boost::system  -liio atomic quill::quill cactus_rt readerwriterqueue nlohmann_json::nlohmann_json)
endif()
