cmake_minimum_required(VERSION 3.16)

project(arduino_due_manchester_receiver LANGUAGES CXX)

add_library(receiver
    lib/receiver/src/ManchesterDecoder.cpp
)
target_include_directories(receiver
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/receiver/src
)
target_compile_features(receiver PUBLIC cxx_std_11)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(receiver PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(receiver_sim
    tools/receiver_sim.cpp
)
target_link_libraries(receiver_sim PRIVATE receiver)
target_compile_features(receiver_sim PRIVATE cxx_std_17)

add_executable(receiver_tests
    tests/receiver_tests.cpp
)
target_link_libraries(receiver_tests PRIVATE receiver)
target_compile_features(receiver_tests PRIVATE cxx_std_17)

enable_testing()
add_test(NAME receiver_tests COMMAND receiver_tests)
