cmake_minimum_required(VERSION 3.10)
project(can_listener LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(can_listener
    main.cpp
    can_car_listener.cpp
)

# Link pthreads on POSIX systems (needed for std::mutex)
if(NOT WIN32)
    find_package(Threads REQUIRED)
    target_link_libraries(can_listener PRIVATE Threads::Threads)
endif()
