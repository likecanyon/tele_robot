cmake_minimum_required(VERSION 3.0.0)
project(sigma_example VERSION 0.1.0 LANGUAGES C CXX)

include_directories(include)

if(WIN32)
add_definitions(/utf-8)
    link_directories(lib\\release\\win)

    set(BOOST_ROOT D:\\Workspace\\3rdParty\\boost_1_82_0\\my_boost)
    find_package(Boost REQUIRED COMPONENTS system thread program_options)

      find_package(ur_rtde REQUIRED PATHS D:\\Workspace\\3rdParty\\ur_rtde\\build\\ur_rtde)

    find_package(Eigen3 REQUIRED PATHS)

    add_executable(sigma_example main.cpp)
    target_link_libraries(sigma_example PUBLIC
        dhdms
        dhdms64
        drdms
        drdms64
        ur_rtde::rtde
        Eigen3::Eigen
    )
elseif(UNIX AND NOT APPLE)
    link_directories(lib/release/lin-x86_64-gcc)
    find_package(ur_rtde REQUIRED)
    add_executable(sigma_example main.cpp)
    target_link_libraries(sigma_example PUBLIC
        dhd
        drd
        pthread
        usb-1.0
        rt
        dl
        ur_rtde::rtde
    )
endif()
