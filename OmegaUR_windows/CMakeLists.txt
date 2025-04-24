cmake_minimum_required(VERSION 3.0.0)
project(omega_example VERSION 0.1.0 LANGUAGES C CXX)


include_directories(include)
link_directories( lib)


set(BOOST_ROOT W:\\libraries\\boost_1_82_0\\my_boost)
find_package(Boost REQUIRED COMPONENTS system thread program_options)

find_package(ur_rtde REQUIRED PATHS W:\\libraries\\ur_rtde\\my_rtde\\lib\\cmake\\ur_rtde)

find_package(Eigen3 REQUIRED PATHS W:\\libraries\\eigen-3.4.0\\my_eigen\\share\\eigen3\\cmake)



add_executable(omega_example main.cpp)
target_link_libraries(  omega_example  PUBLIC
dhdms
dhdms64
drdms
drdms64

ur_rtde::rtde
Eigen3::Eigen

)

