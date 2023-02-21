cmake_minimum_required(VERSION 3.12)

add_subdirectory(libs)
add_subdirectory(test-a2)

if(CMM_BUILD_GUI)
add_subdirectory(app)
endif(CMM_BUILD_GUI)
