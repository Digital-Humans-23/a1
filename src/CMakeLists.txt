cmake_minimum_required(VERSION 3.12)

add_subdirectory(libs)
add_subdirectory(test-a1)

if(DH_BUILD_APP)
add_subdirectory(app)
endif(DH_BUILD_APP)
