cmake_minimum_required(VERSION 3.12)

project(test-a2)

add_executable(${PROJECT_NAME}
    test.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC "crl::utils" #
    PUBLIC "crl::simAndControl" #
)

add_test(${PROJECT_NAME} "test-a2")
