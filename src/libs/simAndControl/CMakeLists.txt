cmake_minimum_required(VERSION 3.12)

project(simAndControl)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
    GLOB
    CRL_SOURCES #
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/*.h" #
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/*.cpp" #
    "${CMAKE_CURRENT_SOURCE_DIR}/kinematics/*.h" #
    "${CMAKE_CURRENT_SOURCE_DIR}/kinematics/*.cpp" #
    "${CMAKE_CURRENT_SOURCE_DIR}/locomotion/*.h" #
    "${CMAKE_CURRENT_SOURCE_DIR}/locomotion/*.h" #
    "${CMAKE_CURRENT_SOURCE_DIR}/locomotion/*.cpp" #
)

# basic dependencies
list(
    APPEND
    CRL_TARGET_DEPENDENCIES #
    "crl::utils" #
    "crl::gui" #
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)

# target link libs
list(
    APPEND
    CRL_TARGET_LINK_LIBS #
    PUBLIC
    "crl::utils" #
    PUBLIC
    "crl::gui" #
    PUBLIC
    "eigen" #
)

if(CRL_BUILD_ODE)
    add_subdirectory(ode)
endif(CRL_BUILD_ODE)

# compile definitions
list(APPEND CRL_COMPILE_DEFINITIONS #
     ""
)

# create target
create_crl_library(
    ${CRL_TARGET_NAME}
    "${CRL_SOURCES}" #
    "${CRL_TARGET_DEPENDENCIES}" #
    "${CRL_TARGET_INCLUDE_DIRS}" #
    "${CRL_TARGET_LINK_LIBS}" #
    "${CRL_COMPILE_DEFINITIONS}"
)