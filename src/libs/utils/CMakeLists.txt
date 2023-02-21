cmake_minimum_required(VERSION 3.12)

project(utils)

set(CMAKE_CXX_STANDARD 11)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(GLOB CRL_SOURCES #
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)

# dependencies
list(APPEND CRL_TARGET_DEPENDENCIES #
     "eigen" #
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# target link libs
list(APPEND CRL_TARGET_LINK_LIBS #
     PUBLIC "eigen" #
)

# compile definitions
list(APPEND CRL_COMPILE_DEFINITIONS #
     PUBLIC "CRL_DATA_FOLDER=${CRL_DATA_FOLDER}" #
)

# create target
create_crl_library(
    ${CRL_TARGET_NAME}
    "${CRL_SOURCES}" #
    "${CRL_TARGET_DEPENDENCIES}" #
    "${CRL_TARGET_INCLUDE_DIRS}" #
    "${CRL_TARGET_LINK_LIBS}" #
    "${CRL_COMPILE_DEFINITIONS}" #
)