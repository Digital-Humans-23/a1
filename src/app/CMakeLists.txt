cmake_minimum_required(VERSION 3.12)

project(a1App)

file(GLOB CRL_SOURCES #
     "*.h" #
     "*.cpp" #
)

list(
    APPEND
    CRL_TARGET_DEPENDENCIES #
    "crl::utils" #
    "crl::gui" #
    "crl::simAndControl" #
)

list(APPEND CRL_TARGET_INCLUDE_DIRS #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)

list(
    APPEND
    CRL_TARGET_LINK_LIBS #
    PUBLIC
    "crl::utils" #
    PUBLIC
    "crl::gui" #
    PUBLIC
    "crl::simAndControl" #
)

list(APPEND CRL_COMPILE_DEFINITIONS #
     PUBLIC "CRL_DATA_FOLDER=${CRL_DATA_FOLDER}" #
)

create_crl_app(
    ${PROJECT_NAME}
    "${CRL_SOURCES}" #
    "${CRL_TARGET_DEPENDENCIES}" #
    "${CRL_TARGET_INCLUDE_DIRS}" #
    "${CRL_TARGET_LINK_LIBS}" #
    "${CRL_COMPILE_DEFINITIONS}"
)
