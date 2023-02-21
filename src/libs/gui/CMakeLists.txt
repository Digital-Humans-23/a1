cmake_minimum_required(VERSION 3.12)

project(gui)

set(CRL_TARGET_NAME ${PROJECT_NAME})

set(CRL_SHADER_FOLDER
    "\"${CMAKE_CURRENT_SOURCE_DIR}/shaders\""
    CACHE STRING ""
)

file(
    GLOB
    CRL_SOURCES #
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gui/*.h" #
    "${CMAKE_CURRENT_SOURCE_DIR}/imgui_widgets/include/imgui_widgets/*.h" #
    "${CMAKE_CURRENT_SOURCE_DIR}/imgui_widgets/*.cpp" #
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.vert" #
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.frag" #
)

# dependencies
list(
    APPEND
    CRL_TARGET_DEPENDENCIES #
    "glad" #
    "glfw" #
    "glm" #
    "imgui" #
    "stb_image" #
    "tinyobjloader" #
    "eigen" #
    "crl::utils" #
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/imgui_widgets/include" #
)

# target link libs
list(
    APPEND
    CRL_TARGET_LINK_LIBS #
    PUBLIC
    "glm" #
    PUBLIC
    "glad" #
    PUBLIC 
    "glfw" #
    PUBLIC
    "imgui" #
    PUBLIC
    "stb_image" #
    PUBLIC
    "tinyobjloader" #
    PUBLIC
    "eigen" #
    PUBLIC
    "crl::utils" #
)

# compile definitions
list(
    APPEND
    CRL_COMPILE_DEFINITIONS #
    PUBLIC
    "CRL_DATA_FOLDER=${CRL_DATA_FOLDER}" #
    PUBLIC
    "CRL_SHADER_FOLDER=${CRL_SHADER_FOLDER}" #
    PUBLIC
    "CMM_IMGUI_FONT_FOLDER=${CMM_IMGUI_FONT_FOLDER}" #
    PUBLIC
    "IMGUI_IMPL_OPENGL_LOADER_GLAD" #
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