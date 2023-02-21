
include(ExternalProject)
include(FetchContent)

# Eigen library
# Download as zip, so it's a bit smaller than downloading the whole repo
FetchContent_Declare(
    eigen
    URL                 https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip
    URL_HASH            MD5=888aab45512cc0c734b3e8f60280daba
)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
        add_library(eigen INTERFACE)
        target_include_directories(eigen INTERFACE ${eigen_SOURCE_DIR})
endif()

# stb_image
add_subdirectory(stb_image)

# json
FetchContent_Declare(
    json
    URL                 https://github.com/nlohmann/json/releases/download/v3.9.1/include.zip
    URL_HASH            MD5=d2f66c608af689e21d69a33c220e974e
)
FetchContent_GetProperties(json)
if(NOT json_POPULATED)
 FetchContent_Populate(json)
endif()
add_library(nlohmann_json INTERFACE)
add_library(nlohmann::json ALIAS nlohmann_json)
target_include_directories(nlohmann_json INTERFACE $<BUILD_INTERFACE:${json_SOURCE_DIR}/include>)

# GUI stuff below
if(CMM_BUILD_GUI)
message(STATUS "building CMM GUI")
# glad
set(OpenGL_GL_PREFERENCE GLVND)
add_subdirectory(glad)

# glfw
FetchContent_Declare(
    glfw
    URL                 https://github.com/glfw/glfw/archive/3.3.2.zip
    URL_HASH            MD5=69d0aa90a9278c0dd8be975d1ae4aaf2
)
FetchContent_GetProperties(glfw)
if(NOT glfw_POPULATED)
    FetchContent_Populate(glfw)
endif()
set( GLFW_BUILD_EXAMPLES    OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_BUILD_TESTS       OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_BUILD_DOCS        OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_BUILD_INSTALL     OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/../ext/glfw/include)
add_subdirectory(${glfw_SOURCE_DIR} ext/glfw)

# glm
FetchContent_Declare(
    glm
    URL                 https://github.com/g-truc/glm/releases/download/0.9.9.7/glm-0.9.9.7.zip
    URL_HASH            MD5=da4740b5b41a1e2ccb2955af74e7c298
)
FetchContent_GetProperties(glm)
if(NOT glm_POPULATED)
    FetchContent_Populate(glm)
endif()
set( GLM_TEST_ENABLE OFF CACHE BOOL "" )
add_subdirectory(${glm_SOURCE_DIR} ext/glm)
target_include_directories(glm INTERFACE ${glm_SOURCE_DIR}/glm)

# nanovg
FetchContent_Declare(
    nanovg
    GIT_REPOSITORY      https://github.com/memononen/nanovg
    GIT_TAG             077b65e0cf3e22ee4f588783e319b19b0a608065
)
FetchContent_GetProperties(nanovg)
if(NOT nanovg_POPULATED)
    FetchContent_Populate(nanovg)
endif()
file(GLOB sources ${nanovg_SOURCE_DIR}/src/*.h ${nanovg_SOURCE_DIR}/src/*.c)
add_library(nanovg ${sources})
target_include_directories(nanovg INTERFACE ${nanovg_SOURCE_DIR}/src/)
# set(NANOVG_SRC_DIR ${NANOVG_SRC_DIR} PARENT_SCOPE)

# imgui
FetchContent_Declare(
    imgui
    URL                 https://github.com/ocornut/imgui/archive/v1.81.zip
    URL_HASH            MD5=78af0cf170fb3f2a431eb39cac7f55b8
)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()
file(GLOB sources ${imgui_SOURCE_DIR}/*.h ${imgui_SOURCE_DIR}/*.cpp)
add_library(imgui STATIC ${sources})
set_target_properties(imgui PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR})
set( CMM_IMGUI_FONT_FOLDER "\"${imgui_SOURCE_DIR}/misc/fonts\"" CACHE STRING "")

# tinyobjloader
FetchContent_Declare(
    tinyobjloader #
    URL https://github.com/tinyobjloader/tinyobjloader/archive/v2.0-rc1.zip #
    URL_HASH MD5=f660a9f91fc2ea5c4f33d7530f6a69bc #
)
FetchContent_GetProperties(tinyobjloader)
if(NOT tinyobjloader_POPULATED)
    FetchContent_Populate(tinyobjloader)
endif()
add_subdirectory(${tinyobjloader_SOURCE_DIR} tinyobjloader)


endif(CMM_BUILD_GUI)
