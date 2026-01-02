cmake_minimum_required(VERSION 3.10)
project(PDBEngine VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_path(GLM_INCLUDE_DIRS "glm/glm.hpp" PATHS "/opt/homebrew/include")

list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
find_path(RAYLIB_INCLUDE_DIR "raylib.h" PATHS "/opt/homebrew/include")
find_library(RAYLIB_LIBRARY NAMES raylib PATHS "/opt/homebrew/lib")

if(NOT RAYLIB_LIBRARY)
    message(FATAL_ERROR "Raylib not found!")
endif()

add_subdirectory(core)
add_subdirectory(graphics)
add_subdirectory(interaction)
add_subdirectory(common)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GLM_INCLUDE_DIRS}
    ${RAYLIB_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    PhysicsCore 
    GraphicsModule
    InteractionSystem
    CommonDetails
    ${RAYLIB_LIBRARY}
    "-framework IOKit" 
    "-framework Cocoa" 
    "-framework OpenGL"
)