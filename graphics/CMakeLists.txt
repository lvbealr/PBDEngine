set(GRAPHICS_SOURCES 
    src/renderer.cpp 
    include/renderer.hpp
)

add_library(GraphicsModule STATIC ${GRAPHICS_SOURCES})

target_include_directories(GraphicsModule PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${GLM_INCLUDE_DIRS}
    ${RAYLIB_INCLUDE_DIR}
)