set(CORE_SOURCES
    src/physics.cpp
    src/particle_system.cpp
    src/constraints.cpp
    src/self_collision.cpp
    include/physics.hpp
    include/particle_system.hpp
    include/constraints.hpp
    include/self_collision.hpp
)

add_library(PhysicsCore STATIC ${CORE_SOURCES})

target_include_directories(PhysicsCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} include/)

target_include_directories(PhysicsCore PRIVATE ${GLM_INCLUDE_DIRS})