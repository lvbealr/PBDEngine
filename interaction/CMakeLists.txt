set(CORE_SOURCES
    src/interaction.cpp
    include/interaction.hpp
)

add_library(InteractionSystem STATIC ${CORE_SOURCES})

target_include_directories(InteractionSystem PUBLIC include/)

target_include_directories(InteractionSystem PRIVATE ${CMAKE_SOURCE_DIR})

target_include_directories(InteractionSystem PRIVATE ${GLM_INCLUDE_DIRS})

target_link_libraries(InteractionSystem PRIVATE CommonDetails)