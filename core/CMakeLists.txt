add_subdirectory(src)

add_library(core SHARED ${SRC_FILES})

target_link_libraries(core PUBLIC
    3rdparty::opengl
)


target_include_directories(core PUBLIC
    ${CMAKE_SOURCE_DIR}/core/include
    ${CMAKE_SOURCE_DIR}/3rdparty/opengl/include
)

