find_package(OpenGL REQUIRED)


add_library(3rdparty_opengl INTERFACE)

target_include_directories(3rdparty_opengl INTERFACE
    ${OPENGL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(3rdparty_opengl INTERFACE 
    OpenGL::GL
    GLEW::GLEW
    glm::glm
)

add_library(3rdparty::opengl ALIAS 3rdparty_opengl)


