find_package(OpenGL REQUIRED)
find_package(glm    REQUIRED)
find_package(GLEW   REQUIRED)
find_package(GLFW QUIET)
if(NOT GLFW_FOUND)
    message(STATUS "GLFW NOT FOUND PKG FOUND...")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GLFW REQUIRED glfw3)
endif()


add_library(3rdparty_opengl INTERFACE)

target_include_directories(3rdparty_opengl INTERFACE
    ${OPENGL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LINUX_INCLUDE_DIRS}
)

if(NOT TARGET glm::glm)
    add_library(glm::glm INTERFACE IMPORTED)
    set_target_properties(glm::glm PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${GLM_INCLUDE_DIRS}"
    )
endif()

target_link_libraries(3rdparty_opengl INTERFACE 
    OpenGL::GL
    GLEW::GLEW
    glm::glm    
)

add_library(3rdparty::opengl ALIAS 3rdparty_opengl)


