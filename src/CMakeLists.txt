
file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_executable(plane_rcnn ${SRC_FILES})

target_link_libraries(plane_rcnn PRIVATE core)

target_include_directories(plane_rcnn PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
    target_compile_options(plane_rcnn PRIVATE -fPIC)
ENDIF()

install(TARGETS plane_rcnn DESTINATION bin)