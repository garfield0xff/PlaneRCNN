include_directories(.)
include_directories(..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
foreach(subdir ${SUBDIRS})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
    add_source_files(SUBDIR_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
endforeach()

message(STATUS ${SUBDIR_SRCS})

file(GLOB LOCALDIR_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(APPEND SUBDIR_SRCS ${LOCALDIR_SRCS})
set(SRC_FILES ${SUBDIR_SRCS} PARENT_SCOPE)