cmake_minimum_required(VERSION 3.21)
project(k4a CXX)

if (WIN32)
    set(CMAKE_FIND_LIBRARY_PREFIXES "")
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib" ".dll")
else()
    set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".so" ".a")
endif()

find_library(K4A_LIB
    NAMES
        k4a
    HINTS
        ${CMAKE_CURRENT_LIST_DIR}/win32/lib/
    REQUIRED
)

find_library(K4ARECORD_LIB
    NAMES
        k4arecord
    HINTS
        ${CMAKE_CURRENT_LIST_DIR}/win32/lib/
    REQUIRED
)


set(K4A_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/win32/include/)

message("K4A_LIB: ${K4A_LIB}")
message("K4ARECORD_LIB: ${K4ARECORD_LIB}")

if (WIN32)

    find_library(K4A_DYNLIB MODULE
        NAMES
            k4a
        HINTS
            ${CMAKE_CURRENT_LIST_DIR}/win32/bin/
        NO_CMAKE_SYSTEM_PATH
    )
    find_library(K4ARECORD_DYNLIB MODULE
        NAMES
            k4arecord
        HINTS
            ${CMAKE_CURRENT_LIST_DIR}/win32/bin/
        NO_CMAKE_SYSTEM_PATH
    )
    find_library(DEPTHENGINE_DYNLIB MODULE
        NAMES
            depthengine_2_0
        HINTS
            ${CMAKE_CURRENT_LIST_DIR}/win32/bin/
        NO_CMAKE_SYSTEM_PATH
    )
    
    if(NOT K4A_DYNLIB)
        message(FATAL_ERROR "k4a dynlib not found")
    endif()
    if(NOT K4ARECORD_DYNLIB)
        message(FATAL_ERROR "k4arecord dynlib not found")
    endif()
    if(NOT DEPTHENGINE_DYNLIB)
        message(FATAL_ERROR "depthengine dynlib not found")
    endif()

    message("K4A_DYNLIB: ${K4A_DYNLIB}")
    message("K4ARECORD_DYNLIB: ${K4ARECORD_DYNLIB}")
    message("DEPTHENGINE_DYNLIB: ${DEPTHENGINE_DYNLIB}")

endif()


################################################################################
# Targets

# k4a library

add_library(k4a INTERFACE)
target_include_directories(k4a INTERFACE ${K4A_INCLUDE_DIRS})
target_link_libraries(k4a INTERFACE
    ${K4A_LIB}
    ${K4ARECORD_LIB}
)
add_library(k4a::k4a ALIAS k4a)