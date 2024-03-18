cmake_minimum_required(VERSION 3.21)
project(ir-tracking)

# Set C++17 as the standard globally
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 -D_ENABLE_EXTENDED_ALIGNED_STORAGE=1")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /FORCE") # /PROFILE
    add_definitions("/MP")
    add_definitions("/MT")
else()
    # Warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing")

    # Remove Asio warning
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-unused-but-set-variable")

    # Static library: -fPIC
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -march=native -fstack-protector")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")
endif()

IF(MSVC)
   ADD_DEFINITIONS(/arch:AVX)
ENDIF(MSVC)

add_subdirectory(extern)

# Simple non robust way to find the k4a library
if(NOT WIN32)
    # Find the Azure Kinect SDK
    find_package( k4a REQUIRED )
endif()

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

set(TRACKER_SOURCE_FILES
    include/IRStructs.h
    include/IRKalmanFilter.h
    include/IRToolTrack.h
    include/IRToolTracking.h
    src/IRToolTrack.cpp
    src/IRToolTracking.cpp 
)
include_directories(include)
# Add the IRToolTracking library
add_library(IRToolTracking STATIC ${TRACKER_SOURCE_FILES})
# Specify C++17 for IRToolTracking library
target_compile_features(IRToolTracking PUBLIC cxx_std_17)

# Link RealSense SDK and OpenCV with the IRToolTracking library
target_link_libraries(IRToolTracking PUBLIC Eigen3 k4a::k4a opencv_core opencv_imgproc opencv_imgcodecs opencv_video)

################################################################################
# Dependencies for the main application



#set(MACOS_ICON "resources/app_icon.icns")


find_package(OpenGL REQUIRED)
include_directories(
    .
    ${OPENGL_INCLUDE_DIRS}
)

set(VIEWER_SOURCE_FILES
    #${MACOS_ICON}
    win.rc
    include/ROMParser.h
    include/ViewerWindow.h
    src/ViewerWindow.cpp
    src/viewer_main.cpp
)

# Add the main application
add_executable(ir-tracking-app  ${VIEWER_SOURCE_FILES})

# if(APPLE)
#     set_target_properties(ir-tracking-app PROPERTIES
#                             MACOSX_BUNDLE TRUE
#                             MACOSX_BUNDLE_ICON_FILE app_icon.icns
#                             RESOURCE "${MACOS_ICON}"
#                             )
#     set_source_files_properties(${MACOS_ICON} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
# endif()

include_directories( ${IMGUI_DIR} )
include_directories( ${IMGUI_DIR}/backends)
include_directories( ${OPENGL_INCLUDE_DIR} )
include_directories( ${GLFW_INCLUDE_DIR} )

# Specify C++17 for the main application
target_compile_features(ir-tracking-app PUBLIC cxx_std_17)

target_link_libraries(ir-tracking-app PRIVATE
                        IRToolTracking
                        glfw::glfw
                        imgui::imgui
                        ${OPENGL_LIBRARIES} 
                        nanosockets
                        nfd
                        json)

if(WIN32)

    set(DYNLIB_TARGET ir-tracking-app)
    set(DYNLIB_DEST "$<TARGET_FILE_DIR:${DYNLIB_TARGET}>")

    message("Copying k4a library to build folder: ${K4A_DYNLIB}")
    add_custom_command(TARGET ${DYNLIB_TARGET} POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${K4A_DYNLIB}"
            "${DYNLIB_DEST}"
    )

    message("Copying k4a library to build folder: ${K4ARECORD_DYNLIB}")
    add_custom_command(TARGET ${DYNLIB_TARGET} POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${K4ARECORD_DYNLIB}"
            "${DYNLIB_DEST}"
    )

    message("Copying k4a library to build folder: ${DEPTHENGINE_DYNLIB}")
    add_custom_command(TARGET ${DYNLIB_TARGET} POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${DEPTHENGINE_DYNLIB}"
            "${DYNLIB_DEST}"
    )

    
endif()




