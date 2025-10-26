cmake_minimum_required(VERSION 3.23)
project(VisionFlowStudio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable Qt's automatic processing
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find dependencies
find_package(Qt6 COMPONENTS Widgets OpenGLWidgets REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(VTK COMPONENTS
    CommonCore
    CommonDataModel
    CommonColor
    FiltersSources
    RenderingCore
    InteractionStyle
    RenderingOpenGL2
    GUISupportQt
    REQUIRED)
# FFmpeg: prefer config package (unofficial-ffmpeg), fallback to module find
find_package(unofficial-ffmpeg CONFIG QUIET)
if(TARGET unofficial::ffmpeg::avcodec)
  set(FFMPEG_USE_CONFIG TRUE)
else()
  find_package(ffmpeg REQUIRED)
  set(FFMPEG_USE_CONFIG FALSE)
endif()

# Sources
add_executable(VisionFlowStudio
    src/app/main.cpp
    src/app/MainWindow.cpp
    src/app/MainWindow.h
    src/modules/FFmpegUtils.cpp
    src/modules/FFmpegUtils.h
    # PIV module
    src/modules/piv/PivParams.h
    src/modules/piv/PivResult.h
    src/modules/piv/PivEngine.h
    src/modules/piv/PivEngine.cpp
    # RDIC module
    src/modules/rdic/RdicParams.h
    src/modules/rdic/RdicResult.h
    src/modules/rdic/RdicEngine.h
    src/modules/rdic/RdicEngine.cpp
)

# Include directories
target_include_directories(VisionFlowStudio PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules
)

# Link libraries
# OpenCV may export component targets or an umbrella target depending on build
# Here we link the components typically needed for video playback
target_link_libraries(VisionFlowStudio PRIVATE
    Qt6::Widgets
    Qt6::OpenGLWidgets
    opencv_core opencv_imgproc opencv_videoio opencv_video
    VTK::CommonCore VTK::CommonDataModel VTK::CommonColor VTK::FiltersSources VTK::RenderingCore VTK::InteractionStyle VTK::RenderingOpenGL2 VTK::GUISupportQt
)
# FFmpeg link: config targets if available, otherwise module libraries
if(FFMPEG_USE_CONFIG)
  target_link_libraries(VisionFlowStudio PRIVATE
    unofficial::ffmpeg::avcodec unofficial::ffmpeg::avformat unofficial::ffmpeg::avutil unofficial::ffmpeg::swscale unofficial::ffmpeg::swresample
  )
else()
  target_link_libraries(VisionFlowStudio PRIVATE ${FFMPEG_LIBRARIES})
endif()

# VTK module auto-init (required for some VTK builds)
vtk_module_autoinit(
  TARGETS VisionFlowStudio
  MODULES VTK::CommonCore VTK::CommonDataModel VTK::CommonColor VTK::FiltersSources VTK::RenderingCore VTK::InteractionStyle VTK::RenderingOpenGL2 VTK::GUISupportQt
)

# Installation
install(TARGETS VisionFlowStudio RUNTIME DESTINATION bin)