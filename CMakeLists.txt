project(KinectDepth)
cmake_minimum_required(VERSION 2.8)

#find VTK header 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})


#add KINECT header (From Environment SYSTEM)
#C:\Program Files\Microsoft SDKs\Kinect\v1.8
#C:\Program Files\Microsoft SDKs\Kinect\Developer Toolkit v1.8.0 
set(INCLUDE_DIR $ENV{KINECT_TOOLKIT_DIR}inc 
                $ENV{KINECTSDK10_DIR}inc   )
include_directories (${INCLUDE_DIR})   

#execute source
SET(HEADERS vtkImageRender.h DepthSensor.h Timer.h )
add_executable(DepthSensor DepthSensor.cpp vtkImageRender.cpp Timer.cpp  ${HEADERS})

#add  KINECT lib
set(KINECT_SDK_DIR "$ENV{KINECTSDK10_DIR}lib/x86/")
set(KINECT_TOOL_DIR "$ENV{KINECT_TOOLKIT_DIR}lib/x86/" )
 
add_library(KINECT_SDK_LIB STATIC IMPORTED)
   set_property(TARGET KINECT_SDK_LIB PROPERTY IMPORTED_LOCATION
				${KINECT_SDK_DIR}Kinect10.lib)
				
add_library(KINECT_TOOL_LIB STATIC IMPORTED)
   set_property(TARGET KINECT_TOOL_LIB PROPERTY IMPORTED_LOCATION 
				${KINECT_TOOL_DIR}FaceTrackLib.lib
				${KINECT_TOOL_DIR}KinectBackgroundRemoval180_32.lib
				${KINECT_TOOL_DIR}KinectFusion180_32.lib
				${KINECT_TOOL_DIR}KinectInteraction180_32.lib)				

#add lib VTK and KINECT
target_link_libraries(DepthSensor ${VTK_LIBRARIES} KINECT_SDK_LIB KINECT_TOOL_LIB)
