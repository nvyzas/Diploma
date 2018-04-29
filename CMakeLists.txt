cmake_minimum_required (VERSION 2.8.11)
project(Diploma)
set(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release"
        CACHE STRING "List of supported configuration types." FORCE)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#set(CMAKE_MODULE_PATH "C:\Qt\5.9.2\msvc2017_64")
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenSim REQUIRED)
IF(OpenSim_FOUND) 
MESSAGE( "Project ${PROJECT_NAME} found OpenSim." ) 
ENDIF(OpenSim_FOUND) 

# Set Include Directories
set(Diploma_INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/res/assimp/include/
	${PROJECT_SOURCE_DIR}/res/kinect/include/
	#${OpenSim_INCLUDE_DIRS}
)

# Set Link Directories
set(Diploma_LINK_DIRS
	${PROJECT_SOURCE_DIR}/res/assimp/lib/
	${PROJECT_SOURCE_DIR}/res/kinect/lib/
	#${OpenSim_LIB_DIR}
)

# Set Link Libraries
set(Diploma_LINK_LIBS
	assimp-vc140-mt
	Kinect20
	Qt5::Core
	Qt5::Gui
	Qt5::Widgets
	${OpenSim_LIBRARIES}
	#libblas
	#liblapack
	#OpenSim_SimTKcommon
	#OpenSim_SimTKmath
	#OpenSim_SimTKsimbody
	#osimActuators
	#osimAnalyses
	#osimCommon
	#osimJavaJNI
	#osimLepton
	#osimSimulation
	#osimTools
	#pthreadVC2_x64
)

# Set Sources
set(Diploma_SRCS
	src/camera.cpp
	src/main.cpp
	src/main_widget.cpp
	src/main_window.cpp
	src/opensim_model.cpp
	src/pipeline.cpp
	src/ksensor.cpp
	src/kskeleton.cpp
	src/skinned_mesh.cpp
	src/skinning_technique.cpp
	src/technique.cpp
	src/util.cpp
)

set(CMAKE_AUTOUIC_SEARCH_PATHS forms/)

# Build & Link
include_directories(${Diploma_INCLUDE_DIRS})
link_directories(${Diploma_LINK_DIRS})
add_executable(Diploma ${Diploma_SRCS})
target_link_libraries(Diploma ${Diploma_LINK_LIBS})

add_custom_command(TARGET Diploma PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PROJECT_SOURCE_DIR}/assimp-vc140-mt.dll" $<TARGET_FILE_DIR:Diploma>)
add_custom_command(TARGET Diploma POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/models" $<TARGET_FILE_DIR:Diploma>/models)
add_custom_command(TARGET Diploma POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/shaders" $<TARGET_FILE_DIR:Diploma>/shaders)
add_custom_command(TARGET Diploma POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/plane" $<TARGET_FILE_DIR:Diploma>/plane)
#add_custom_command(TARGET Diploma POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${OpenSim_ROOT_DIR}/bin" $<TARGET_FILE_DIR:Diploma>)