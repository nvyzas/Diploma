project(Diploma)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

# Set Include Directories
set(Diploma_INCLUDE_DIRS
	res/assimp/include/
	res/imagemagick/include/
	res/kinect/include/
)

# Set Link Directories
set(Diploma_LINK_DIRS
	../res/assimp/lib/
	../res/imagemagick/lib/
	../res/kinect/lib/
)

# Set Link Libraries
set(Diploma_LINK_LIBS
	assimp-vc140-mt
	CORE_RL_Magick++_
	CORE_RL_MagickCore_
	CORE_RL_MagickWand_
	Kinect20
	Qt5::Core
	Qt5::Gui
	Qt5::Widgets
)

# Set Sources
set(Diploma_SRCS
	src/camera.cpp
	src/main.cpp
	src/main_widget.cpp
	src/main_window.cpp
	src/math_3d.cpp
	src/pipeline.cpp
	src/sensor.cpp
	src/skinned_mesh.cpp
	src/skinning_technique.cpp
	src/technique.cpp
	src/texture.cpp
	src/util.cpp
)

set(CMAKE_AUTOUIC_SEARCH_PATHS forms/)

# Build & Link
include_directories(${Diploma_INCLUDE_DIRS})
link_directories(${Diploma_LINK_DIRS})
add_executable(Diploma ${Diploma_SRCS})
target_link_libraries(Diploma ${Diploma_LINK_LIBS})