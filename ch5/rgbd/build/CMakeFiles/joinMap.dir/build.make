# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/hgfs/slambook2/ch5/rgbd

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/hgfs/slambook2/ch5/rgbd/build

# Include any dependencies generated for this target.
include CMakeFiles/joinMap.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/joinMap.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/joinMap.dir/flags.make

CMakeFiles/joinMap.dir/joinMap.cpp.o: CMakeFiles/joinMap.dir/flags.make
CMakeFiles/joinMap.dir/joinMap.cpp.o: ../joinMap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/hgfs/slambook2/ch5/rgbd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/joinMap.dir/joinMap.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joinMap.dir/joinMap.cpp.o -c /mnt/hgfs/slambook2/ch5/rgbd/joinMap.cpp

CMakeFiles/joinMap.dir/joinMap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joinMap.dir/joinMap.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/hgfs/slambook2/ch5/rgbd/joinMap.cpp > CMakeFiles/joinMap.dir/joinMap.cpp.i

CMakeFiles/joinMap.dir/joinMap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joinMap.dir/joinMap.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/hgfs/slambook2/ch5/rgbd/joinMap.cpp -o CMakeFiles/joinMap.dir/joinMap.cpp.s

CMakeFiles/joinMap.dir/joinMap.cpp.o.requires:

.PHONY : CMakeFiles/joinMap.dir/joinMap.cpp.o.requires

CMakeFiles/joinMap.dir/joinMap.cpp.o.provides: CMakeFiles/joinMap.dir/joinMap.cpp.o.requires
	$(MAKE) -f CMakeFiles/joinMap.dir/build.make CMakeFiles/joinMap.dir/joinMap.cpp.o.provides.build
.PHONY : CMakeFiles/joinMap.dir/joinMap.cpp.o.provides

CMakeFiles/joinMap.dir/joinMap.cpp.o.provides.build: CMakeFiles/joinMap.dir/joinMap.cpp.o


# Object files for target joinMap
joinMap_OBJECTS = \
"CMakeFiles/joinMap.dir/joinMap.cpp.o"

# External object files for target joinMap
joinMap_EXTERNAL_OBJECTS =

joinMap: CMakeFiles/joinMap.dir/joinMap.cpp.o
joinMap: CMakeFiles/joinMap.dir/build.make
joinMap: /usr/local/lib/libopencv_dnn.so.4.3.0
joinMap: /usr/local/lib/libopencv_gapi.so.4.3.0
joinMap: /usr/local/lib/libopencv_highgui.so.4.3.0
joinMap: /usr/local/lib/libopencv_ml.so.4.3.0
joinMap: /usr/local/lib/libopencv_objdetect.so.4.3.0
joinMap: /usr/local/lib/libopencv_photo.so.4.3.0
joinMap: /usr/local/lib/libopencv_stitching.so.4.3.0
joinMap: /usr/local/lib/libopencv_video.so.4.3.0
joinMap: /usr/local/lib/libopencv_videoio.so.4.3.0
joinMap: /usr/local/lib/libpangolin.so
joinMap: /usr/local/lib/libopencv_imgcodecs.so.4.3.0
joinMap: /usr/local/lib/libopencv_calib3d.so.4.3.0
joinMap: /usr/local/lib/libopencv_features2d.so.4.3.0
joinMap: /usr/local/lib/libopencv_flann.so.4.3.0
joinMap: /usr/local/lib/libopencv_imgproc.so.4.3.0
joinMap: /usr/local/lib/libopencv_core.so.4.3.0
joinMap: /usr/lib/x86_64-linux-gnu/libOpenGL.so
joinMap: /usr/lib/x86_64-linux-gnu/libGLX.so
joinMap: /usr/lib/x86_64-linux-gnu/libGLU.so
joinMap: /usr/lib/x86_64-linux-gnu/libGLEW.so
joinMap: /usr/lib/x86_64-linux-gnu/libEGL.so
joinMap: /usr/lib/x86_64-linux-gnu/libSM.so
joinMap: /usr/lib/x86_64-linux-gnu/libICE.so
joinMap: /usr/lib/x86_64-linux-gnu/libX11.so
joinMap: /usr/lib/x86_64-linux-gnu/libXext.so
joinMap: /usr/lib/x86_64-linux-gnu/libOpenGL.so
joinMap: /usr/lib/x86_64-linux-gnu/libGLX.so
joinMap: /usr/lib/x86_64-linux-gnu/libGLU.so
joinMap: /usr/lib/x86_64-linux-gnu/libGLEW.so
joinMap: /usr/lib/x86_64-linux-gnu/libEGL.so
joinMap: /usr/lib/x86_64-linux-gnu/libSM.so
joinMap: /usr/lib/x86_64-linux-gnu/libICE.so
joinMap: /usr/lib/x86_64-linux-gnu/libX11.so
joinMap: /usr/lib/x86_64-linux-gnu/libXext.so
joinMap: /usr/lib/x86_64-linux-gnu/libavcodec.so
joinMap: /usr/lib/x86_64-linux-gnu/libavformat.so
joinMap: /usr/lib/x86_64-linux-gnu/libavutil.so
joinMap: /usr/lib/x86_64-linux-gnu/libswscale.so
joinMap: /usr/lib/x86_64-linux-gnu/libavdevice.so
joinMap: CMakeFiles/joinMap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/hgfs/slambook2/ch5/rgbd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable joinMap"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joinMap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/joinMap.dir/build: joinMap

.PHONY : CMakeFiles/joinMap.dir/build

CMakeFiles/joinMap.dir/requires: CMakeFiles/joinMap.dir/joinMap.cpp.o.requires

.PHONY : CMakeFiles/joinMap.dir/requires

CMakeFiles/joinMap.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/joinMap.dir/cmake_clean.cmake
.PHONY : CMakeFiles/joinMap.dir/clean

CMakeFiles/joinMap.dir/depend:
	cd /mnt/hgfs/slambook2/ch5/rgbd/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/hgfs/slambook2/ch5/rgbd /mnt/hgfs/slambook2/ch5/rgbd /mnt/hgfs/slambook2/ch5/rgbd/build /mnt/hgfs/slambook2/ch5/rgbd/build /mnt/hgfs/slambook2/ch5/rgbd/build/CMakeFiles/joinMap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/joinMap.dir/depend
