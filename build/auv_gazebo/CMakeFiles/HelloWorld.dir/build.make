# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/dev_ws/src/auv_gazebo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/dev_ws/build/auv_gazebo

# Include any dependencies generated for this target.
include CMakeFiles/HelloWorld.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/HelloWorld.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/HelloWorld.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/HelloWorld.dir/flags.make

CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.o: CMakeFiles/HelloWorld.dir/flags.make
CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.o: /home/ubuntu/dev_ws/src/auv_gazebo/src/HelloWorld.cc
CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.o: CMakeFiles/HelloWorld.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/dev_ws/build/auv_gazebo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.o -MF CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.o.d -o CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.o -c /home/ubuntu/dev_ws/src/auv_gazebo/src/HelloWorld.cc

CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/dev_ws/src/auv_gazebo/src/HelloWorld.cc > CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.i

CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/dev_ws/src/auv_gazebo/src/HelloWorld.cc -o CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.s

# Object files for target HelloWorld
HelloWorld_OBJECTS = \
"CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.o"

# External object files for target HelloWorld
HelloWorld_EXTERNAL_OBJECTS =

libHelloWorld.so: CMakeFiles/HelloWorld.dir/src/HelloWorld.cc.o
libHelloWorld.so: CMakeFiles/HelloWorld.dir/build.make
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-gazebo6.so.6.11.0
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-fuel_tools7.so.7.1.0
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-gui6.so.6.6.1
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-common4-profiler.so.4.5.2
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-common4-events.so.4.5.2
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-common4-av.so.4.5.2
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libswscale.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libswscale.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libavdevice.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libavdevice.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libavformat.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libavformat.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libavcodec.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libavcodec.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libavutil.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libavutil.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-common4-graphics.so.4.5.2
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-common4.so.4.5.2
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-plugin1-loader.so.1.3.0
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-plugin1.so.1.3.0
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-transport11-log.so.11.2.0
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libQt5QuickControls2.so.5.15.3
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libQt5Quick.so.5.15.3
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libQt5QmlModels.so.5.15.3
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libQt5Qml.so.5.15.3
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libQt5Network.so.5.15.3
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libQt5Widgets.so.5.15.3
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libQt5Gui.so.5.15.3
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libQt5Core.so.5.15.3
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-transport11.so.11.2.0
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libuuid.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libuuid.so
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-msgs8.so.8.6.0
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libsdformat12.so.12.5.0
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-math6.so.6.12.0
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libignition-utils1.so.1.4.0
libHelloWorld.so: /usr/lib/aarch64-linux-gnu/libprotobuf.so
libHelloWorld.so: CMakeFiles/HelloWorld.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/dev_ws/build/auv_gazebo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libHelloWorld.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HelloWorld.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/HelloWorld.dir/build: libHelloWorld.so
.PHONY : CMakeFiles/HelloWorld.dir/build

CMakeFiles/HelloWorld.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/HelloWorld.dir/cmake_clean.cmake
.PHONY : CMakeFiles/HelloWorld.dir/clean

CMakeFiles/HelloWorld.dir/depend:
	cd /home/ubuntu/dev_ws/build/auv_gazebo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/dev_ws/src/auv_gazebo /home/ubuntu/dev_ws/src/auv_gazebo /home/ubuntu/dev_ws/build/auv_gazebo /home/ubuntu/dev_ws/build/auv_gazebo /home/ubuntu/dev_ws/build/auv_gazebo/CMakeFiles/HelloWorld.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/HelloWorld.dir/depend
