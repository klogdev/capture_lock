# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /tmp1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /tmp1/workspace

# Include any dependencies generated for this target.
include workspace/CMakeFiles/ba_run.dir/depend.make

# Include the progress variables for this target.
include workspace/CMakeFiles/ba_run.dir/progress.make

# Include the compile flags for this target's objects.
include workspace/CMakeFiles/ba_run.dir/flags.make

workspace/CMakeFiles/ba_run.dir/global_bundle.cpp.o: workspace/CMakeFiles/ba_run.dir/flags.make
workspace/CMakeFiles/ba_run.dir/global_bundle.cpp.o: global_bundle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/tmp1/workspace/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object workspace/CMakeFiles/ba_run.dir/global_bundle.cpp.o"
	cd /tmp1/workspace/workspace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ba_run.dir/global_bundle.cpp.o -c /tmp1/workspace/global_bundle.cpp

workspace/CMakeFiles/ba_run.dir/global_bundle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ba_run.dir/global_bundle.cpp.i"
	cd /tmp1/workspace/workspace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /tmp1/workspace/global_bundle.cpp > CMakeFiles/ba_run.dir/global_bundle.cpp.i

workspace/CMakeFiles/ba_run.dir/global_bundle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ba_run.dir/global_bundle.cpp.s"
	cd /tmp1/workspace/workspace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /tmp1/workspace/global_bundle.cpp -o CMakeFiles/ba_run.dir/global_bundle.cpp.s

# Object files for target ba_run
ba_run_OBJECTS = \
"CMakeFiles/ba_run.dir/global_bundle.cpp.o"

# External object files for target ba_run
ba_run_EXTERNAL_OBJECTS =

workspace/libba_run.a: workspace/CMakeFiles/ba_run.dir/global_bundle.cpp.o
workspace/libba_run.a: workspace/CMakeFiles/ba_run.dir/build.make
workspace/libba_run.a: workspace/CMakeFiles/ba_run.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/tmp1/workspace/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libba_run.a"
	cd /tmp1/workspace/workspace && $(CMAKE_COMMAND) -P CMakeFiles/ba_run.dir/cmake_clean_target.cmake
	cd /tmp1/workspace/workspace && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ba_run.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
workspace/CMakeFiles/ba_run.dir/build: workspace/libba_run.a

.PHONY : workspace/CMakeFiles/ba_run.dir/build

workspace/CMakeFiles/ba_run.dir/clean:
	cd /tmp1/workspace/workspace && $(CMAKE_COMMAND) -P CMakeFiles/ba_run.dir/cmake_clean.cmake
.PHONY : workspace/CMakeFiles/ba_run.dir/clean

workspace/CMakeFiles/ba_run.dir/depend:
	cd /tmp1/workspace && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /tmp1 /tmp1/workspace /tmp1/workspace /tmp1/workspace/workspace /tmp1/workspace/workspace/CMakeFiles/ba_run.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : workspace/CMakeFiles/ba_run.dir/depend

