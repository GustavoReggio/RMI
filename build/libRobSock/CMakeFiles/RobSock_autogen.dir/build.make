# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /home/pedro/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/pedro/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pedro/ciberRatoTools

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pedro/ciberRatoTools/build

# Utility rule file for RobSock_autogen.

# Include any custom commands dependencies for this target.
include libRobSock/CMakeFiles/RobSock_autogen.dir/compiler_depend.make

# Include the progress variables for this target.
include libRobSock/CMakeFiles/RobSock_autogen.dir/progress.make

libRobSock/CMakeFiles/RobSock_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pedro/ciberRatoTools/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target RobSock"
	cd /home/pedro/ciberRatoTools/build/libRobSock && /home/pedro/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E cmake_autogen /home/pedro/ciberRatoTools/build/libRobSock/CMakeFiles/RobSock_autogen.dir/AutogenInfo.json Release

RobSock_autogen: libRobSock/CMakeFiles/RobSock_autogen
RobSock_autogen: libRobSock/CMakeFiles/RobSock_autogen.dir/build.make
.PHONY : RobSock_autogen

# Rule to build all files generated by this target.
libRobSock/CMakeFiles/RobSock_autogen.dir/build: RobSock_autogen
.PHONY : libRobSock/CMakeFiles/RobSock_autogen.dir/build

libRobSock/CMakeFiles/RobSock_autogen.dir/clean:
	cd /home/pedro/ciberRatoTools/build/libRobSock && $(CMAKE_COMMAND) -P CMakeFiles/RobSock_autogen.dir/cmake_clean.cmake
.PHONY : libRobSock/CMakeFiles/RobSock_autogen.dir/clean

libRobSock/CMakeFiles/RobSock_autogen.dir/depend:
	cd /home/pedro/ciberRatoTools/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pedro/ciberRatoTools /home/pedro/ciberRatoTools/libRobSock /home/pedro/ciberRatoTools/build /home/pedro/ciberRatoTools/build/libRobSock /home/pedro/ciberRatoTools/build/libRobSock/CMakeFiles/RobSock_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libRobSock/CMakeFiles/RobSock_autogen.dir/depend

