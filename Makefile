# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/gkiri/Desktop/Applied_Robotics/Workspace/Team_Project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gkiri/Desktop/Applied_Robotics/Workspace/Team_Project

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/gkiri/Desktop/Applied_Robotics/Workspace/Team_Project/CMakeFiles /home/gkiri/Desktop/Applied_Robotics/Workspace/Team_Project/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/gkiri/Desktop/Applied_Robotics/Workspace/Team_Project/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named demo_simple_shape_detection

# Build rule for target.
demo_simple_shape_detection: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 demo_simple_shape_detection
.PHONY : demo_simple_shape_detection

# fast build rule for target.
demo_simple_shape_detection/fast:
	$(MAKE) -f CMakeFiles/demo_simple_shape_detection.dir/build.make CMakeFiles/demo_simple_shape_detection.dir/build
.PHONY : demo_simple_shape_detection/fast

#=============================================================================
# Target rules for targets named demo_rgb_filter

# Build rule for target.
demo_rgb_filter: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 demo_rgb_filter
.PHONY : demo_rgb_filter

# fast build rule for target.
demo_rgb_filter/fast:
	$(MAKE) -f CMakeFiles/demo_rgb_filter.dir/build.make CMakeFiles/demo_rgb_filter.dir/build
.PHONY : demo_rgb_filter/fast

#=============================================================================
# Target rules for targets named demo_filters

# Build rule for target.
demo_filters: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 demo_filters
.PHONY : demo_filters

# fast build rule for target.
demo_filters/fast:
	$(MAKE) -f CMakeFiles/demo_filters.dir/build.make CMakeFiles/demo_filters.dir/build
.PHONY : demo_filters/fast

#=============================================================================
# Target rules for targets named student

# Build rule for target.
student: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 student
.PHONY : student

# fast build rule for target.
student/fast:
	$(MAKE) -f CMakeFiles/student.dir/build.make CMakeFiles/student.dir/build
.PHONY : student/fast

#=============================================================================
# Target rules for targets named demo_hsv_filter

# Build rule for target.
demo_hsv_filter: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 demo_hsv_filter
.PHONY : demo_hsv_filter

# fast build rule for target.
demo_hsv_filter/fast:
	$(MAKE) -f CMakeFiles/demo_hsv_filter.dir/build.make CMakeFiles/demo_hsv_filter.dir/build
.PHONY : demo_hsv_filter/fast

#=============================================================================
# Target rules for targets named demo_full_example

# Build rule for target.
demo_full_example: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 demo_full_example
.PHONY : demo_full_example

# fast build rule for target.
demo_full_example/fast:
	$(MAKE) -f CMakeFiles/demo_full_example.dir/build.make CMakeFiles/demo_full_example.dir/build
.PHONY : demo_full_example/fast

#=============================================================================
# Target rules for targets named demo_dilatation_erosion

# Build rule for target.
demo_dilatation_erosion: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 demo_dilatation_erosion
.PHONY : demo_dilatation_erosion

# fast build rule for target.
demo_dilatation_erosion/fast:
	$(MAKE) -f CMakeFiles/demo_dilatation_erosion.dir/build.make CMakeFiles/demo_dilatation_erosion.dir/build
.PHONY : demo_dilatation_erosion/fast

#=============================================================================
# Target rules for targets named demo_smoothing

# Build rule for target.
demo_smoothing: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 demo_smoothing
.PHONY : demo_smoothing

# fast build rule for target.
demo_smoothing/fast:
	$(MAKE) -f CMakeFiles/demo_smoothing.dir/build.make CMakeFiles/demo_smoothing.dir/build
.PHONY : demo_smoothing/fast

src/demo_shape_detection/demo_dilatation_erosion.o: src/demo_shape_detection/demo_dilatation_erosion.cpp.o

.PHONY : src/demo_shape_detection/demo_dilatation_erosion.o

# target to build an object file
src/demo_shape_detection/demo_dilatation_erosion.cpp.o:
	$(MAKE) -f CMakeFiles/demo_dilatation_erosion.dir/build.make CMakeFiles/demo_dilatation_erosion.dir/src/demo_shape_detection/demo_dilatation_erosion.cpp.o
.PHONY : src/demo_shape_detection/demo_dilatation_erosion.cpp.o

src/demo_shape_detection/demo_dilatation_erosion.i: src/demo_shape_detection/demo_dilatation_erosion.cpp.i

.PHONY : src/demo_shape_detection/demo_dilatation_erosion.i

# target to preprocess a source file
src/demo_shape_detection/demo_dilatation_erosion.cpp.i:
	$(MAKE) -f CMakeFiles/demo_dilatation_erosion.dir/build.make CMakeFiles/demo_dilatation_erosion.dir/src/demo_shape_detection/demo_dilatation_erosion.cpp.i
.PHONY : src/demo_shape_detection/demo_dilatation_erosion.cpp.i

src/demo_shape_detection/demo_dilatation_erosion.s: src/demo_shape_detection/demo_dilatation_erosion.cpp.s

.PHONY : src/demo_shape_detection/demo_dilatation_erosion.s

# target to generate assembly for a file
src/demo_shape_detection/demo_dilatation_erosion.cpp.s:
	$(MAKE) -f CMakeFiles/demo_dilatation_erosion.dir/build.make CMakeFiles/demo_dilatation_erosion.dir/src/demo_shape_detection/demo_dilatation_erosion.cpp.s
.PHONY : src/demo_shape_detection/demo_dilatation_erosion.cpp.s

src/demo_shape_detection/demo_filters.o: src/demo_shape_detection/demo_filters.cpp.o

.PHONY : src/demo_shape_detection/demo_filters.o

# target to build an object file
src/demo_shape_detection/demo_filters.cpp.o:
	$(MAKE) -f CMakeFiles/demo_filters.dir/build.make CMakeFiles/demo_filters.dir/src/demo_shape_detection/demo_filters.cpp.o
.PHONY : src/demo_shape_detection/demo_filters.cpp.o

src/demo_shape_detection/demo_filters.i: src/demo_shape_detection/demo_filters.cpp.i

.PHONY : src/demo_shape_detection/demo_filters.i

# target to preprocess a source file
src/demo_shape_detection/demo_filters.cpp.i:
	$(MAKE) -f CMakeFiles/demo_filters.dir/build.make CMakeFiles/demo_filters.dir/src/demo_shape_detection/demo_filters.cpp.i
.PHONY : src/demo_shape_detection/demo_filters.cpp.i

src/demo_shape_detection/demo_filters.s: src/demo_shape_detection/demo_filters.cpp.s

.PHONY : src/demo_shape_detection/demo_filters.s

# target to generate assembly for a file
src/demo_shape_detection/demo_filters.cpp.s:
	$(MAKE) -f CMakeFiles/demo_filters.dir/build.make CMakeFiles/demo_filters.dir/src/demo_shape_detection/demo_filters.cpp.s
.PHONY : src/demo_shape_detection/demo_filters.cpp.s

src/demo_shape_detection/demo_full_example.o: src/demo_shape_detection/demo_full_example.cpp.o

.PHONY : src/demo_shape_detection/demo_full_example.o

# target to build an object file
src/demo_shape_detection/demo_full_example.cpp.o:
	$(MAKE) -f CMakeFiles/demo_full_example.dir/build.make CMakeFiles/demo_full_example.dir/src/demo_shape_detection/demo_full_example.cpp.o
.PHONY : src/demo_shape_detection/demo_full_example.cpp.o

src/demo_shape_detection/demo_full_example.i: src/demo_shape_detection/demo_full_example.cpp.i

.PHONY : src/demo_shape_detection/demo_full_example.i

# target to preprocess a source file
src/demo_shape_detection/demo_full_example.cpp.i:
	$(MAKE) -f CMakeFiles/demo_full_example.dir/build.make CMakeFiles/demo_full_example.dir/src/demo_shape_detection/demo_full_example.cpp.i
.PHONY : src/demo_shape_detection/demo_full_example.cpp.i

src/demo_shape_detection/demo_full_example.s: src/demo_shape_detection/demo_full_example.cpp.s

.PHONY : src/demo_shape_detection/demo_full_example.s

# target to generate assembly for a file
src/demo_shape_detection/demo_full_example.cpp.s:
	$(MAKE) -f CMakeFiles/demo_full_example.dir/build.make CMakeFiles/demo_full_example.dir/src/demo_shape_detection/demo_full_example.cpp.s
.PHONY : src/demo_shape_detection/demo_full_example.cpp.s

src/demo_shape_detection/demo_hsv_filter.o: src/demo_shape_detection/demo_hsv_filter.cpp.o

.PHONY : src/demo_shape_detection/demo_hsv_filter.o

# target to build an object file
src/demo_shape_detection/demo_hsv_filter.cpp.o:
	$(MAKE) -f CMakeFiles/demo_hsv_filter.dir/build.make CMakeFiles/demo_hsv_filter.dir/src/demo_shape_detection/demo_hsv_filter.cpp.o
.PHONY : src/demo_shape_detection/demo_hsv_filter.cpp.o

src/demo_shape_detection/demo_hsv_filter.i: src/demo_shape_detection/demo_hsv_filter.cpp.i

.PHONY : src/demo_shape_detection/demo_hsv_filter.i

# target to preprocess a source file
src/demo_shape_detection/demo_hsv_filter.cpp.i:
	$(MAKE) -f CMakeFiles/demo_hsv_filter.dir/build.make CMakeFiles/demo_hsv_filter.dir/src/demo_shape_detection/demo_hsv_filter.cpp.i
.PHONY : src/demo_shape_detection/demo_hsv_filter.cpp.i

src/demo_shape_detection/demo_hsv_filter.s: src/demo_shape_detection/demo_hsv_filter.cpp.s

.PHONY : src/demo_shape_detection/demo_hsv_filter.s

# target to generate assembly for a file
src/demo_shape_detection/demo_hsv_filter.cpp.s:
	$(MAKE) -f CMakeFiles/demo_hsv_filter.dir/build.make CMakeFiles/demo_hsv_filter.dir/src/demo_shape_detection/demo_hsv_filter.cpp.s
.PHONY : src/demo_shape_detection/demo_hsv_filter.cpp.s

src/demo_shape_detection/demo_rgb_filter.o: src/demo_shape_detection/demo_rgb_filter.cpp.o

.PHONY : src/demo_shape_detection/demo_rgb_filter.o

# target to build an object file
src/demo_shape_detection/demo_rgb_filter.cpp.o:
	$(MAKE) -f CMakeFiles/demo_rgb_filter.dir/build.make CMakeFiles/demo_rgb_filter.dir/src/demo_shape_detection/demo_rgb_filter.cpp.o
.PHONY : src/demo_shape_detection/demo_rgb_filter.cpp.o

src/demo_shape_detection/demo_rgb_filter.i: src/demo_shape_detection/demo_rgb_filter.cpp.i

.PHONY : src/demo_shape_detection/demo_rgb_filter.i

# target to preprocess a source file
src/demo_shape_detection/demo_rgb_filter.cpp.i:
	$(MAKE) -f CMakeFiles/demo_rgb_filter.dir/build.make CMakeFiles/demo_rgb_filter.dir/src/demo_shape_detection/demo_rgb_filter.cpp.i
.PHONY : src/demo_shape_detection/demo_rgb_filter.cpp.i

src/demo_shape_detection/demo_rgb_filter.s: src/demo_shape_detection/demo_rgb_filter.cpp.s

.PHONY : src/demo_shape_detection/demo_rgb_filter.s

# target to generate assembly for a file
src/demo_shape_detection/demo_rgb_filter.cpp.s:
	$(MAKE) -f CMakeFiles/demo_rgb_filter.dir/build.make CMakeFiles/demo_rgb_filter.dir/src/demo_shape_detection/demo_rgb_filter.cpp.s
.PHONY : src/demo_shape_detection/demo_rgb_filter.cpp.s

src/demo_shape_detection/demo_simple_shape_detection.o: src/demo_shape_detection/demo_simple_shape_detection.cpp.o

.PHONY : src/demo_shape_detection/demo_simple_shape_detection.o

# target to build an object file
src/demo_shape_detection/demo_simple_shape_detection.cpp.o:
	$(MAKE) -f CMakeFiles/demo_simple_shape_detection.dir/build.make CMakeFiles/demo_simple_shape_detection.dir/src/demo_shape_detection/demo_simple_shape_detection.cpp.o
.PHONY : src/demo_shape_detection/demo_simple_shape_detection.cpp.o

src/demo_shape_detection/demo_simple_shape_detection.i: src/demo_shape_detection/demo_simple_shape_detection.cpp.i

.PHONY : src/demo_shape_detection/demo_simple_shape_detection.i

# target to preprocess a source file
src/demo_shape_detection/demo_simple_shape_detection.cpp.i:
	$(MAKE) -f CMakeFiles/demo_simple_shape_detection.dir/build.make CMakeFiles/demo_simple_shape_detection.dir/src/demo_shape_detection/demo_simple_shape_detection.cpp.i
.PHONY : src/demo_shape_detection/demo_simple_shape_detection.cpp.i

src/demo_shape_detection/demo_simple_shape_detection.s: src/demo_shape_detection/demo_simple_shape_detection.cpp.s

.PHONY : src/demo_shape_detection/demo_simple_shape_detection.s

# target to generate assembly for a file
src/demo_shape_detection/demo_simple_shape_detection.cpp.s:
	$(MAKE) -f CMakeFiles/demo_simple_shape_detection.dir/build.make CMakeFiles/demo_simple_shape_detection.dir/src/demo_shape_detection/demo_simple_shape_detection.cpp.s
.PHONY : src/demo_shape_detection/demo_simple_shape_detection.cpp.s

src/demo_shape_detection/demo_smoothing.o: src/demo_shape_detection/demo_smoothing.cpp.o

.PHONY : src/demo_shape_detection/demo_smoothing.o

# target to build an object file
src/demo_shape_detection/demo_smoothing.cpp.o:
	$(MAKE) -f CMakeFiles/demo_smoothing.dir/build.make CMakeFiles/demo_smoothing.dir/src/demo_shape_detection/demo_smoothing.cpp.o
.PHONY : src/demo_shape_detection/demo_smoothing.cpp.o

src/demo_shape_detection/demo_smoothing.i: src/demo_shape_detection/demo_smoothing.cpp.i

.PHONY : src/demo_shape_detection/demo_smoothing.i

# target to preprocess a source file
src/demo_shape_detection/demo_smoothing.cpp.i:
	$(MAKE) -f CMakeFiles/demo_smoothing.dir/build.make CMakeFiles/demo_smoothing.dir/src/demo_shape_detection/demo_smoothing.cpp.i
.PHONY : src/demo_shape_detection/demo_smoothing.cpp.i

src/demo_shape_detection/demo_smoothing.s: src/demo_shape_detection/demo_smoothing.cpp.s

.PHONY : src/demo_shape_detection/demo_smoothing.s

# target to generate assembly for a file
src/demo_shape_detection/demo_smoothing.cpp.s:
	$(MAKE) -f CMakeFiles/demo_smoothing.dir/build.make CMakeFiles/demo_smoothing.dir/src/demo_shape_detection/demo_smoothing.cpp.s
.PHONY : src/demo_shape_detection/demo_smoothing.cpp.s

src/image_undistort.o: src/image_undistort.cpp.o

.PHONY : src/image_undistort.o

# target to build an object file
src/image_undistort.cpp.o:
	$(MAKE) -f CMakeFiles/student.dir/build.make CMakeFiles/student.dir/src/image_undistort.cpp.o
.PHONY : src/image_undistort.cpp.o

src/image_undistort.i: src/image_undistort.cpp.i

.PHONY : src/image_undistort.i

# target to preprocess a source file
src/image_undistort.cpp.i:
	$(MAKE) -f CMakeFiles/student.dir/build.make CMakeFiles/student.dir/src/image_undistort.cpp.i
.PHONY : src/image_undistort.cpp.i

src/image_undistort.s: src/image_undistort.cpp.s

.PHONY : src/image_undistort.s

# target to generate assembly for a file
src/image_undistort.cpp.s:
	$(MAKE) -f CMakeFiles/student.dir/build.make CMakeFiles/student.dir/src/image_undistort.cpp.s
.PHONY : src/image_undistort.cpp.s

src/student_interface.o: src/student_interface.cpp.o

.PHONY : src/student_interface.o

# target to build an object file
src/student_interface.cpp.o:
	$(MAKE) -f CMakeFiles/student.dir/build.make CMakeFiles/student.dir/src/student_interface.cpp.o
.PHONY : src/student_interface.cpp.o

src/student_interface.i: src/student_interface.cpp.i

.PHONY : src/student_interface.i

# target to preprocess a source file
src/student_interface.cpp.i:
	$(MAKE) -f CMakeFiles/student.dir/build.make CMakeFiles/student.dir/src/student_interface.cpp.i
.PHONY : src/student_interface.cpp.i

src/student_interface.s: src/student_interface.cpp.s

.PHONY : src/student_interface.s

# target to generate assembly for a file
src/student_interface.cpp.s:
	$(MAKE) -f CMakeFiles/student.dir/build.make CMakeFiles/student.dir/src/student_interface.cpp.s
.PHONY : src/student_interface.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... demo_simple_shape_detection"
	@echo "... demo_rgb_filter"
	@echo "... demo_filters"
	@echo "... student"
	@echo "... demo_hsv_filter"
	@echo "... demo_full_example"
	@echo "... demo_dilatation_erosion"
	@echo "... demo_smoothing"
	@echo "... src/demo_shape_detection/demo_dilatation_erosion.o"
	@echo "... src/demo_shape_detection/demo_dilatation_erosion.i"
	@echo "... src/demo_shape_detection/demo_dilatation_erosion.s"
	@echo "... src/demo_shape_detection/demo_filters.o"
	@echo "... src/demo_shape_detection/demo_filters.i"
	@echo "... src/demo_shape_detection/demo_filters.s"
	@echo "... src/demo_shape_detection/demo_full_example.o"
	@echo "... src/demo_shape_detection/demo_full_example.i"
	@echo "... src/demo_shape_detection/demo_full_example.s"
	@echo "... src/demo_shape_detection/demo_hsv_filter.o"
	@echo "... src/demo_shape_detection/demo_hsv_filter.i"
	@echo "... src/demo_shape_detection/demo_hsv_filter.s"
	@echo "... src/demo_shape_detection/demo_rgb_filter.o"
	@echo "... src/demo_shape_detection/demo_rgb_filter.i"
	@echo "... src/demo_shape_detection/demo_rgb_filter.s"
	@echo "... src/demo_shape_detection/demo_simple_shape_detection.o"
	@echo "... src/demo_shape_detection/demo_simple_shape_detection.i"
	@echo "... src/demo_shape_detection/demo_simple_shape_detection.s"
	@echo "... src/demo_shape_detection/demo_smoothing.o"
	@echo "... src/demo_shape_detection/demo_smoothing.i"
	@echo "... src/demo_shape_detection/demo_smoothing.s"
	@echo "... src/image_undistort.o"
	@echo "... src/image_undistort.i"
	@echo "... src/image_undistort.s"
	@echo "... src/student_interface.o"
	@echo "... src/student_interface.i"
	@echo "... src/student_interface.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
