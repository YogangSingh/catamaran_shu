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
CMAKE_SOURCE_DIR = /home/kiitan/catamaran_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kiitan/catamaran_ws/build

# Utility rule file for gflags_src.

# Include the progress variables for this target.
include gflags_catkin/CMakeFiles/gflags_src.dir/progress.make

gflags_catkin/CMakeFiles/gflags_src: gflags_catkin/CMakeFiles/gflags_src-complete


gflags_catkin/CMakeFiles/gflags_src-complete: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-install
gflags_catkin/CMakeFiles/gflags_src-complete: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-mkdir
gflags_catkin/CMakeFiles/gflags_src-complete: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-download
gflags_catkin/CMakeFiles/gflags_src-complete: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-update
gflags_catkin/CMakeFiles/gflags_src-complete: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-patch
gflags_catkin/CMakeFiles/gflags_src-complete: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-configure
gflags_catkin/CMakeFiles/gflags_src-complete: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-build
gflags_catkin/CMakeFiles/gflags_src-complete: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kiitan/catamaran_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'gflags_src'"
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E make_directory /home/kiitan/catamaran_ws/build/gflags_catkin/CMakeFiles
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E touch /home/kiitan/catamaran_ws/build/gflags_catkin/CMakeFiles/gflags_src-complete
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E touch /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-done

gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-install: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kiitan/catamaran_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing install step for 'gflags_src'"
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-build && cd ../gflags_src && make install -j 8
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-build && /usr/bin/cmake -E touch /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-install

gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kiitan/catamaran_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'gflags_src'"
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E make_directory /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E make_directory /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-build
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E make_directory /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E make_directory /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/tmp
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E make_directory /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E make_directory /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E make_directory /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E touch /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-mkdir

gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-download: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-urlinfo.txt
gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-download: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kiitan/catamaran_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (download, verify and extract) for 'gflags_src'"
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src && /usr/bin/cmake -P /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/download-gflags_src.cmake
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src && /usr/bin/cmake -P /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/verify-gflags_src.cmake
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src && /usr/bin/cmake -P /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/extract-gflags_src.cmake
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src && /usr/bin/cmake -E touch /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-download

gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-update: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kiitan/catamaran_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No update step for 'gflags_src'"
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src && /usr/bin/cmake -E echo_append
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src && /usr/bin/cmake -E touch /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-update

gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-patch: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kiitan/catamaran_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'gflags_src'"
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E echo_append
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && /usr/bin/cmake -E touch /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-patch

gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-configure: gflags_catkin/gflags_src-prefix/tmp/gflags_src-cfgcmd.txt
gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-configure: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-update
gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-configure: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kiitan/catamaran_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing configure step for 'gflags_src'"
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-build && cd ../gflags_src && cmake . -DCMAKE_INSTALL_PREFIX:PATH=/home/kiitan/catamaran_ws/devel -DBUILD_SHARED_LIBS:BOOL=true -DGFLAGS_NAMESPACE:STRING=google -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_TOOLCHAIN_FILE:STRING=
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-build && /usr/bin/cmake -E touch /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-configure

gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-build: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kiitan/catamaran_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing build step for 'gflags_src'"
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-build && cd ../gflags_src && make -j 8
	cd /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-build && /usr/bin/cmake -E touch /home/kiitan/catamaran_ws/build/gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-build

gflags_src: gflags_catkin/CMakeFiles/gflags_src
gflags_src: gflags_catkin/CMakeFiles/gflags_src-complete
gflags_src: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-install
gflags_src: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-mkdir
gflags_src: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-download
gflags_src: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-update
gflags_src: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-patch
gflags_src: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-configure
gflags_src: gflags_catkin/gflags_src-prefix/src/gflags_src-stamp/gflags_src-build
gflags_src: gflags_catkin/CMakeFiles/gflags_src.dir/build.make

.PHONY : gflags_src

# Rule to build all files generated by this target.
gflags_catkin/CMakeFiles/gflags_src.dir/build: gflags_src

.PHONY : gflags_catkin/CMakeFiles/gflags_src.dir/build

gflags_catkin/CMakeFiles/gflags_src.dir/clean:
	cd /home/kiitan/catamaran_ws/build/gflags_catkin && $(CMAKE_COMMAND) -P CMakeFiles/gflags_src.dir/cmake_clean.cmake
.PHONY : gflags_catkin/CMakeFiles/gflags_src.dir/clean

gflags_catkin/CMakeFiles/gflags_src.dir/depend:
	cd /home/kiitan/catamaran_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kiitan/catamaran_ws/src /home/kiitan/catamaran_ws/src/gflags_catkin /home/kiitan/catamaran_ws/build /home/kiitan/catamaran_ws/build/gflags_catkin /home/kiitan/catamaran_ws/build/gflags_catkin/CMakeFiles/gflags_src.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gflags_catkin/CMakeFiles/gflags_src.dir/depend
