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
include workspace/test/CMakeFiles/ceres_gpt.dir/depend.make

# Include the progress variables for this target.
include workspace/test/CMakeFiles/ceres_gpt.dir/progress.make

# Include the compile flags for this target's objects.
include workspace/test/CMakeFiles/ceres_gpt.dir/flags.make

workspace/test/CMakeFiles/ceres_gpt.dir/ceres_test.cpp.o: workspace/test/CMakeFiles/ceres_gpt.dir/flags.make
workspace/test/CMakeFiles/ceres_gpt.dir/ceres_test.cpp.o: test/ceres_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/tmp1/workspace/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object workspace/test/CMakeFiles/ceres_gpt.dir/ceres_test.cpp.o"
	cd /tmp1/workspace/workspace/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ceres_gpt.dir/ceres_test.cpp.o -c /tmp1/workspace/test/ceres_test.cpp

workspace/test/CMakeFiles/ceres_gpt.dir/ceres_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ceres_gpt.dir/ceres_test.cpp.i"
	cd /tmp1/workspace/workspace/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /tmp1/workspace/test/ceres_test.cpp > CMakeFiles/ceres_gpt.dir/ceres_test.cpp.i

workspace/test/CMakeFiles/ceres_gpt.dir/ceres_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ceres_gpt.dir/ceres_test.cpp.s"
	cd /tmp1/workspace/workspace/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /tmp1/workspace/test/ceres_test.cpp -o CMakeFiles/ceres_gpt.dir/ceres_test.cpp.s

# Object files for target ceres_gpt
ceres_gpt_OBJECTS = \
"CMakeFiles/ceres_gpt.dir/ceres_test.cpp.o"

# External object files for target ceres_gpt
ceres_gpt_EXTERNAL_OBJECTS =

workspace/test/ceres_gpt: workspace/test/CMakeFiles/ceres_gpt.dir/ceres_test.cpp.o
workspace/test/ceres_gpt: workspace/test/CMakeFiles/ceres_gpt.dir/build.make
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libglog.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libfreeimage.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libmetis.so
workspace/test/ceres_gpt: /usr/local/lib/libceres.a
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libGL.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libGLU.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libGLEW.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
workspace/test/ceres_gpt: /usr/local/cuda/lib64/libcudart_static.a
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/librt.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libgmp.so
workspace/test/ceres_gpt: /usr/local/lib/libabsl_base.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_strings.a
workspace/test/ceres_gpt: workspace/feature/libimg.a
workspace/test/ceres_gpt: workspace/feature/libsift.a
workspace/test/ceres_gpt: workspace/test/libtest_util.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_random_distributions.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_random_seed_sequences.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_random_internal_pool_urbg.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_random_internal_randen.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_random_internal_randen_hwaes.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_random_internal_randen_hwaes_impl.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_random_internal_randen_slow.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_random_internal_platform.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_random_internal_seed_material.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_bad_optional_access.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_random_seed_gen_exception.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_str_format_internal.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_strings.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_strings_internal.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_base.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_spinlock_wait.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_int128.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_throw_delegate.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_raw_logging_internal.a
workspace/test/ceres_gpt: /usr/local/lib/libabsl_log_severity.a
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libfreeimage.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libmetis.so
workspace/test/ceres_gpt: /usr/local/lib/libceres.a
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libglog.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libspqr.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libcholmod.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libmetis.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libamd.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libcamd.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libccolamd.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libcolamd.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libtbb.so.2
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libcxsparse.so
workspace/test/ceres_gpt: /usr/local/cuda/lib64/libcublas.so
workspace/test/ceres_gpt: /usr/local/cuda/lib64/libcusolver.so
workspace/test/ceres_gpt: /usr/local/cuda/lib64/libcusparse.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/liblapack.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libf77blas.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libatlas.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libGL.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libGLU.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libGLEW.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
workspace/test/ceres_gpt: /usr/local/cuda/lib64/libcudart_static.a
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/librt.so
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.12.8
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.12.8
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.12.8
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libgmp.so
workspace/test/ceres_gpt: workspace/feature/libsift.a
workspace/test/ceres_gpt: workspace/feature/libimg.a
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
workspace/test/ceres_gpt: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
workspace/test/ceres_gpt: workspace/test/CMakeFiles/ceres_gpt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/tmp1/workspace/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ceres_gpt"
	cd /tmp1/workspace/workspace/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ceres_gpt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
workspace/test/CMakeFiles/ceres_gpt.dir/build: workspace/test/ceres_gpt

.PHONY : workspace/test/CMakeFiles/ceres_gpt.dir/build

workspace/test/CMakeFiles/ceres_gpt.dir/clean:
	cd /tmp1/workspace/workspace/test && $(CMAKE_COMMAND) -P CMakeFiles/ceres_gpt.dir/cmake_clean.cmake
.PHONY : workspace/test/CMakeFiles/ceres_gpt.dir/clean

workspace/test/CMakeFiles/ceres_gpt.dir/depend:
	cd /tmp1/workspace && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /tmp1 /tmp1/workspace/test /tmp1/workspace /tmp1/workspace/workspace/test /tmp1/workspace/workspace/test/CMakeFiles/ceres_gpt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : workspace/test/CMakeFiles/ceres_gpt.dir/depend

