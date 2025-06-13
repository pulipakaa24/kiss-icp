# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Find Open3D
find_package(Open3D REQUIRED CONFIG)

if (Open3D_FOUND)
    message(STATUS "Open3D found. Version: ${Open3D_VERSION}")
else()
    message(FATAL_ERROR "Open3D not found. Please install Open3D or set Open3D_DIR.")
endif()

add_library(kiss_icp_core STATIC)
target_sources(kiss_icp_core PRIVATE Registration.cpp VoxelHashMap.cpp VoxelUtils.cpp Preprocessing.cpp Threshold.cpp)
target_link_libraries(kiss_icp_core PUBLIC Eigen3::Eigen tsl::robin_map TBB::tbb Sophus::Sophus Open3D::Open3D)
set_global_target_properties(kiss_icp_core)
