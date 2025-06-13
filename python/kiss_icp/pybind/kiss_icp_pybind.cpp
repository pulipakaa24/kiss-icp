// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/Threshold.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"
#include "kiss_icp/metrics/Metrics.hpp"
#include "stl_vector_eigen.h"

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);

namespace kiss_icp {
PYBIND11_MODULE(kiss_icp_pybind, m) {
    auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
        m, "_Vector3dVector", "std::vector<Eigen::Vector3d>",
        py::py_array_to_vectors_double<Eigen::Vector3d>);

    // Map representation
    py::class_<VoxelHashMap> internal_map(m, "_VoxelHashMap", "Don't use this");
    internal_map
        .def(py::init<double, double, int>(), "voxel_size"_a, "max_distance"_a,
             "max_points_per_voxel"_a)
        .def(py::init<const std::string&, double, double, unsigned int>(), "map_path"_a,
            "voxel_size"_a, "max_distance"_a, "max_points_per_voxel"_a)

        .def("_clear", &VoxelHashMap::Clear)
        .def("_empty", &VoxelHashMap::Empty)
        .def("_update",
             py::overload_cast<const std::vector<Eigen::Vector3d> &, const Eigen::Vector3d &>(
                 &VoxelHashMap::Update),
             "points"_a, "origin"_a)
        .def(
            "_update",
            [](VoxelHashMap &self, const std::vector<Eigen::Vector3d> &points,
               const Eigen::Matrix4d &T) {
                Sophus::SE3d pose(T);
                self.Update(points, pose);
            },
            "points"_a, "pose"_a)
        .def("_add_points", &VoxelHashMap::AddPoints, "points"_a)
        .def("_remove_far_away_points", &VoxelHashMap::RemovePointsFarFromLocation, "origin"_a)
        .def("_point_cloud", &VoxelHashMap::Pointcloud);

    py::class_<Preprocessor> internal_preprocessor(m, "_Preprocessor", "Don't use this");
    internal_preprocessor
        .def(py::init<double, double, bool, int>(), "max_range"_a, "min_range"_a, "deskew"_a,
             "max_num_threads"_a)
        .def(
            "_preprocess",
            [](Preprocessor &self, const std::vector<Eigen::Vector3d> &points,
               const std::vector<double> &timestamps, const Eigen::Matrix4d &relative_motion) {
                Sophus::SE3d motion(relative_motion);
                return self.Preprocess(points, timestamps, motion);
            },
            "points"_a, "timestamps"_a, "relative_motion"_a);

    // Point Cloud registration
    py::class_<Registration> internal_registration(m, "_Registration", "Don't use this");
    internal_registration
        .def(py::init<int, double, int>(), "max_num_iterations"_a, "convergence_criterion"_a,
             "max_num_threads"_a)
        .def(
            "_align_points_to_map",
            [](Registration &self, const std::vector<Eigen::Vector3d> &points,
               const VoxelHashMap &voxel_map, const Eigen::Matrix4d &T_guess,
               double max_correspondence_distance, double kernel) {
                Sophus::SE3d initial_guess(T_guess);
                return self
                    .AlignPointsToMap(points, voxel_map, initial_guess, max_correspondence_distance,
                                      kernel)
                    .matrix();
            },
            "points"_a, "voxel_map"_a, "initial_guess"_a, "max_correspondance_distance"_a,
            "kernel"_a);

    // AdaptiveThreshold bindings
    py::class_<AdaptiveThreshold> adaptive_threshold(m, "_AdaptiveThreshold", "Don't use this");
    adaptive_threshold
        .def(py::init<double, double, double>(), "initial_threshold"_a, "min_motion_th"_a,
             "max_range"_a)
        .def("_compute_threshold", &AdaptiveThreshold::ComputeThreshold)
        .def(
            "_update_model_deviation",
            [](AdaptiveThreshold &self, const Eigen::Matrix4d &T) {
                Sophus::SE3d model_deviation(T);
                self.UpdateModelDeviation(model_deviation);
            },
            "model_deviation"_a);

    // prerpocessing modules
    m.def("_voxel_down_sample", &VoxelDownsample, "frame"_a, "voxel_size"_a);
    /// This function only applies for the KITTI dataset, and should NOT be used by any other
    /// dataset, the original idea and part of the implementation is taking from CT-ICP(Although
    /// IMLS-SLAM Originally introduced the calibration factor)
    m.def(
        "_correct_kitti_scan",
        [](const std::vector<Eigen::Vector3d> &frame) {
            constexpr double VERTICAL_ANGLE_OFFSET = (0.205 * M_PI) / 180.0;
            std::vector<Eigen::Vector3d> frame_ = frame;
            std::transform(frame_.cbegin(), frame_.cend(), frame_.begin(), [&](const auto pt) {
                const Eigen::Vector3d rotationVector = pt.cross(Eigen::Vector3d(0., 0., 1.));
                return Eigen::AngleAxisd(VERTICAL_ANGLE_OFFSET, rotationVector.normalized()) * pt;
            });
            return frame_;
        },
        "frame"_a);

    // Metrics
    m.def("_kitti_seq_error", &metrics::SeqError, "gt_poses"_a, "results_poses"_a);
    m.def("_absolute_trajectory_error", &metrics::AbsoluteTrajectoryError, "gt_poses"_a,
          "results_poses"_a);
}

}  // namespace kiss_icp
