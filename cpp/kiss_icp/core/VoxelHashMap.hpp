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
//
// NOTE: This implementation is heavily inspired in the original CT-ICP VoxelHashMap implementation,
// although it was heavily modifed and drastically simplified, but if you are using this module you
// should at least acknoowledge the work from CT-ICP by giving a star on GitHub
#pragma once

#include <tsl/robin_map.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>

#include "VoxelUtils.hpp"

// ADDED Open3D INCLUDES
#include <open3d/io/PointCloudIO.h>
#include <open3d/geometry/PointCloud.h>

namespace kiss_icp {
struct VoxelHashMap {
    explicit VoxelHashMap(double voxel_size, double max_distance, unsigned int max_points_per_voxel)
        : voxel_size_(voxel_size),
          max_distance_(max_distance),
          max_points_per_voxel_(max_points_per_voxel) {}

    VoxelHashMap(const std::string &map_path, double voxel_size, double max_distance, unsigned int max_points_per_voxel)
      : voxel_size_(voxel_size), max_distance_(max_distance), max_points_per_voxel_(max_points_per_voxel) {
        // --- MODIFIED: Use Open3D to load the point cloud ---
        auto cloud_open3d = std::make_shared<open3d::geometry::PointCloud>();
        if (!open3d::io::ReadPointCloud(map_path, *cloud_open3d)) {
            throw std::runtime_error("Failed to load point cloud file with Open3D: " + map_path);
        }

        // Open3D's PointCloud::points_ is already a std::vector<Eigen::Vector3d>,
        // which matches what AddPoints expects, so no manual conversion loop is needed.
        AddPoints(cloud_open3d->points_);
        // --- END MODIFIED ---
      }

    inline void Clear() { map_.clear(); }
    inline bool Empty() const { return map_.empty(); }
    void Update(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &origin);
    void Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose);
    void AddPoints(const std::vector<Eigen::Vector3d> &points);
    void RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    std::vector<Eigen::Vector3d> Pointcloud() const;
    std::tuple<Eigen::Vector3d, double> GetClosestNeighbor(const Eigen::Vector3d &query) const;

    double voxel_size_;
    double max_distance_;
    unsigned int max_points_per_voxel_;
    tsl::robin_map<Voxel, std::vector<Eigen::Vector3d>> map_;
};
}  // namespace kiss_icp