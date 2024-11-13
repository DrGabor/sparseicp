#pragma once

#include <memory>
#include <vector>
#include <string>
#include <deque>
#include <Eigen/Dense>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>

typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign> SafeMatrix4d;
