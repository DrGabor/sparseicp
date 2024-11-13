#include <iostream>
#include <chrono>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/filesystem.hpp>
#include <pcl/io/ply_io.h>

#include "utils.h"
#include "ICP.h"
#include "io_obj.h"

DEFINE_int64(max_source_point_number, 10000, " ");
DEFINE_int64(max_target_point_number, 10000, " ");
DEFINE_string(source_file, "specify_one_file", " ");
DEFINE_string(target_file, "specify_one_file", " ");
DEFINE_string(output_file, "specify_one_file", " ");

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vertices;

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloudByPCL(const std::string& point_cloud_file) {
    auto file_extension = boost::filesystem::path(point_cloud_file).extension();
    LOG(INFO) << "file_extension = " << file_extension;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>());
    if(file_extension == ".pcd") {
        if (0 != pcl::io::loadPCDFile(point_cloud_file, *cloud_raw)) {
            LOG(FATAL) << "Couldn't read file " << point_cloud_file;
        }
    } else if(file_extension == ".ply") {
        if(0 != pcl::io::loadPLYFile(point_cloud_file, *cloud_raw)) {
            LOG(FATAL) << "Couldn't read file " << point_cloud_file;
        }
    } else {
        LOG(FATAL) << "un-support file format! " << file_extension;
    }
    if(cloud_raw->empty()) {
        LOG(FATAL) << "empty point cloud!";
    }
    // remove invalid point
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(cloud_raw->size());
    for(const auto& point : *cloud_raw) {
        if(std::isnan(point.x) || std::isinf(point.x) || 
           std::isnan(point.y) || std::isinf(point.y) || 
           std::isnan(point.z) || std::isinf(point.z) ) {
            continue;
        }
        // for stanford dataset, a lot of points are zeros
        double distance = Eigen::Vector3d{point.x, point.y, point.z}.norm();
        if(distance < 0.001) {
            continue;
        }
        cloud->push_back(point);
    }
    if(cloud->empty()) {
        LOG(FATAL) << "empty point cloud!";
    }
    cloud->width = 1;
    cloud->height = cloud->size();
    cloud->is_dense = true;
    LOG(INFO) << "load point cloud success, point_size = " << cloud->size() << ", remove " << cloud_raw->size() - cloud->size() << " points";
    return cloud;
}

Vertices loadPointCloud(const std::string& point_cloud_file, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, 
                        int max_point_number) {
    cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
    Vertices vertices;
    if(!boost::filesystem::exists(point_cloud_file)) {
        LOG(FATAL) << "file not exists! " << point_cloud_file;
    }
    // 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw = loadPointCloudByPCL(point_cloud_file);
        int N = cloud_raw->size();
        if(max_point_number >= N) {
            pcl::copyPointCloud(*cloud_raw, *cloud_filtered);
        } else {
            double random_sampling_ratio = double(max_point_number) / N;
            for(const auto& point : *cloud_raw) {
                double value = rand() / (1.0 + RAND_MAX);
                if(value < random_sampling_ratio) {
                    cloud_filtered->push_back(point);
                }
            }
            cloud_filtered->width = 1;
            cloud_filtered->height = cloud_filtered->size();
        }
    }
    //     
    int N = cloud_filtered->size();
    vertices.resize(Eigen::NoChange, N);
    for(int i = 0; i < N; ++i) {
        const auto& point = cloud_filtered->at(i);
        vertices.col(i) << point.x, point.y, point.z;
    }
    return vertices;
}

int main (int argc, char** argv)
{   
    FLAGS_logtostderr = 1;
    google::ParseCommandLineFlags(&argc, &argv, true);
    // Initialize Google’s logging library.
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Journey started!";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source;
    Vertices vertices_source = loadPointCloud(FLAGS_source_file, cloud_source, int(FLAGS_max_source_point_number));
    LOG(INFO) << "source: " << vertices_source.rows() << "x" << vertices_source.cols();
    ///--- Model that source will be aligned to
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target;
    Vertices vertices_target = loadPointCloud(FLAGS_target_file, cloud_target, int(FLAGS_max_source_point_number));
    LOG(INFO) << "target: " << vertices_target.rows() << "x" << vertices_target.cols();
    
    ///--- Execute registration
    auto tic = std::chrono::steady_clock::now(); 
        SICP::Parameters pars;
        pars.p = .5;
        pars.max_icp = 15;
        pars.print_icpn = true;
        SICP::point_to_point(vertices_source, vertices_target, pars);
    auto toc = std::chrono::steady_clock::now();
    ///--- Print execution time
    double time_ms = std::chrono::duration <double, std::milli> (toc-tic).count();
    LOG(INFO) << "sparseicp registered source to target in: " << time_ms << "ms";
    ///--- Write result to file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_aligned->reserve(vertices_source.cols());
    for(int i = 0; i < vertices_source.cols(); ++i) {
        pcl::PointXYZ point;
        point.x = vertices_source(0, i);
        point.y = vertices_source(1, i);
        point.z = vertices_source(2, i);
        cloud_aligned->push_back(point);
    }
    cloud_aligned->width = 1;
    cloud_aligned->height = cloud_aligned->size();
    // https://pointclouds.org/documentation/group__io.html
    if(0 != pcl::io::savePCDFileBinaryCompressed(FLAGS_output_file, *cloud_aligned)) {
        LOG(FATAL) << "cannot save cloud_aligned! " << FLAGS_output_file;
    }
    boost::filesystem::path save_root = boost::filesystem::path{FLAGS_output_file}.parent_path();
    boost::filesystem::path save_file = save_root / "cloud_source.pcd";
    if(0 != pcl::io::savePCDFileBinaryCompressed(save_file.string(), *cloud_source)) {
        LOG(FATAL) << "cannot save cloud_source! " << save_file;
    }
    save_file = save_root / "cloud_target.pcd";
    if(0 != pcl::io::savePCDFileBinaryCompressed(save_file.string(), *cloud_target)) {
        LOG(FATAL) << "cannot save cloud_target! " << save_file;
    }
    // write_obj_replaceverts(file_source.string(), vertices_source, file_source_reg.string());
    LOG(INFO) << "Journey finished";
    return 0;
}
