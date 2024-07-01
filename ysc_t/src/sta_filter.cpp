#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "sta_filter");

    // std::vector<int> i_vec;
    // std::cout << "size = " << i_vec.size() << " capacity = " << i_vec.capacity() << std::endl;

    // i_vec.push_back(1);
    // std::cout << "size = " << i_vec.size() << " capacity = " << i_vec.capacity() << std::endl;

    // // i_vec.resize(5);
    // i_vec.reserve(10);
    // std::cout << "size = " << i_vec.size() << " capacity = " << i_vec.capacity() << std::endl;
    // std::cout << i_vec[2] << std::endl;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZINormal>);

    if (pcl::io::loadPCDFile("/home/lzq/map_pcd/cloud_result.pcd", *cloud) == -1)
    {
        ROS_ERROR("cannot load pcd file");
        return 0;
    }
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // pcl::PassThrough<pcl::PointXYZINormal> pass(true);
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-350, 30);
    // pass.setNegative(false);
    // pass.filter(*cloud_filtered);

    pcl::VoxelGrid<pcl::PointXYZINormal> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.3f, 0.3f, 0.3f);
    vg.filter(*cloud_filtered);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    pcl::io::savePCDFileBinary("/home/lzq/map_pcd/cloud_result_fil.pcd", *cloud_filtered);

    return 0;
}
