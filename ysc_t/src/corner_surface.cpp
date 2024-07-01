// #include "tic_toc.h"
#include <ros/ros.h>
#include <iostream>
#include <cmath>

// #include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>

#include <vector>
#include <algorithm>

using namespace std;
typedef pcl::PointXYZI PointType;

struct smoothness_t
{
    float value;
    size_t ind;
};

struct by_value
{
    bool operator()(smoothness_t const &left, smoothness_t const &right)
    {
        return left.value < right.value;
    }
};

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

using std::atan2;
using std::cos;
using std::sin;

double scanPeriod = 0.1;
double odometrySurfLeafSize = 0.4;

int N_SCAN;
int Horizon_SCAN;
float lidarMinRange = 1.0;
float edgeThreshold = 0.1;
float surfThreshold = 0.1;

class scanRegistration
{

private:
    ros::NodeHandle nh;
    pcl::PointCloud<PointType>::Ptr surfaceCloud2;
    pcl::PointCloud<PointXYZIRPYT> cloudKeyPose;
    pcl::VoxelGrid<PointType> downSizeFilter;

    vector<smoothness_t> cloudSmoothness;
    vector<float> cloudCurvature;
    vector<int> cloudNeighborPicked;
    vector<int> cloudLabel;
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudKeyFrame;

    string keyframe_dir;

    // vector<pcl::PointCloud<PointType>> lidarCloudScans;

public:
    scanRegistration(string dir, int N_scan, int H_scan);
    void markBadPoints(pcl::PointCloud<PointType>::Ptr cloudIn);
    void lidarCloudHandler();
    void keyframeHandler(pcl::PointCloud<PointType>::Ptr lidarCloudIn, int intensity);
    pcl::PointCloud<PointType> transformCloud(pcl::PointCloud<PointType>::Ptr cloud_in, PointXYZIRPYT pose);
};

scanRegistration::scanRegistration(string dir, int N_scan, int H_scan)
{
    downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);
    keyframe_dir = dir;
    N_SCAN = N_scan;
    Horizon_SCAN = H_scan;

    // to avoid overflow (sometimes points in one frame can be a lot)
    cloudCurvature.resize(N_SCAN * Horizon_SCAN * 10);
    cloudNeighborPicked.resize(N_SCAN * Horizon_SCAN * 10);
    cloudLabel.resize(N_SCAN * Horizon_SCAN * 10);
    cloudSmoothness.resize(N_SCAN * Horizon_SCAN * 10);
}

void scanRegistration::lidarCloudHandler()
{

    // TicToc t_whole;
    // TicToc t_prepare;

    string dir = keyframe_dir + "/optimized_poses.txt";
    ifstream fin(dir);
    if (!fin.is_open())
    {
        ROS_ERROR("file is not valid !");
        return;
    }

    int ind = 0;
    while (true)
    {
        float time_stamp;
        PointXYZIRPYT point;
        Eigen::Quaterniond que;
        fin >> time_stamp >> point.x >> point.y >> point.z >> point.roll >> point.pitch >> point.yaw;

        // Eigen::Vector3d euler = que.toRotationMatrix().eulerAngles(0, 1, 2);
        // point.roll = euler[0];
        // point.pitch = euler[1];
        // point.yaw = euler[2];
        point.intensity = ind;

        if (fin.peek() == EOF)
            break;
        else
        {
            // cout << "123" << endl;
            cloudKeyPose.push_back(point);
            ind++;
        }
    }
    ROS_INFO("all %ld keyframes", cloudKeyPose.size());
    for (int i = 0; i < cloudKeyPose.size(); i++)
    {
        string keyframeName = keyframe_dir + "/lcd_rawcloud" + to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr keyframe(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<PointType>(keyframeName, *keyframe) == -1)
            cout << "Couldn't read file" + keyframeName << endl;

        *keyframe = transformCloud(keyframe, cloudKeyPose[i]);
        cloudKeyFrame.push_back(keyframe);
    }

    for (int i = 0; i < cloudKeyFrame.size(); i++)
    {
        keyframeHandler(cloudKeyFrame[i], cloudKeyPose.points[i].intensity);
        if (i % 10 == 0)
            ROS_INFO("caculating %d / %ld cloudkeyframe", i, cloudKeyPose.size());
    }

    ofstream pose_file;
    pose_file.open(keyframe_dir + "/keyframes/poses.txt", ios::out); // downsampled
    if (!pose_file.is_open())
    {
        std::cout << "Cannot open" << keyframe_dir + "/keyframes/poses.txt" << std::endl;
        return;
    }

    for (int i = 0; i < cloudKeyPose.size(); i++)
    {
        pose_file << cloudKeyPose.points[i].x << " " << cloudKeyPose.points[i].y << " " << cloudKeyPose.points[i].z
                  << " " << cloudKeyPose.points[i].roll << " " << cloudKeyPose.points[i].pitch << " " << cloudKeyPose.points[i].yaw
                  << " " << i << " " << 0 /* indoor 1; outdoor 0; */ << "\n";
    }

    ROS_INFO("\033[1;32m----> Lidar Scan Registration Completed.\033[0m");
}

pcl::PointCloud<PointType> scanRegistration::transformCloud(pcl::PointCloud<PointType>::Ptr cloud_in, PointXYZIRPYT pose)
{
    pcl::PointCloud<PointType> cloud_out;
    cloud_out.resize(cloud_in->size());
    Eigen::Affine3f trans = pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
    Eigen::Affine3f trans_invers = trans.inverse();

    for (int i = 0; i < cloud_in->size(); ++i)
    {
        const auto &pointFrom = cloud_in->points[i];
        cloud_out.points[i].x = trans_invers(0, 0) * pointFrom.x + trans_invers(0, 1) * pointFrom.y + trans_invers(0, 2) * pointFrom.z + trans_invers(0, 3);
        cloud_out.points[i].y = trans_invers(1, 0) * pointFrom.x + trans_invers(1, 1) * pointFrom.y + trans_invers(1, 2) * pointFrom.z + trans_invers(1, 3);
        cloud_out.points[i].z = trans_invers(2, 0) * pointFrom.x + trans_invers(2, 1) * pointFrom.y + trans_invers(2, 2) * pointFrom.z + trans_invers(2, 3);
        cloud_out.points[i].intensity = pointFrom.intensity;
    }

    return cloud_out;
}

void scanRegistration::keyframeHandler(pcl::PointCloud<PointType>::Ptr lidarCloudIn, int intensity)
{
    // pcl::PointCloud<PointType>::Ptr lidarCloudIn(new pcl::PointCloud<PointType>());
    std::vector<int> scanStartInd(N_SCAN, 0);
    std::vector<int> scanEndInd(N_SCAN, 0);
    std::vector<int> indices;

    downSizeFilter.setInputCloud(lidarCloudIn);
    downSizeFilter.filter(*lidarCloudIn);

    pcl::removeNaNFromPointCloud(*lidarCloudIn, *lidarCloudIn, indices);

    int cloudSize = lidarCloudIn->points.size();
    if (cloudSize < 1000)
    {
        ROS_WARN("Empty cloud or points too few");
        return;
    }
    if (cloudSize > 2 * N_SCAN * Horizon_SCAN)
    {
        // ROS_WARN("Points too many");
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/big_cloud.pcd", *lidarCloudIn);
    }
    float startOri = -atan2(lidarCloudIn->points[0].y, lidarCloudIn->points[0].x);
    float endOri = -atan2(lidarCloudIn->points[cloudSize - 1].y, lidarCloudIn->points[cloudSize - 1].x) + 2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> lidarCloudScans(N_SCAN);
    cv::Mat rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

    for (int i = 0; i < cloudSize; i++)
    {
        point.x = lidarCloudIn->points[i].x;
        point.y = lidarCloudIn->points[i].y;
        point.z = lidarCloudIn->points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCAN == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCAN - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCAN == 32)
        {
            scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
            if (scanID > (N_SCAN - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCAN == 64)
        {
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCAN / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCAN == 256)
        {
            float angleResolution = (16.0 + 48.0) / 256.0; // 128线LiDAR的角度分辨率
            scanID = int((angle + 16) / angleResolution);

            if (scanID > (N_SCAN - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }

        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + scanPeriod * relTime;
        lidarCloudScans[scanID].push_back(point);
    }
    // printf("Before projection, points size: %d \n", cloudSize);

    pcl::PointCloud<PointType>::Ptr lidarCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCAN; i++)
    {
        scanStartInd[i] = lidarCloud->size() + 5;
        *lidarCloud += lidarCloudScans[i];
        scanEndInd[i] = lidarCloud->size() - 6;
    }

    cloudSize = lidarCloud->size();
    // cout<<"After projection, point size: "<<lidarCloud->size()<<endl;
    // printf("prepare time %f \n", t_prepare.toc());

    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffX = lidarCloud->points[i - 5].x + lidarCloud->points[i - 4].x + lidarCloud->points[i - 3].x + lidarCloud->points[i - 2].x + lidarCloud->points[i - 1].x - 10 * lidarCloud->points[i].x + lidarCloud->points[i + 1].x + lidarCloud->points[i + 2].x + lidarCloud->points[i + 3].x + lidarCloud->points[i + 4].x + lidarCloud->points[i + 5].x;
        float diffY = lidarCloud->points[i - 5].y + lidarCloud->points[i - 4].y + lidarCloud->points[i - 3].y + lidarCloud->points[i - 2].y + lidarCloud->points[i - 1].y - 10 * lidarCloud->points[i].y + lidarCloud->points[i + 1].y + lidarCloud->points[i + 2].y + lidarCloud->points[i + 3].y + lidarCloud->points[i + 4].y + lidarCloud->points[i + 5].y;
        float diffZ = lidarCloud->points[i - 5].z + lidarCloud->points[i - 4].z + lidarCloud->points[i - 3].z + lidarCloud->points[i - 2].z + lidarCloud->points[i - 1].z - 10 * lidarCloud->points[i].z + lidarCloud->points[i + 1].z + lidarCloud->points[i + 2].z + lidarCloud->points[i + 3].z + lidarCloud->points[i + 4].z + lidarCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ; // 当前点与前后五个点的坐标差的平方和
        cloudSmoothness[i].ind = i;
        cloudSmoothness[i].value = cloudCurvature[i];
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }
    // cout<<"smoothness calculated"<<endl;

    markBadPoints(lidarCloud);
    // cout<<"After removing bad points, point size: "<<lidarCloud->size()<<endl;
    // TicToc t_pts;

    pcl::PointCloud<PointType>::Ptr cornerCloudSharp(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cornerCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudFlat(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloud(new pcl::PointCloud<PointType>());

    float t_q_sort = 0;
    float t_filter = 0;
    for (int i = 0; i < N_SCAN; i++)
    {
        if (scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<PointType>::Ptr surfaceCloudTmp(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            // TicToc t_tmp;
            std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep + 1, by_value());
            // t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;

            for (int k = ep; k >= sp; k--) // cout<<"start filtering"<<endl;
            {
                int ind = cloudSmoothness[k].ind;

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > edgeThreshold)
                {
                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {
                        cloudLabel[ind] = 2;
                        cornerCloudSharp->push_back(lidarCloud->points[ind]);
                        cornerCloud->push_back(lidarCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {
                        cloudLabel[ind] = 1;
                        cornerCloud->push_back(lidarCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = lidarCloud->points[ind + l].x - lidarCloud->points[ind + l - 1].x;
                        float diffY = lidarCloud->points[ind + l].y - lidarCloud->points[ind + l - 1].y;
                        float diffZ = lidarCloud->points[ind + l].z - lidarCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = lidarCloud->points[ind + l].x - lidarCloud->points[ind + l + 1].x;
                        float diffY = lidarCloud->points[ind + l].y - lidarCloud->points[ind + l + 1].y;
                        float diffZ = lidarCloud->points[ind + l].z - lidarCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSmoothness[k].ind;

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < surfThreshold)
                {

                    cloudLabel[ind] = -1;
                    surfaceCloudFlat->push_back(lidarCloud->points[ind]);
                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = lidarCloud->points[ind + l].x - lidarCloud->points[ind + l - 1].x;
                        float diffY = lidarCloud->points[ind + l].y - lidarCloud->points[ind + l - 1].y;
                        float diffZ = lidarCloud->points[ind + l].z - lidarCloud->points[ind + l - 1].z;

                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }

                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = lidarCloud->points[ind + l].x - lidarCloud->points[ind + l + 1].x;
                        float diffY = lidarCloud->points[ind + l].y - lidarCloud->points[ind + l + 1].y;
                        float diffZ = lidarCloud->points[ind + l].z - lidarCloud->points[ind + l + 1].z;

                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfaceCloudTmp->push_back(lidarCloud->points[k]);
                }
            }
        }
        // TicToc tmp;

        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        downSizeFilter.setInputCloud(surfaceCloudTmp);
        downSizeFilter.filter(*surfaceCloudScanDS);

        // t_filter += tmp.toc();
        *surfaceCloud += *surfaceCloudScanDS;
    }
    // cout << "corner.size = " << cornerCloud->size() << endl;
    // cout << "surf.size = " << surfaceCloud->size() << endl;
    pcl::io::savePCDFile(keyframe_dir + "/keyframes/corner" + to_string(intensity) + ".pcd", *cornerCloud);
    pcl::io::savePCDFile(keyframe_dir + "/keyframes/surf" + to_string(intensity) + ".pcd", *surfaceCloud);
}

void scanRegistration::markBadPoints(pcl::PointCloud<PointType>::Ptr cloudIn)
{
    int cloudSize = cloudIn->size();
    for (int i = 5; i < cloudSize - 6; i++)
    {

        float diffX = cloudIn->points[i + 1].x - cloudIn->points[i].x;
        float diffY = cloudIn->points[i + 1].y - cloudIn->points[i].y;
        float diffZ = cloudIn->points[i + 1].z - cloudIn->points[i].z;

        float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;
        // 0.2 horizontal angle accuracy, so dist > 0.1/(0.2/57.3) = 28.65 m
        if (diff > lidarMinRange)
        {
            float depth1 = sqrt(cloudIn->points[i].x * cloudIn->points[i].x +
                                cloudIn->points[i].y * cloudIn->points[i].y +
                                cloudIn->points[i].z * cloudIn->points[i].z);

            float depth2 = sqrt(cloudIn->points[i + 1].x * cloudIn->points[i + 1].x +
                                cloudIn->points[i + 1].y * cloudIn->points[i + 1].y +
                                cloudIn->points[i + 1].z * cloudIn->points[i + 1].z);

            if (depth1 > depth2)
            {
                diffX = cloudIn->points[i + 1].x - cloudIn->points[i].x * depth2 / depth1;
                diffY = cloudIn->points[i + 1].y - cloudIn->points[i].y * depth2 / depth1;
                diffZ = cloudIn->points[i + 1].z - cloudIn->points[i].z * depth2 / depth1;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1)
                {

                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
            }
            else
            {
                diffX = cloudIn->points[i + 1].x * depth1 / depth2 - cloudIn->points[i].x;
                diffY = cloudIn->points[i + 1].y * depth1 / depth2 - cloudIn->points[i].y;
                diffZ = cloudIn->points[i + 1].z * depth1 / depth2 - cloudIn->points[i].z;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1)
                {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
        }

        float diffX2 = cloudIn->points[i].x - cloudIn->points[i - 1].x;
        float diffY2 = cloudIn->points[i].y - cloudIn->points[i - 1].y;
        float diffZ2 = cloudIn->points[i].z - cloudIn->points[i - 1].z;

        float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
        float dis = cloudIn->points[i].x * cloudIn->points[i].x + cloudIn->points[i].y * cloudIn->points[i].y + cloudIn->points[i].z * cloudIn->points[i].z;
        if (diff > 0.0002 * dis && diff2 > 0.0002 * dis)
        {
            cloudNeighborPicked[i] = 1;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");

    ROS_INFO("\033[1;32m----> Lidar Scan Registration Started.\033[0m");
    scanRegistration SR("/home/lzq/ysc_ws/logs/keyframes", 256, 1800);
    SR.lidarCloudHandler();

    // ros::spin();

    return 0;
}
