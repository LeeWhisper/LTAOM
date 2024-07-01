#include "include/std.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

int findPoseIndexUsingTime(std::vector<double> &time_list, long &time) {
  long time_inc = 10000000000;
  int min_index = -1;
  for (size_t i = 0; i < time_list.size(); i++) {
    if (fabs(time_list[i] - time) < time_inc) {
      time_inc = fabs(time_list[i] - time);
      min_index = i;
    }
  }
  return min_index;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "std_loop");
  ros::NodeHandle nh;
  std::string data_name = "";
  std::string setting_path = "";
  std::string bag_file = "";
  std::string pose_file = "";
  std::string result_path = "";
  double icp_threshold = 0.5;
  nh.param<std::string>("data_name", data_name, "");
  nh.param<std::string>("setting_path", setting_path, "");
  nh.param<std::string>("bag_file", bag_file, "");
  nh.param<std::string>("pose_file", pose_file, "");
  nh.param<std::string>("result_path", result_path, "");

  std::ofstream result_file(result_path);
  std::ofstream debug_file("/home/ycj/catkin_ws/src/STD/Log/log.txt");
  std::ofstream debug_augment("/home/ycj/catkin_ws/src/STD/Log/augument.txt");

  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  ros::Publisher pubRegisterCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  ros::Publisher pubCureentCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  ros::Publisher pubCurrentBinary =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  ros::Publisher pubMatchedCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  ros::Publisher pubMatchedBinary =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  ros::Publisher pubSTD =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);

  ros::Rate loop(1000);

  ConfigSetting config_setting;
  load_config_setting(setting_path, config_setting);

  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_list;
  std::vector<double> time_list;
  load_cu_pose_with_time(pose_file, pose_list, time_list);
  std::string print_msg = "Sucessfully load pose file:" + pose_file +
                          ". pose size:" + std::to_string(time_list.size());
  ROS_INFO_STREAM(print_msg.c_str());

  // save all point clouds of key frame
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_list;

  // save all planes of key frame
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> history_plane_list;

  // save all binary descriptors of key frame
  std::vector<std::vector<BinaryDescriptor>> history_binary_list;

  // save all STD descriptors of key frame
  std::vector<std::vector<STD>> history_STD_list;

  // save all poses(translation only) of key frame
  std::vector<Eigen::Vector3d> key_pose_list;

  // hash table, save all descriptor
  std::unordered_map<STD_LOC, std::vector<STD>> STD_map;

  // calc mean time
  double mean_time = 0;

  // record mean position
  Eigen::Vector3d current_mean_position(0, 0, 0);

  long current_time = 0;

  // load lidar point cloud and start loop
  // load lidar point cloud and start loop
  int pose_cnt = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pose_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_key_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  bool is_build_descriptor = false;
  int key_frame_id = 0;
  std::fstream file_;
  file_.open(bag_file, std::ios::in);
  if (!file_) {
    std::cout << "File " << bag_file << " does not exit" << std::endl;
  }
  ROS_INFO("Start to load the rosbag %s", bag_file.c_str());
  rosbag::Bag bag;
  try {
    bag.open(bag_file, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
  }
  std::vector<std::string> topics;
  topics.push_back(std::string("/ns1/velodyne_points"));
  topics.push_back(std::string("/ns2/velodyne_points"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  bool is_init_bag = false;
  bool is_skip_frame = false;
  Eigen::Vector3d init_translation;
  Eigen::Vector3d last_translation;
  Eigen::Quaterniond last_q;
  int count = 0;
  Eigen::Vector3d vehicle_to_velo_t;
  Eigen::Matrix3d vehicle_to_velo_rot;
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    sensor_msgs::PointCloud2::ConstPtr cloud_ptr =
        m.instantiate<sensor_msgs::PointCloud2>();
    if (cloud_ptr != NULL) {
      if (cloud_ptr->header.frame_id == "left_velodyne") {
        vehicle_to_velo_t << -0.31189, 0.394734, 1.94661;
        vehicle_to_velo_rot << -0.514169, -0.702457, -0.492122, 0.48979,
            -0.711497, 0.503862, -0.704085, 0.0180335, 0.709886;
      } else {
        vehicle_to_velo_t << -0.306052, -0.417145, 1.95223;
        vehicle_to_velo_rot << -0.507842, 0.704544, -0.495695, -0.49974,
            -0.709646, -0.496651, -0.701681, -0.00450156, 0.712477;
      }

      if (count == 0) {
        if (!is_skip_frame) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
              new pcl::PointCloud<pcl::PointXYZI>);
          key_cloud_list.push_back(temp_cloud);
          current_key_cloud->clear();
        }
      }
      long laser_time = cloud_ptr->header.stamp.toNSec();
      pcl::PCLPointCloud2 pcl_pc;
      pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
      pcl_conversions::toPCL(*cloud_ptr, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, pcl_cloud);
      int pose_index = findPoseIndexUsingTime(time_list, laser_time);
      if (pose_index < 0) {
        is_skip_frame = true;
        std::cout << "!!!!!!!!!! skip frame!!!!!!!!!!!" << std::endl;
        continue;
      }
      is_skip_frame = false;
      Eigen::Vector3d translation = pose_list[pose_index].first;
      Eigen::Matrix3d rotation = pose_list[pose_index].second;
      Eigen::Quaterniond q(rotation);
      if (!is_init_bag) {
        init_translation = translation;
        translation << 0, 0, 0;
        is_init_bag = true;
        last_translation = translation;
        last_q = q;
      } else {
        translation = translation - init_translation;
      }
      nav_msgs::Odometry odom;
      odom.header.frame_id = "camera_init";
      odom.pose.pose.position.x = translation[0];
      odom.pose.pose.position.y = translation[1];
      odom.pose.pose.position.z = translation[2];
      odom.pose.pose.orientation.w = q.w();
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      pubOdomAftMapped.publish(odom);
      loop.sleep();

      if (config_setting.stop_skip_enable_) {
        Eigen::Vector3d position_inc;
        position_inc = translation - last_translation;
        double rotation_inc = q.angularDistance(last_q);
        if (position_inc.norm() < 0.2 && rotation_inc < DEG2RAD(5)) {
          is_skip_frame = true;
          continue;
        }
        last_translation = translation;
        last_q = q;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud(
          new pcl::PointCloud<pcl::PointXYZI>);
      for (size_t i = 0; i < pcl_cloud.size(); i++) {
        Eigen::Vector3d pv(pcl_cloud.points[i].x, pcl_cloud.points[i].y,
                           pcl_cloud.points[i].z);
        pv = vehicle_to_velo_rot * pv + vehicle_to_velo_t;
        pv = rotation * pv + translation;
        pcl::PointXYZI pi = pcl_cloud.points[i];
        pi.x = pv[0];
        pi.y = pv[1];
        pi.z = pv[2];
        register_cloud->push_back(pi);
      }

      down_sampling_voxel(*register_cloud, config_setting.ds_size_);
      for (size_t i = 0; i < register_cloud->size(); i++) {
        key_cloud_list.back()->points.push_back(register_cloud->points[i]);
      }
      if (count == config_setting.sub_frame_num_ / 2) {
        current_time = cloud_ptr->header.stamp.toNSec() / 1000;
        current_mean_position = translation;
        pose_cnt++;
        pcl::PointXYZI pi;
        pi.x = current_mean_position[0];
        pi.y = current_mean_position[1];
        pi.z = current_mean_position[2];
        pi.intensity = pose_cnt;
        pose_cloud->points.push_back(pi);
      }
      if (count < config_setting.sub_frame_num_ - 1) {
        count++;
      } else {
        count = 0;
        is_build_descriptor = true;
      }
      if (is_build_descriptor) {
        is_build_descriptor = false;
        down_sampling_voxel(*key_cloud_list[key_frame_id], 0.25);
        std::cout << "key frame:" << key_frame_id
                  << ", cloud size:" << key_cloud_list[key_frame_id]->size()
                  << std::endl;
        key_frame_id++;
      }
    }
  }
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZI>);
  kd_tree->setInputCloud(pose_cloud);
  std::vector<int> indices;
  std::vector<float> distances;
  double radius = 30;
  double overlap_threshold = 0.4;
  int gt_loop_num = 0;
  for (int i = 0; i < pose_cloud->size(); i++) {
    double max_overlap = 0;
    bool trigger_loop = false;
    int loop_id = 0;
    pcl::PointXYZI searchPoint = pose_cloud->points[i];
    int size = kd_tree->radiusSearch(searchPoint, radius, indices, distances);
    for (int j = 0; j < size; j++) {
      if (indices[j] >= i - 50) {
        continue;
      } else {
        pcl::PointCloud<pcl::PointXYZI> ds_cloud1 = *key_cloud_list[i];
        pcl::PointCloud<pcl::PointXYZI> ds_cloud2 = *key_cloud_list[indices[j]];
        // down_sampling_voxel(ds_cloud1, 0.5);
        // down_sampling_voxel(ds_cloud2, 0.5);
        double overlap =
            calc_overlap(ds_cloud1.makeShared(), ds_cloud2.makeShared(), 0.25);
        if (overlap > max_overlap) {
          max_overlap = overlap;
          loop_id = indices[j];
        }
      }
    }
    if (max_overlap > overlap_threshold) {
      trigger_loop = true;
      gt_loop_num++;
      sensor_msgs::PointCloud2 pub_cloud;
      pcl::toROSMsg(*key_cloud_list[i], pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCureentCloud.publish(pub_cloud);
      loop.sleep();
      pcl::toROSMsg(*key_cloud_list[loop_id], pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubMatchedCloud.publish(pub_cloud);
      loop.sleep();
      result_file << i << "," << searchPoint.x << "," << searchPoint.y << ","
                  << searchPoint.z << "," << 1 << "," << loop_id << ","
                  << max_overlap << std::endl;
      max_overlap = floor((max_overlap * pow(10, 3) + 0.5)) / pow(10, 3);
      std::cout << "loop trigger:" << i << "-" << loop_id
                << ", overlap:" << max_overlap << std::endl;
      std::string max_overlap_str = std::to_string(max_overlap);
      max_overlap_str =
          max_overlap_str.substr(0, max_overlap_str.find(".") + 4);
      max_overlap_str = "Overlap: " + max_overlap_str;
      // publish_map(pubLaserCloudMap);
      cv::Mat max_overlap_pic = cv::Mat::zeros(200, 800, CV_8UC3);
      cv::Point siteNO;
      siteNO.x = 100;
      siteNO.y = 100;
      cv::putText(max_overlap_pic, max_overlap_str, siteNO, 4, 2,
                  cv::Scalar(255, 255, 255), 4);
      cv::imshow("", max_overlap_pic);
      cv::waitKey(500);
      // getchar();
    } else {
      result_file << i << "," << searchPoint.x << "," << searchPoint.y << ","
                  << searchPoint.z << "," << 0 << "," << 0 << "," << 0
                  << std::endl;
    }
  }

  return 0;
}