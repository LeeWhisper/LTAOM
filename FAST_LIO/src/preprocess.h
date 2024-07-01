#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

/**
 * @brief 表示支持的雷达类型
 */
enum LID_TYPE
{
  AVIA = 1,
  VELO16,
  OUSTER,
  HESAI128,
  XGRIDS
}; //{1, 2, 3}

/**
 * @brief 特征点类型
 */
enum Feature
{
  Nor,        // 正常点
  Poss_Plane, // 可能的平面点
  Real_Plane, // 确定的平面点
  Edge_Jump,  // 有跨越的边
  Edge_Plane, // 边上的平面点
  Wire,       // 线段
  ZeroPoint   // 无效点
};
enum Surround
{
  Prev,
  Next
};
// 枚举类型：表示有跨越边的类型
enum E_jump
{
  Nr_nor,  // 正常
  Nr_zero, // 0
  Nr_180,  // 180
  Nr_inf,  // 无穷大 跳变较远？
  Nr_blind // 在盲区？
};

struct orgtype
{
  double range;     // 点云在xy平面离雷达中心的距离
  double dista;     // 当前点与后一个点之间的距离 //假设雷达原点为O 前一个点为M 当前点为A 后一个点为N
  double angle[2];  // 角OAM和角OAN的cos值
  double intersect; // 角MAN的cos值
  E_jump edj[2];    // 前后两点的类型
  Feature ftype;    // 点类型
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(uint16_t, ring, ring))

namespace ouster_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

namespace pandar128_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float timestamp;
  uint16_t  ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar128_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, timestamp, timestamp)
                                  (std::uint16_t, ring, ring)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;// 全部点、边缘点、平面点
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS;// 雷达类型、采样间隔、扫描线数
  double blind;
  bool feature_enabled, given_offset_time;// 是否提取特征、是否进行时间偏移
  ros::Publisher pub_full, pub_surf, pub_corn;// 发布全部点、发布平面点、发布边缘点
  double leaf_size = 0.01;

  private:
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void ouster_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void hesai128_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;

  ros::NodeHandle nh;
};
