#include <pcl_conversions/pcl_conversions.h>

#include <deque>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LidType { AVIA = 1, VELO16, OUST64, HESA_IXT32 };  //{1, 2, 3, 4}
enum TimeUnit { SEC = 0, MS = 1, US = 2, NS = 3 };
enum Feature { NOR, POSS_PLANE, REAL_PLANE, EDGE_JUMP, EDGE_PLANE, WIRE, ZERO_POINT };
enum Surround { PREV, NEXT };
enum EJump { NR_NOR, NR_ZERO, NR_180, NR_INF, NR_BLIND };

const bool time_list_cut_frame(PointType & x, PointType & y);

struct Orgtype
{
  double range;
  double dista;
  double angle[2];
  double intersect;
  EJump edj[2];
  Feature ftype;
  Orgtype()
  {
    range = 0;
    edj[PREV] = NR_NOR;
    edj[NEXT] = NR_NOR;
    ftype = NOR;
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
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
  velodyne_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                         float, time, time)(std::uint16_t, ring, ring))

namespace hesai_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace hesai_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
  hesai_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                      double, timestamp, timestamp)(std::uint16_t, ring, ring))

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
}  // namespace ouster_ros

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

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void processCutFrameLivox(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, std::deque<PointCloudXYZI::Ptr> &pcl_out, std::deque<double> &time_lidar, const int required_frame_num, int scan_count);
  
  void processCutFramePcl2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, std::deque<PointCloudXYZI::Ptr> &pcl_out, std::deque<double> &time_lidar, const int required_frame_num, int scan_count);
 
  void process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::msg::PointCloud2::SharedPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  std::vector<Orgtype> typess[128]; //maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;
  bool given_offset_time;

  private:
  void aviaHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);
  void oust64Handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  void velodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  void hesaiHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  void giveFeature(PointCloudXYZI &pl, std::vector<Orgtype> &types);
  void pubFunc(PointCloudXYZI &pl, const rclcpp::Time &ct);
  int  planeJudge(const PointCloudXYZI &pl, std::vector<Orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool smallPlane(const PointCloudXYZI &pl, std::vector<Orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edgeJumpJudge(const PointCloudXYZI &pl, std::vector<Orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
