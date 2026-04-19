#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

/** Create parent directories for a file path (mkdir -p). */
static bool ensureParentDirsForFile(const string& filepath) {
  for (size_t i = 1; i < filepath.size(); ++i) {
    if (filepath[i] == '/') {
      string sub = filepath.substr(0, i);
      if (mkdir(sub.c_str(), 0755) != 0 && errno != EEXIST) {
        ROS_WARN("mkdir %s failed: %s", sub.c_str(), strerror(errno));
        return false;
      }
    }
  }
  return true;
}

static string basenamePath(const string& p) {
  size_t s = p.find_last_of('/');
  return (s == string::npos) ? p : p.substr(s + 1);
}

/** Same format as legacy metrics filenames: YYYY-M-D-H-M-S */
static string makeWallClockTimeString() {
  time_t logTime = time(0);
  tm* ltm = localtime(&logTime);
  return to_string(1900 + ltm->tm_year) + "-" + to_string(1 + ltm->tm_mon) + "-" +
         to_string(ltm->tm_mday) + "-" + to_string(ltm->tm_hour) + "-" +
         to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);
}

string metricFile;
string trajFile;
string mapFile;
double overallMapVoxelSize = 0.5;
double exploredAreaVoxelSize = 0.3;
double exploredVolumeVoxelSize = 0.5;
double transInterval = 0.2;
double yawInterval = 10.0;
int overallMapDisplayInterval = 2;
int overallMapDisplayCount = 0;
int exploredAreaDisplayInterval = 1;
int exploredAreaDisplayCount = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudrefined(new pcl::PointCloud<pcl::PointXYZ>());

pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr exploredVolumeCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr exploredVolumeCloud2(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());

const int systemDelay = 5;
int systemDelayCount = 0;
bool systemDelayInited = false;
double systemTime = 0;
double systemInitTime = 0;
bool systemInited = false;

float vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float exploredVolume = 0, travelingDis = 0, runtime = 0, timeDuration = 0;
float maxBatteryDistance = 400.0f;
float batteryRemainingDistance = 400.0f;
float batteryRemainingPercent = 100.0f;
bool batteryDepleted = false;

// Optional multi-term battery model (equivalent meters per second / per m^3
// mapped into the same budget as maxBatteryDistance for compatibility).
bool battery_use_power_model = false;
double battery_charge_01 = 1.0;
double last_battery_integrator_time = -1.0;
float exploration_load_scale_cached = 1.f;
bool exploration_scale_have_msg = false;
double battery_motion_linear_weight = 1.0;
double battery_motion_yaw_rad_weight = 0.12;
double battery_explore_fixed_equivalent_mps = 0.025;
double battery_explore_volume_equivalent_per_m3 = 0.0025;
double battery_comm_base_equivalent_mps = 0.006;
double battery_comm_proximity_equivalent_mps = 0.0;
double battery_comm_ref_distance_m = 30.0;
bool battery_comm_use_pairwise = false;
double battery_base_idle_equivalent_mps = 0.012;
double battery_planning_load_equivalent_mps = 0.0;
float min_peer_distance_m = 1e6f;
int g_battery_robot_id = 0;
int g_battery_num_robots = 3;
float last_explored_volume_for_battery = 0.f;
bool g_battery_volume_seeded = false;

void explorationLoadScaleCb(const std_msgs::Float32ConstPtr& msg) {
  exploration_load_scale_cached =
      std::max(0.f, std::min(1.f, msg->data));
  exploration_scale_have_msg = true;
}

void pairwiseDistCb(const std_msgs::Float32MultiArrayConstPtr& msg) {
  const int n = g_battery_num_robots;
  const int need = n * (n - 1) / 2;
  if (static_cast<int>(msg->data.size()) < need) return;
  const int r = std::max(0, g_battery_robot_id - 1);
  if (n == 3 && r >= 0 && r < 3) {
    const float d01 = msg->data[0];
    const float d02 = msg->data[1];
    const float d12 = msg->data[2];
    if (r == 0)
      min_peer_distance_m = std::min(d01, d02);
    else if (r == 1)
      min_peer_distance_m = std::min(d01, d12);
    else
      min_peer_distance_m = std::min(d02, d12);
  }
}

void applyBatteryDrainFromOdom(double t_sec, float planar_dis_m,
                               float yaw_delta_rad) {
  if (!battery_use_power_model || !systemInited) return;
  if (last_battery_integrator_time < 0.0) {
    last_battery_integrator_time = t_sec;
    return;
  }
  double dt = t_sec - last_battery_integrator_time;
  last_battery_integrator_time = t_sec;
  if (dt <= 1e-9 || dt > 5.0) return;

  const double scale =
      exploration_scale_have_msg ? exploration_load_scale_cached : 1.0;

  const double motion_eq_m = battery_motion_linear_weight * planar_dis_m +
                             battery_motion_yaw_rad_weight * yaw_delta_rad;
  const double explore_eq_m =
      scale * battery_explore_fixed_equivalent_mps * dt;
  // Communication: default is constant self-TX (base only). Optional pairwise
  // term only if battery_comm_use_pairwise and proximity coefficient > 0.
  double comm_eq_m = battery_comm_base_equivalent_mps * dt;
  if (battery_comm_use_pairwise &&
      battery_comm_proximity_equivalent_mps > 1e-12 &&
      min_peer_distance_m < 1e5f && battery_comm_ref_distance_m > 1e-6) {
    const double nd = static_cast<double>(min_peer_distance_m);
    const double x =
        std::max(0.0, 1.0 - nd / battery_comm_ref_distance_m);
    comm_eq_m += battery_comm_proximity_equivalent_mps * x * dt;
  }
  const double idle_eq_m = battery_base_idle_equivalent_mps * dt;
  const double plan_eq_m = battery_planning_load_equivalent_mps * dt;

  const double total_eq_m =
      motion_eq_m + explore_eq_m + comm_eq_m + idle_eq_m + plan_eq_m;
  const double denom =
      std::max(static_cast<double>(1e-6), static_cast<double>(maxBatteryDistance));
  battery_charge_01 -= total_eq_m / denom;
  if (battery_charge_01 < 0.0) battery_charge_01 = 0.0;
}

void maybeApplyVolumeBatteryDrain(float new_volume_m3) {
  if (!battery_use_power_model || !systemInited) return;
  if (!g_battery_volume_seeded) {
    g_battery_volume_seeded = true;
    last_explored_volume_for_battery = new_volume_m3;
    return;
  }
  const float dV = std::max(0.f, new_volume_m3 - last_explored_volume_for_battery);
  last_explored_volume_for_battery = new_volume_m3;
  if (dV <= 0.f) return;
  const double scale =
      exploration_scale_have_msg ? exploration_load_scale_cached : 1.0;
  const double vol_eq_m =
      scale * battery_explore_volume_equivalent_per_m3 * dV;
  const double denom =
      std::max(static_cast<double>(1e-6), static_cast<double>(maxBatteryDistance));
  battery_charge_01 -= vol_eq_m / denom;
  if (battery_charge_01 < 0.0) battery_charge_01 = 0.0;
}

pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;
pcl::VoxelGrid<pcl::PointXYZ> exploredAreaDwzFilter;
pcl::VoxelGrid<pcl::PointXYZ> exploredVolumeDwzFilter;

sensor_msgs::PointCloud2 overallMap2;

ros::Publisher *pubExploredAreaPtr = NULL;
ros::Publisher *pubTrajectoryPtr = NULL;
ros::Publisher *pubExploredVolumePtr = NULL;
ros::Publisher *pubTravelingDisPtr = NULL;
ros::Publisher *pubTimeDurationPtr = NULL;
ros::Publisher *pubBatteryRemainingDistancePtr = NULL;
ros::Publisher *pubBatteryRemainingPercentPtr = NULL;
ros::Publisher *pubStopPtr = NULL;

FILE *metricFilePtr = NULL;
FILE *trajFilePtr = NULL;

void updateBatteryStatus()
{
  if (maxBatteryDistance <= 0.0f) {
    batteryRemainingDistance = 0.0f;
    batteryRemainingPercent = 0.0f;
    batteryDepleted = true;
    return;
  }

  if (battery_use_power_model) {
    battery_charge_01 = std::max(0.0, std::min(1.0, battery_charge_01));
    batteryRemainingPercent =
        static_cast<float>(100.0 * battery_charge_01);
    batteryRemainingDistance =
        static_cast<float>(maxBatteryDistance * battery_charge_01);
    batteryDepleted = (battery_charge_01 <= 0.0);
    return;
  }

  batteryRemainingDistance = std::max(0.0f, maxBatteryDistance - travelingDis);
  batteryRemainingPercent = std::max(0.0f, std::min(100.0f, 100.0f * batteryRemainingDistance / maxBatteryDistance));
  batteryDepleted = (batteryRemainingDistance <= 0.0f);
}

void writeMetricsLineToFile() {
  if (metricFilePtr == NULL) return;
  updateBatteryStatus();
  fprintf(metricFilePtr, "%f %f %f %f %f %f\n", exploredVolume, travelingDis,
          runtime, timeDuration, batteryRemainingPercent,
          batteryRemainingDistance);
}

void publishBatteryAndStopIfNeeded()
{
  if (pubBatteryRemainingDistancePtr != NULL) {
    std_msgs::Float32 batteryDistanceMsg;
    batteryDistanceMsg.data = batteryRemainingDistance;
    pubBatteryRemainingDistancePtr->publish(batteryDistanceMsg);
  }

  if (pubBatteryRemainingPercentPtr != NULL) {
    std_msgs::Float32 batteryPercentMsg;
    batteryPercentMsg.data = batteryRemainingPercent;
    pubBatteryRemainingPercentPtr->publish(batteryPercentMsg);
  }

  if (batteryDepleted && pubStopPtr != NULL) {
    std_msgs::Int8 stopMsg;
    stopMsg.data = 1;
    pubStopPtr->publish(stopMsg);
  }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  systemTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  float dYaw = fabs(yaw - vehicleYaw);
  if (dYaw > PI) dYaw = 2 * PI  - dYaw;

  float dx = odom->pose.pose.position.x - vehicleX;
  float dy = odom->pose.pose.position.y - vehicleY;
  float dz = odom->pose.pose.position.z - vehicleZ;
  float dis = sqrt(dx * dx + dy * dy + dz * dz);

  if (!systemDelayInited) {
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    return;
  }

  if (!systemInited) {
    systemInitTime = systemTime;
    systemInited = true;
    dis = 0.0f;
  } else {
    timeDuration = systemTime - systemInitTime;
    std_msgs::Float32 timeDurationMsg;
    timeDurationMsg.data = timeDuration;
    pubTimeDurationPtr->publish(timeDurationMsg);
  }

  // Always accumulate traveled distance for metrics / legacy battery.
  travelingDis += dis;
  const float planar_dis =
      static_cast<float>(std::sqrt(static_cast<double>(dx * dx + dy * dy)));
  applyBatteryDrainFromOdom(systemTime, planar_dis, dYaw);

  updateBatteryStatus();
  publishBatteryAndStopIfNeeded();

  // Downsample trajectory logging/visualization to avoid excessive points.
  if (dis < transInterval && dYaw < yawInterval) {
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    return;
  }

  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  fprintf(trajFilePtr, "%f %f %f %f %f %f %f\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, timeDuration);

  pcl::PointXYZI point;
  point.x = vehicleX;
  point.y = vehicleY;
  point.z = vehicleZ;
  point.intensity = travelingDis;
  trajectory->push_back(point);

  sensor_msgs::PointCloud2 trajectory2;
  pcl::toROSMsg(*trajectory, trajectory2);
  trajectory2.header.stamp = odom->header.stamp;
  trajectory2.header.frame_id = "map";
  pubTrajectoryPtr->publish(trajectory2);
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  if (!systemDelayInited) {
    systemDelayCount++;
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  if (!systemInited) {
    return;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  maybeApplyVolumeBatteryDrain(exploredVolume);

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "map";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  writeMetricsLineToFile();

  // runtime=0;
  std_msgs::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);
  
  std_msgs::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
  publishBatteryAndStopIfNeeded();
}



void laserCloudHandlerB1(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  if (!systemDelayInited) {
    systemDelayCount++;
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  if (!systemInited) {
    return;
  }

  laserCloud->clear();
  laserCloudrefined->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudrefined(new pcl::PointCloud<pcl::PointXYZ>());

  float offset_b1_x = 0.0;
  float offset_b1_y = -2.0;
  float offset_b1_z = 0.0;

  pcl::PointXYZ point;
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      // geometry_msgs::Point cloudpoint;
      // tf::Vector3 cloudpoint(point.x,point.y,point.z);
      // tf::Vector3 cloudpoint_tf_robot = cloud_to_robot_transform * cloudpoint;
      // tf::pointTFToMsg(carrot_point_tf_robot, carrot_waypoint_robot.position);

      point.x = point.x + offset_b1_x; // cloudpoint_tf_robot[0];
      point.y = point.y + offset_b1_y; // cloudpoint_tf_robot[1];
      point.z = point.z + offset_b1_z; //cloudpoint_tf_robot[2];

      
      laserCloudrefined->push_back(point);
    
    }
  laserCloud->clear();

  for (int i = 0; i < laserCloudrefined->points.size(); i++) {

    laserCloudrefined->push_back(laserCloudrefined->points[i]);  
  }

  // laserCloud->setInputCloud(laserCloudrefined);

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  maybeApplyVolumeBatteryDrain(exploredVolume);

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "map";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  writeMetricsLineToFile();

  // runtime=0;
  std_msgs::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);
  
  std_msgs::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
  publishBatteryAndStopIfNeeded();
}


void laserCloudHandlerB2(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  if (!systemDelayInited) {
    systemDelayCount++;
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  if (!systemInited) {
    return;
  }

  laserCloud->clear();
  laserCloudrefined->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);


  float offset_b2_x = 0.0;
  float offset_b2_y = -4.0;
  float offset_b2_z = 0.0;

  pcl::PointXYZ point;
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      // geometry_msgs::Point cloudpoint;
      // tf::Vector3 cloudpoint(point.x,point.y,point.z);
      // tf::Vector3 cloudpoint_tf_robot = cloud_to_robot_transform * cloudpoint;
      // tf::pointTFToMsg(carrot_point_tf_robot, carrot_waypoint_robot.position);

      point.x = point.x + offset_b2_x; // cloudpoint_tf_robot[0];
      point.y = point.y + offset_b2_y; // cloudpoint_tf_robot[1];
      point.z = point.z + offset_b2_z; //cloudpoint_tf_robot[2];

      
      laserCloudrefined->push_back(point);
    
    }
  laserCloud->clear();

  for (int i = 0; i < laserCloudrefined->points.size(); i++) {

    laserCloudrefined->push_back(laserCloudrefined->points[i]);  
  }

  // laserCloud->setInputCloud(laserCloudrefined);

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  maybeApplyVolumeBatteryDrain(exploredVolume);

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "map";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  writeMetricsLineToFile();

  // runtime=0;
  std_msgs::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);
  
  std_msgs::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
  publishBatteryAndStopIfNeeded();
}


void runtimeHandler(const std_msgs::Float32::ConstPtr& runtimeIn)
{
  runtime = runtimeIn->data;
}

void gbplannerruntimeHandler(const std_msgs::Float32MultiArray::ConstPtr& runtimeIn)
{
  runtime=0;
  for(int i =0; i < runtimeIn->data.size(); ++i){
    runtime += runtimeIn->data[i];
  }
  // runtime = runtimeIn->data[0]+runtimeIn->data[1]+runtimeIn->data[2]+runtimeIn->data[3];
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualizationTools");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  std::string metricFileIn;
  std::string trajFileIn;
  nhPrivate.getParam("metricFile", metricFileIn);
  nhPrivate.getParam("trajFile", trajFileIn);

  int robot_id_param = 0;
  nhPrivate.param("robotID", robot_id_param, 0);

  bool metrics_use_session_subdir = true;
  nhPrivate.param("metrics_use_session_subdir", metrics_use_session_subdir, true);

  std::string metrics_run_subdir;
  nhPrivate.param<std::string>("metrics_run_subdir", metrics_run_subdir, "");

  if (metrics_use_session_subdir) {
    std::string pkg_path;
    try {
      pkg_path = ros::package::getPath("visualization_tools");
    } catch (std::runtime_error& e) {
      ROS_WARN("ros::package::getPath(visualization_tools) failed: %s", e.what());
    }

    if (!pkg_path.empty()) {
      std::string session_stamp;
      if (!metrics_run_subdir.empty()) {
        session_stamp = metrics_run_subdir;
      } else {
        // One folder per launch: R1 publishes wall-clock stamp; R2/R3 wait for it.
        const std::string kSessionParam = "/mgg_metrics_session_stamp";
        if (robot_id_param == 1) {
          session_stamp = makeWallClockTimeString();
          ros::param::set(kSessionParam, session_stamp);
          ROS_INFO("[visualizationTools] session folder %s (R1 sets %s)", session_stamp.c_str(),
                   kSessionParam.c_str());
        } else {
          ros::Rate wait_rate(20);
          for (int i = 0; i < 200 && ros::ok(); ++i) {
            if (ros::param::get(kSessionParam, session_stamp) && !session_stamp.empty()) {
              break;
            }
            wait_rate.sleep();
          }
          if (session_stamp.empty()) {
            session_stamp = makeWallClockTimeString();
            ROS_WARN(
                "[visualizationTools] robotID=%d: %s not set in time, using local stamp %s",
                robot_id_param, kSessionParam.c_str(), session_stamp.c_str());
          } else {
            ROS_INFO("[visualizationTools] robotID=%d using session_stamp=%s", robot_id_param,
                     session_stamp.c_str());
          }
        }
      }

      const std::string base_m = basenamePath(metricFileIn);
      const std::string base_t = basenamePath(trajFileIn);
      metricFile = pkg_path + "/log/" + session_stamp + "/" + base_m + "_" + session_stamp + ".txt";
      trajFile = pkg_path + "/log/" + session_stamp + "/" + base_t + "_" + session_stamp + ".txt";
      ROS_INFO("[visualizationTools] metricFile=%s", metricFile.c_str());
    } else {
      ROS_WARN("metrics_use_session_subdir but package path failed; fallback to flat log/");
      metricFile = metricFileIn;
      trajFile = trajFileIn;
      const std::string ts = makeWallClockTimeString();
      metricFile += "_" + ts + ".txt";
      trajFile += "_" + ts + ".txt";
    }
  } else {
    metricFile = metricFileIn;
    trajFile = trajFileIn;
    const std::string ts = makeWallClockTimeString();
    metricFile += "_" + ts + ".txt";
    trajFile += "_" + ts + ".txt";
  }

  nhPrivate.getParam("mapFile", mapFile);
  nhPrivate.getParam("overallMapVoxelSize", overallMapVoxelSize);
  nhPrivate.getParam("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nhPrivate.getParam("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nhPrivate.getParam("transInterval", transInterval);
  nhPrivate.getParam("yawInterval", yawInterval);
  nhPrivate.getParam("overallMapDisplayInterval", overallMapDisplayInterval);
  nhPrivate.getParam("exploredAreaDisplayInterval", exploredAreaDisplayInterval);
  nhPrivate.param("max_battery_distance", maxBatteryDistance, 400.0f);

  g_battery_robot_id = robot_id_param;
  nhPrivate.param("battery_num_robots", g_battery_num_robots, 3);
  nhPrivate.param("battery_use_power_model", battery_use_power_model, false);
  nhPrivate.param("battery_motion_linear_weight", battery_motion_linear_weight,
                   1.0);
  nhPrivate.param("battery_motion_yaw_rad_weight", battery_motion_yaw_rad_weight,
                   0.12);
  nhPrivate.param("battery_explore_fixed_equivalent_mps",
                   battery_explore_fixed_equivalent_mps, 0.025);
  nhPrivate.param("battery_explore_volume_equivalent_per_m3",
                   battery_explore_volume_equivalent_per_m3, 0.0025);
  nhPrivate.param("battery_comm_base_equivalent_mps",
                   battery_comm_base_equivalent_mps, 0.006);
  nhPrivate.param("battery_comm_proximity_equivalent_mps",
                   battery_comm_proximity_equivalent_mps, 0.0);
  nhPrivate.param("battery_comm_ref_distance_m", battery_comm_ref_distance_m,
                   30.0);
  nhPrivate.param("battery_comm_use_pairwise", battery_comm_use_pairwise, false);
  nhPrivate.param("battery_base_idle_equivalent_mps",
                   battery_base_idle_equivalent_mps, 0.012);
  nhPrivate.param("battery_planning_load_equivalent_mps",
                   battery_planning_load_equivalent_mps, 0.0);
  if (battery_use_power_model) {
    battery_charge_01 = 1.0;
    last_battery_integrator_time = -1.0;
    g_battery_volume_seeded = false;
    ROS_INFO(
        "[visualizationTools] battery_use_power_model=ON max_equiv_m=%.1f "
        "robotID=%d",
        static_cast<double>(maxBatteryDistance), g_battery_robot_id);
  }

  nhPrivate.getParam("vehicleX", vehicleX);
  nhPrivate.getParam("vehicleY", vehicleY);
  nhPrivate.getParam("vehicleZ", vehicleZ);
  updateBatteryStatus();

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> ("state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> ("registered_scan", 5, laserCloudHandler);


  ros::Subscriber subLaserCloudB1 = nh.subscribe<sensor_msgs::PointCloud2> ("b1_scan", 5, laserCloudHandlerB1);

  ros::Subscriber subLaserCloudB2 = nh.subscribe<sensor_msgs::PointCloud2> ("b2_scan", 5, laserCloudHandlerB2);


  ros::Subscriber subRuntime = nh.subscribe<std_msgs::Float32> ("runtime", 5, runtimeHandler);

  ros::Subscriber subgbRuntime = nh.subscribe<std_msgs::Float32MultiArray> ("gbp_time_log", 5, gbplannerruntimeHandler);

  ros::Subscriber subExplorationScale = nh.subscribe<std_msgs::Float32>(
      "exploration_load_scale", 10, explorationLoadScaleCb);
  ros::Subscriber subPairwise;
  if (battery_comm_use_pairwise) {
    std::string battery_pairwise_topic;
    nhPrivate.param("battery_pairwise_topic", battery_pairwise_topic,
                    std::string("/inter_robot_pairwise_distances"));
    subPairwise = nh.subscribe<std_msgs::Float32MultiArray>(
        battery_pairwise_topic, 5, pairwiseDistCb);
  }

  ros::Publisher pubOverallMap = nh.advertise<sensor_msgs::PointCloud2> ("overall_map", 5);

  ros::Publisher pubExploredArea = nh.advertise<sensor_msgs::PointCloud2> ("explored_areas", 5);
  pubExploredAreaPtr = &pubExploredArea;

  ros::Publisher pubTrajectory = nh.advertise<sensor_msgs::PointCloud2> ("trajectory", 5);
  pubTrajectoryPtr = &pubTrajectory;

  ros::Publisher pubExploredVolume = nh.advertise<std_msgs::Float32> ("explored_volume", 5);
  pubExploredVolumePtr = &pubExploredVolume;

  ros::Publisher pubTravelingDis = nh.advertise<std_msgs::Float32> ("traveling_distance", 5);
  pubTravelingDisPtr = &pubTravelingDis;

  ros::Publisher pubTimeDuration = nh.advertise<std_msgs::Float32> ("time_duration", 5);
  pubTimeDurationPtr = &pubTimeDuration;

  ros::Publisher pubstop = nh.advertise<std_msgs::Int8> ("stop", 5);
  pubStopPtr = &pubstop;
  
  ros::Publisher pubBatteryRemainingDistance = nh.advertise<std_msgs::Float32> ("battery_remaining_distance", 5);
  pubBatteryRemainingDistancePtr = &pubBatteryRemainingDistance;
  
  ros::Publisher pubBatteryRemainingPercent = nh.advertise<std_msgs::Float32> ("battery_remaining_percent", 5);
  pubBatteryRemainingPercentPtr = &pubBatteryRemainingPercent;
  

  //ros::Publisher pubRuntime = nh.advertise<std_msgs::Float32> ("/runtime", 5);

  overallMapDwzFilter.setLeafSize(overallMapVoxelSize, overallMapVoxelSize, overallMapVoxelSize);
  exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
  exploredVolumeDwzFilter.setLeafSize(exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);

  pcl::PLYReader ply_reader;
  if (ply_reader.read(mapFile, *overallMapCloud) == -1) {
    printf("\nCouldn't read pointcloud.ply file.\n\n");
  }

  overallMapCloudDwz->clear();
  overallMapDwzFilter.setInputCloud(overallMapCloud);
  overallMapDwzFilter.filter(*overallMapCloudDwz);
  overallMapCloud->clear();

  pcl::toROSMsg(*overallMapCloudDwz, overallMap2);

  ensureParentDirsForFile(metricFile);
  ensureParentDirsForFile(trajFile);
  metricFilePtr = fopen(metricFile.c_str(), "w");
  trajFilePtr = fopen(trajFile.c_str(), "w");
  if (metricFilePtr) {
    bool metrics_write_header = true;
    nhPrivate.param("metrics_write_header", metrics_write_header, true);
    if (metrics_write_header) {
      fprintf(metricFilePtr,
              "# explored_volume_m3 traveling_distance_m runtime_s time_duration_s "
              "battery_remaining_percent battery_remaining_distance_m\n");
      fflush(metricFilePtr);
    }
  } else {
    ROS_ERROR("Could not open metricFile for write: %s", metricFile.c_str());
  }
  if (!trajFilePtr) {
    ROS_ERROR("Could not open trajFile for write: %s", trajFile.c_str());
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    publishBatteryAndStopIfNeeded();

    overallMapDisplayCount++;
    if (overallMapDisplayCount >= 100 * overallMapDisplayInterval) {
      overallMap2.header.stamp = ros::Time().fromSec(systemTime);
      overallMap2.header.frame_id = "map";
      pubOverallMap.publish(overallMap2);

      overallMapDisplayCount = 0;
    }

    // if(timeDuration > 1800){
    //   std_msgs::Int8 stop_msg;
    //   stop_msg.data = 1;
    //   pubstop.publish(stop_msg);
    // }

    status = ros::ok();
    rate.sleep();
  }

  fclose(metricFilePtr);
  fclose(trajFilePtr);

  printf("\nExploration metrics and vehicle trajectory are saved in 'src/vehicle_simulator/log'.\n\n");

  return 0;
}
