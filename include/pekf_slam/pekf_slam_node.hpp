#ifndef PEKF_SLAM_NODE_HPP_
#define PEKF_SLAM_NODE_HPP_

// ros stuff
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "pekf_slam.hpp"

// messages 
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/thread/mutex.hpp>

// log files
// #include <fstream>



namespace pekfslam
{


typedef boost::shared_ptr<const nav_msgs::msg::Odometry> OdomConstPtr;
typedef boost::shared_ptr<const sensor_msgs::msg::Imu> ImuConstPtr;
// typedef boost::shared_ptr<sensor_msgs::PointCloud2 const> PointCloud2Ptr;
// typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::PointCloud<pcl::PointXYZ>::Ptr;
class PEKFSLAMNode : public rclcpp::Node
{
public:
  /**
    * @brief A constructor for
    * @param options Additional options to control creation of the node.
    */
  explicit PEKFSLAMNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
    * @brief A destructor for 
    */
  virtual ~PEKFSLAMNode();

private:
  /// the main localization loop that will be called periodically
  void start();

  /// callback function for odo data (Odometry)
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  /*
  * callback function for imu data to be implemented
  */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu);


  /*
    * @brief callback function for laser point cloud
    */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud); 


  // rclcpp::NodeHandle node_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr predicted_pose_pub_, updated_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poses_pub_;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  // rclcpp::Service<>::SharedPtr state_srv_;

  // ekf filter
  PEKFSLAM filter_;

  // estimated robot pose message to send
  geometry_msgs::msg::PoseWithCovarianceStamped  output_; 

  // robot state
  // std::unique_ptr<tf2_ros::Buffer> robot_state_;
  // tf2_ros::TransformBroadcaster odom_broadcaster_;
  
  rclcpp::Time odom_stamp_, imu_stamp_, vo_stamp_, filter_stamp_;

  bool new_scan_, is_pose_start, new_odom_;
  bool odom_initializing_, imu_initializing_, vo_initializing_;
  double imu_yaw;
  double timeout_;
  double time_between_cloud_points_, _noise, prev_time_stamp_, dist_threshold_, prev_dist;

  std::string output_frame_, base_link_frame_, tf_prefix_, odom_topic_, imu_topic_, predict_pose_topic_, update_pose_topic_, poses_topic_, pc_topic_;




  // counters
  unsigned int odom_callback_counter_, imu_callback_counter_, vo_callback_counter_, ekf_sent_counter_;
  
  double _transformation_epsilon; //minimum transformation difference for ICP termination condition
  int _max_iters, _ransac_iterations; //max number of registration iterations
  double _euclidean_fitness_epsilon; //maximum allowed Euclidean error between two consecutive steps in the ICP loop
  double _max_correspondence_distance; //correspondences with higher distances will be ignored
  double _ransac_threshold; //maximum number of RANSAC iterations and RANSAC inlier threshold
  Eigen::VectorXd odom_meas,odom_prev_pose, new_meas, change_;
  Eigen::VectorXd pred_X;
  Eigen::MatrixXd pred_P;
  Eigen::MatrixXd odom_cov;
  Eigen::Vector3d pose;
  Eigen::Matrix3d cov;
  Eigen::VectorXd poses, dead_rec;
  Eigen::MatrixXd covs;
  std::vector<Eigen::Matrix4f> z_vec;
  std::vector<double> z_cov_vec;
  Eigen::Matrix4f robot_pose;
  Eigen::VectorXf robot_pose_vector;
  std::vector<int> Hp;
  

}; // class

}; // namespace

#endif  // PEKF_SLAM_NODE__PEKF_SLAM_NODE_NODE_HPP_