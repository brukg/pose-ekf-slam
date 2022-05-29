#ifndef PEKFSLAMNode_H
#define PEKFSLAMNode_H_H

// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "pekf_slam.h"

// messages 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/thread/mutex.hpp>

// log files
// #include <fstream>



namespace pekfslam
{


typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef boost::shared_ptr<sensor_msgs::Imu const> ImuConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> VoConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> GpsConstPtr;
typedef boost::shared_ptr<geometry_msgs::Twist const> VelConstPtr;
// typedef boost::shared_ptr<sensor_msgs::PointCloud2 const> PointCloud2Ptr;
// typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pclXYZPtr;
class PEKFSLAMNode
{
public:
  /// constructor
  PEKFSLAMNode();

  /// destructor
  virtual ~PEKFSLAMNode();

private:
  /// the main localization loop that will be called periodically
  void start(const ros::TimerEvent& e);

  /// callback function for odo data (Odometry)
  void odomCallback(const OdomConstPtr& msg);

  /*
  * callback function for imu data to be implemented
  */
  // void imuCallback(const ImuConstPtr& imu);

  /*
    * @brief callback function for laser point cloud
    */
  void cloudCallback(const PointCloud2Ptr& msg); //point cloud callback


  ros::NodeHandle node_;
  ros::Timer timer_;
  ros::Publisher predicted_pose_pub_, updated_pose_pub_, pc_pub_, poses_pub_;
  ros::Subscriber odom_sub_, imu_sub_, vo_sub_, pc_sub_;
  ros::ServiceServer state_srv_;

  // ekf filter
  PEKFSLAM my_filter_;

  // estimated robot pose message to send
  geometry_msgs::PoseWithCovarianceStamped  output_; 

  // robot state
  tf::TransformListener    robot_state_;
  tf::TransformBroadcaster odom_broadcaster_;
  
  ros::Time odom_stamp_, imu_stamp_, vo_stamp_, filter_stamp_;

  bool new_scan_, is_pose_start, new_odom_;
  bool odom_initializing_, imu_initializing_, vo_initializing_;
  double timeout_;
  double _time_between_cloud_points, _noise, _prev_time_stamp, dist_threshold_, prev_dist;

  std::string output_frame_, base_footprint_frame_, tf_prefix_, odom_topic_, imu_topic_, predict_pose_topic_, update_pose_topic_, poses_topic_, pc_topic_;




  // counters
  unsigned int odom_callback_counter_, imu_callback_counter_, vo_callback_counter_, ekf_sent_counter_;
  
  double _transformation_epsilon; //minimum transformation difference for ICP termination condition
  int _max_iters; //max number of registration iterations
  double _euclidean_fitness_epsilon; //maximum allowed Euclidean error between two consecutive steps in the ICP loop
  double _max_correspondence_distance; //correspondences with higher distances will be ignored
  double _ransac_iterations, _ransac_threshold; //maximum number of RANSAC iterations and RANSAC inlier threshold
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

#endif
