#include "pekf_slam/pekf_slam_node.hpp"

using namespace std;
using namespace pekfslam;


static const double EPS = 1e-5;

namespace pekfslam
{
  // constructor
  PEKFSLAMNode::PEKFSLAMNode(const rclcpp::NodeOptions& options)
  : Node("pekf_slam_node",options)
  {

    // paramters
    declare_parameter("output_frame", "odom");
    output_frame_ = get_parameter("output_frame").as_string();
    declare_parameter("base_link_frame", "base_link");
    base_link_frame_ = get_parameter("base_link_frame").as_string();
    declare_parameter("odom_topic", "odom");
    odom_topic_ = get_parameter("odom_topic").as_string();
    declare_parameter("imu_topic", "imu");
    imu_topic_ = get_parameter("imu_topic").as_string();
    declare_parameter("pc_topic", "point_cloud");
    pc_topic_ = get_parameter("pc_topic").as_string();
    declare_parameter("predict_pose_topic", "predict_pose");
    predict_pose_topic_ = get_parameter("predict_pose_topic").as_string();
    declare_parameter("update_pose_topic", "update_pose");
    update_pose_topic_ = get_parameter("update_pose_topic").as_string();
    declare_parameter("poses_topic", "poses");
    poses_topic_ = get_parameter("poses_topic").as_string();
    declare_parameter("time_between_cloud_points", 1.0);
    time_between_cloud_points_ = get_parameter("time_between_cloud_points").as_double();
    declare_parameter("noise", 0.0);
    _noise = get_parameter("noise").as_double();
    declare_parameter("dist_threshold", 1.0);
    dist_threshold_ = get_parameter("dist_threshold").as_double();

    // ICP params from yaml file
    declare_parameter("transformation_epsilon", 0.01);
    _transformation_epsilon = get_parameter("transformation_epsilon").as_double();
    declare_parameter("max_iterations", 75);
    _max_iters = get_parameter("max_iterations").as_int();
    declare_parameter("euclidean_fitness_epsilon", 0.01);
    _euclidean_fitness_epsilon = get_parameter("euclidean_fitness_epsilon").as_double();
    declare_parameter("max_correspondence_distance", 1.0);
    _max_correspondence_distance = get_parameter("max_correspondence_distance").as_double();
    declare_parameter("ransac_iterations", 10);
    _ransac_iterations = get_parameter("ransac_iterations").as_int();
    declare_parameter("ransac_threshold", 1.0);
    _ransac_threshold = get_parameter("ransac_threshold").as_double();
    declare_parameter("freq", 30.0);
    double freq = get_parameter("freq").as_double();


    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_, 10, std::bind(&PEKFSLAMNode::odomCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic_, 10, std::bind(&PEKFSLAMNode::imuCallback, this, std::placeholders::_1));
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                pc_topic_, 10, std::bind(&PEKFSLAMNode::cloudCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0/std::max(freq, 1.0)),
        std::bind(&PEKFSLAMNode::start, this));

    predicted_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(predict_pose_topic_, 10);
    updated_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(update_pose_topic_, 10);
    poses_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(poses_topic_, 10);


    prev_time_stamp_ = 0;

    // subscribe to odom messages
    odom_meas = Eigen::VectorXd::Zero(3,1);
    new_meas = Eigen::VectorXd::Zero(3,1);
    odom_prev_pose = Eigen::VectorXd::Zero(3,1);
    change_ = Eigen::VectorXd::Zero(3,1);
    odom_cov = Eigen::MatrixXd::Zero(3,3);

    new_scan_ = false;
    is_pose_start = false;
    new_odom_ = false;
    z_vec.reserve(1000);
    z_cov_vec.reserve(1000);
    Hp.reserve(1000);
    prev_dist = 0.0;
    dead_rec.setZero(3000,1);
    pred_X.setZero(3,1);
    pred_P.setZero(3,3);
    filter_.setICPParams(_max_correspondence_distance, _transformation_epsilon, _max_iters, _euclidean_fitness_epsilon, _ransac_iterations, _ransac_threshold);

  };




  // // destructor
  PEKFSLAMNode::~PEKFSLAMNode(){

  };

  void PEKFSLAMNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // get roll pitch and yaw from quaternion
    tf2::Quaternion q(msg->orientation.x, 
                      msg->orientation.y, 
                      msg->orientation.z, 
                      msg->orientation.w); // quaternion
    
    tf2::Matrix3x3 rot_1(q); // rotation matrix

    double roll, pitch;
    rot_1.getRPY(roll, pitch, imu_yaw); //get the RPY angles from the quaternion
  }




  void PEKFSLAMNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  { 
    RCLCPP_INFO(this->get_logger(), "odomCallback");
    // get roll pitch and yaw from quaternion
    tf2::Quaternion q(msg->pose.pose.orientation.x, 
                      msg->pose.pose.orientation.y, 
                      msg->pose.pose.orientation.z, 
                      msg->pose.pose.orientation.w); // quaternion
    
    tf2::Matrix3x3 rot_1(q); // rotation matrix

    double roll, pitch;
    rot_1.getRPY(roll, pitch, imu_yaw);

    new_meas << msg->pose.pose.position.x, msg->pose.pose.position.y, imu_yaw; // set odom measurement
    odom_cov.diagonal() << 0.5*msg->pose.covariance[0], 0.5*msg->pose.covariance[7],  0.5*msg->pose.covariance[35]; // set odom covariance
    if (!new_odom_){
      odom_meas = new_meas - odom_prev_pose + (_noise*Eigen::VectorXd::Random(3, 1));

      odom_meas(2) = filter_.wrapAngle(odom_meas(2));
      odom_prev_pose = new_meas;
      new_odom_ = true;
    }

    is_pose_start = true;
  };



  /*
  * @brief add new scans to map vector
  */
  void PEKFSLAMNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // add point cloud to the cloud_vector
    if (!filter_.isInitialized() || (msg->header.stamp.sec - prev_time_stamp_ > time_between_cloud_points_
      && !new_scan_ && change_.norm() > dist_threshold_)) { //only add cloud points if the time between them is greater than the time_between_cloud_points threshold
      prev_time_stamp_ = msg->header.stamp.sec;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud);
      RCLCPP_INFO(this->get_logger(), "new scan at %f %f %f", new_meas(0), new_meas(1), new_meas(2));
      filter_.addScans(cloud);
      new_scan_ = true;
    }
  }




  // filter loop
  void PEKFSLAMNode::start()
  { 
    tf2::Quaternion q;
    change_+=odom_meas; 

    if (new_scan_ && filter_.isInitialized()){
      new_scan_ = false;
      int size = filter_.getScansVectorSize();

      if (size>=2 && change_.norm()>dist_threshold_){
        RCLCPP_INFO(this->get_logger(), "Update");
        new_odom_ = false;
        change_.setZero(); //new pose added reset counter
        filter_.addNewPose(odom_meas, odom_cov);

        Hp.clear();
        z_vec.clear();
        z_cov_vec.clear();
        filter_.overLappingScans(new_meas, z_vec, z_cov_vec, Hp);
        RCLCPP_INFO(this->get_logger(), "HP");
        Eigen::VectorXd y; Eigen::MatrixXd R; Eigen::MatrixXd H;
        if (Hp.size()>0){
          filter_.observationMatrix(z_vec, z_cov_vec, Hp, y, R, H);
          filter_.update(y, R, H, Hp);
          filter_.getPoses(poses, covs);
          RCLCPP_INFO(this->get_logger(), "poses %ld, %ld", poses.rows(), poses.cols());
          RCLCPP_INFO(this->get_logger(), "covs %ld, %ld", covs.rows(), covs.cols());
          nav_msgs::msg::Odometry poses_;

          for (int i=0; i<poses.rows(); i+=3){
            RCLCPP_INFO(this->get_logger(), "poses %d", i);
            q.setRPY(0, 0, poses(i+2, 0));
            poses_.header.stamp = this->now();
            poses_.header.frame_id = output_frame_;
            // poses_.header.seq = 0;//i;
            poses_.pose.pose.position.x = poses(i);
            poses_.pose.pose.position.y = poses(i+1);
            poses_.pose.pose.position.z = 0;
            poses_.pose.pose.orientation.x = q.x();
            poses_.pose.pose.orientation.y = q.y();
            poses_.pose.pose.orientation.z = q.z();
            poses_.pose.pose.orientation.w = q.w();
            poses_.pose.covariance[0] = covs.coeff(i,i);
            poses_.pose.covariance[7] = covs.coeff(i+1,i+1);
            poses_.pose.covariance[35] = covs.coeff(i+2,i+2);
            updated_pose_pub_->publish(poses_);
          }
         
        }
        RCLCPP_INFO(this->get_logger(), "poses %ld, %ld", poses.rows(), poses.cols());
        filter_.getPose(pose, cov);

        geometry_msgs::msg::PoseWithCovarianceStamped cur_pose;
        q.setRPY(0, 0, pose(2));
        cur_pose.header.stamp = this->now();
        cur_pose.header.frame_id = output_frame_;
        cur_pose.pose.pose.position.x = pose(0);
        cur_pose.pose.pose.position.y = pose(1);
        cur_pose.pose.pose.position.z = 0;
        cur_pose.pose.pose.orientation.x = q.x();
        cur_pose.pose.pose.orientation.y = q.y();
        cur_pose.pose.pose.orientation.z = q.z();
        cur_pose.pose.pose.orientation.w = q.w();
        cur_pose.pose.covariance[0] = cov(0,0);
        cur_pose.pose.covariance[7] = cov(1,1);
        cur_pose.pose.covariance[35] = cov(2,2);
        poses_pub_->publish(cur_pose);

      }
    } else if(new_odom_ && new_scan_ && !filter_.isInitialized()) { 
      RCLCPP_INFO(this->get_logger(), "Initialize"); 
      filter_.initialize(odom_meas);
      new_odom_ = false;
      new_scan_ = false; //scann added reser for new scan
      change_.setZero();

    }else if(new_odom_ && filter_.isInitialized()) { 
      RCLCPP_INFO(this->get_logger(), "Predict");
      filter_.predict(odom_meas, odom_cov,  pred_X, pred_P);
      new_odom_ = false;

    };

    // RCLCPP_INFO(this->get_logger(), "change_ %f, %d", change_.norm(), new_scan_);


    if(filter_.isInitialized()){
      // // RCLCPP_INFO(this->get_logger(), "Publishing");
      nav_msgs::msg::Odometry pose_msg;
      q.setRPY(0, 0, pred_X(2));
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = output_frame_;
      pose_msg.pose.pose.position.x = pred_X(0);
      pose_msg.pose.pose.position.y = pred_X(1);
      pose_msg.pose.pose.position.z = 0;
      pose_msg.pose.pose.orientation.x = q.x();
      pose_msg.pose.pose.orientation.y = q.y();
      pose_msg.pose.pose.orientation.z = q.z();
      pose_msg.pose.pose.orientation.w = q.w();
      pose_msg.pose.covariance[0] = pred_P(0,0);
      pose_msg.pose.covariance[7] = pred_P(1,1);
      pose_msg.pose.covariance[35] = pred_P(2,2);
      predicted_pose_pub_->publish(pose_msg);

      

    }

  };



}; // namespace



int main(int argc, char **argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // create filter class
  auto my_filter_node = std::make_shared<PEKFSLAMNode>();

  rclcpp::spin(my_filter_node);

  rclcpp::shutdown();

  return 0;
}
