#include <pekf_slam/pekf_slam_node.h>


// using namespace MatrixWrapper;
using namespace std;
using namespace ros;
using namespace tf;
using namespace pekfslam;


static const double EPS = 1e-5;

namespace pekfslam
{
  // constructor
  PEKFSLAMNode::PEKFSLAMNode()
  {
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    // paramters
    private_nh.param("output_frame", output_frame_, std::string("odom"));
    private_nh.param("base_footprint_frame", base_footprint_frame_, std::string("base_footprint"));
    private_nh.param("sensor_timeout", timeout_, 1.0);
    // private_nh.param("odom_used", odom_used_, true);
    // private_nh.param("imu_used",  imu_used_, true);
    // private_nh.param("vo_used",   vo_used_, true);
    private_nh.param("odom_topic", odom_topic_, std::string("/odom"));
    private_nh.param("imu_topic",  imu_topic_, std::string("/imu"));
    private_nh.param("pc_topic",   pc_topic_, std::string("/camera/depth/points"));
    private_nh.param("pose_topic",   pose_topic_, std::string("/pose_slam/pose"));
    private_nh.param("poses_topic",   poses_topic_, std::string("/pose_slam/poses"));
    private_nh.param("time_between_cloud_points",   _time_between_cloud_points, 1.0);
    private_nh.param("dist_threshold",   dist_threshold_, 1.0);


    // ICP params from yaml file
    private_nh.param("time_between_cloud_points", _time_between_cloud_points, 0.1);
    private_nh.param("transformation_epsilon", _transformation_epsilon, 0.01);
    private_nh.param("max_iterations", _max_iters, 75);
    private_nh.param("euclidean_fitness_epsilon", _euclidean_fitness_epsilon, 0.01);
    private_nh.param("max_correspondence_distance", _max_correspondence_distance, 1.0);
    double freq;
    private_nh.param("freq", freq, 30.0);

    // tf_prefix_ = tf::getPrefixParam(private_nh);
    // output_frame_ = tf::resolve(tf_prefix_, output_frame_);
    // base_footprint_frame_ = tf::resolve(tf_prefix_, base_footprint_frame_);


    // my_filter_.setOutputFrame(output_frame_);
    // my_filter_.setBaseFootprintFrame(base_footprint_frame_);

    odom_sub_ = nh.subscribe(odom_topic_, 10, &PEKFSLAMNode::odomCallback, this);
    pc_sub_ = nh.subscribe(pc_topic_, 10, &PEKFSLAMNode::cloudCallback, this);

    timer_ = private_nh.createTimer(ros::Duration(1.0/max(freq,1.0)), &PEKFSLAMNode::start, this);

    // publish pose
    pose_pub_ = private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_, 10); //currnt pose
    poses_pub_ = private_nh.advertise<geometry_msgs::PoseArray>(poses_topic_, 10);// all poses

    // initialize
    filter_stamp_ = Time::now();
    _prev_time_stamp = ros::Time::now().toSec();

// subscribe to odom messages
    odom_meas = Eigen::VectorXd::Zero(3,1); new_meas = Eigen::VectorXd::Zero(3,1); odom_prev_pose = Eigen::VectorXd::Zero(3,1);
    change_ = Eigen::VectorXd::Zero(3,1);
    odom_cov = Eigen::MatrixXd::Zero(3,3);

    new_scan_ = false; is_pose_start =  false; new_odom_ = false;
    z_vec.reserve(100); z_cov_vec.reserve(100); Hp.reserve(100); 
  };




  // // destructor
  PEKFSLAMNode::~PEKFSLAMNode(){

  };





  // callback function for odom data
  void PEKFSLAMNode::odomCallback(const OdomConstPtr& msg)
  { 
    // get roll pitch and yaw from quaternion
    tf2::Quaternion q(msg->pose.pose.orientation.x, 
                      msg->pose.pose.orientation.y, 
                      msg->pose.pose.orientation.z, 
                      msg->pose.pose.orientation.w); // quaternion
    tf2::Matrix3x3 rot_1(q); // rotation matrix

    double roll, pitch, yaw;
    rot_1.getRPY(roll, pitch, yaw); //get the RPY angles from the quaternion
    new_meas << msg->pose.pose.position.x, msg->pose.pose.position.y, yaw; // set odom measurement
    odom_cov.diagonal() << msg->pose.covariance[0], msg->pose.covariance[7],  msg->pose.covariance[35]; // set odom covariance
    if (!new_odom_){
      odom_meas = new_meas - odom_prev_pose;

      odom_meas(2) = my_filter_.wrapAngle(odom_meas(2));
      odom_prev_pose = new_meas;
      new_odom_ = true;
    }

    is_pose_start = true;
  };


/*
* @brief add new scans to map vector
*/
  void PEKFSLAMNode::cloudCallback(const PointCloud2Ptr& msg)
{
    // add point cloud to the cloud_vector
  if(msg->header.stamp.toSec()- _prev_time_stamp > _time_between_cloud_points && !new_scan_ && change_.norm()>dist_threshold_){//only add cloud points if the time between them is greater than the _time_between_cloud_points threshold
    _prev_time_stamp = msg->header.stamp.toSec();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr w_frame_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    // pcl::transformPointCloud(*cloud, *w_frame_cloud, robot_pose);

    // map_vector.push_back(cloud);
    my_filter_.addScans(cloud);
    new_scan_ = true;

  }   

}



  // filter loop
  void PEKFSLAMNode::start(const ros::TimerEvent& e)
  { 
    tf2::Quaternion q;

    if (new_scan_ && my_filter_.isInitialized()){
      new_scan_ = false;
      int size = my_filter_.getScansVectorSize();
      // if(size<=1) 
      if (size>=2 && change_.norm()>dist_threshold_){
        // ROS_INFO("Update");
        my_filter_.addNewPose(odom_meas, odom_cov);
        new_odom_ = false;

        Hp.clear();
        z_vec.clear();
        z_cov_vec.clear();
        my_filter_.overLappingScans(new_meas, z_vec, z_cov_vec, Hp);
        // ROS_INFO("HP");
        if (Hp.size()>0){
          my_filter_.update(z_vec, z_cov_vec, Hp);
          change_.setZero();
          my_filter_.getPoses(poses, covs);
          geometry_msgs::PoseArray poses_msg;
          geometry_msgs::Pose poses_;

          for (int i=0; i<poses.rows(); i+=3){
            q.setRPY(0, 0, poses(i+2, 0));
            poses_msg.header.stamp = ros::Time::now();
            poses_msg.header.frame_id = "odom";
            poses_.position.x = poses(i,0);
            poses_.position.y = poses(i+1,1);
            poses_.position.z = 0;
            poses_.orientation.x = q.x();
            poses_.orientation.y = q.y();
            poses_.orientation.z = q.z();
            poses_.orientation.w = q.w();
            // poses_.pose.covariance[0] = covs(3*i,3*i);
            // poses_.pose.covariance[7] = covs(3*i+1,3*i+1);
            // poses_.pose.covariance[35] = covs(3*i+2,3*i+2);
            poses_msg.poses.push_back(poses_);
          }
          poses_pub_.publish(poses_msg);
        }

      }
    } else if(new_odom_ && !my_filter_.isInitialized()) { 
          ROS_INFO("Initialize"); 
          my_filter_.initialize(odom_meas, filter_stamp_);
          new_odom_ = false;

    }else if(new_odom_ && my_filter_.isInitialized()) { 
          my_filter_.predict(odom_meas, odom_cov);
          new_odom_ = false;

    };
    change_+=odom_meas; 
    // ROS_INFO("change_ %f, %d", change_.norm(), new_scan_);


    if(my_filter_.isInitialized()){
      // ROS_INFO("Publishing");
      my_filter_.getPose(pose, cov);
      // ROS_INFO("pose %f, %f, %f",pose(0), pose(1), pose(2));
      // ROS_INFO("cov %f, %f, %f",cov(0,0), cov(1,1), cov(2,2));
      //publish PoseWithCovarianceStamped
      geometry_msgs::PoseWithCovarianceStamped pose_msg;
      q.setRPY(0, 0, pose(2));
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = "odom";
      pose_msg.pose.pose.position.x = pose(0);
      pose_msg.pose.pose.position.y = pose(1);
      pose_msg.pose.pose.position.z = 0;
      pose_msg.pose.pose.orientation.x = q.x();
      pose_msg.pose.pose.orientation.y = q.y();
      pose_msg.pose.pose.orientation.z = q.z();
      pose_msg.pose.pose.orientation.w = q.w();
      pose_msg.pose.covariance[0] = cov(0,0);
      pose_msg.pose.covariance[7] = cov(1,1);
      pose_msg.pose.covariance[35] = cov(2,2);
      pose_pub_.publish(pose_msg);

      

    }

  };



}; // namespace



int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pekf_slam");

  // create filter class
  PEKFSLAMNode my_filter_node;

  ros::spin();
  
  return 0;
}