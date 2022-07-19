#include <pekf_slam/pekf_slam_node.h>


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
    private_nh.param("odom_topic", odom_topic_, std::string("/odom"));
    private_nh.param("imu_topic",  imu_topic_, std::string("/imu"));
    private_nh.param("pc_topic",   pc_topic_, std::string("/camera/depth/points"));
    private_nh.param("updated_pose_topic",   update_pose_topic_, std::string("/pose_slam/updated_pose"));
    private_nh.param("predicted_pose_topic",   predict_pose_topic_, std::string("/pose_slam/predicted_pose"));
    private_nh.param("poses_topic",   poses_topic_, std::string("/pose_slam/pose"));
    private_nh.param("time_between_cloud_points",   _time_between_cloud_points, 1.0);
    private_nh.param("noise",   _noise, 0.0);
    private_nh.param("dist_threshold",   dist_threshold_, 1.0);


    // ICP params from yaml file
    private_nh.param("transformation_epsilon", _transformation_epsilon, 0.01);
    private_nh.param("max_iterations", _max_iters, 75);
    private_nh.param("euclidean_fitness_epsilon", _euclidean_fitness_epsilon, 0.01);
    private_nh.param("max_correspondence_distance", _max_correspondence_distance, 1.0);
    private_nh.param("ransac_iterations", _ransac_iterations, 1.0);
    private_nh.param("ransac_threshold", _ransac_threshold, 1.0);
    double freq;
    private_nh.param("freq", freq, 30.0);


    odom_sub_ = nh.subscribe(odom_topic_, 10, &PEKFSLAMNode::odomCallback, this);
    imu_sub_ = nh.subscribe(imu_topic_, 10, &PEKFSLAMNode::imuCallback, this);
    pc_sub_ = nh.subscribe(pc_topic_, 10, &PEKFSLAMNode::cloudCallback, this);

    timer_ = private_nh.createTimer(ros::Duration(1.0/max(freq,1.0)), &PEKFSLAMNode::start, this);

    // publish pose
    predicted_pose_pub_ = private_nh.advertise<nav_msgs::Odometry>(predict_pose_topic_, 10); //predicted pose 
    updated_pose_pub_ = private_nh.advertise<nav_msgs::Odometry>(update_pose_topic_, 10); //corrected pose 
    poses_pub_ = private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(poses_topic_, 10);// all poses

    // initialize
    filter_stamp_ = Time::now();
    _prev_time_stamp = 0;//ros::Time::now().toSec();

// subscribe to odom messages
    odom_meas = Eigen::VectorXd::Zero(3,1); new_meas = Eigen::VectorXd::Zero(3,1); odom_prev_pose = Eigen::VectorXd::Zero(3,1);
    change_ = Eigen::VectorXd::Zero(3,1);
    odom_cov = Eigen::MatrixXd::Zero(3,3);

    new_scan_ = false; is_pose_start =  false; new_odom_ = false;
    z_vec.reserve(1000); z_cov_vec.reserve(1000); Hp.reserve(1000); 
    prev_dist = 0.0;
    dead_rec.setZero(3000,1);
    pred_X.setZero(3,1); pred_P.setZero(3,3);
    my_filter_.setICPParams(_max_correspondence_distance, _transformation_epsilon, _max_iters, _euclidean_fitness_epsilon, _ransac_iterations, _ransac_threshold);
  };




  // // destructor
  PEKFSLAMNode::~PEKFSLAMNode(){

  };

  void PEKFSLAMNode::imuCallback(const ImuConstPtr& msg)
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



  // callback function for odom data
  void PEKFSLAMNode::odomCallback(const OdomConstPtr& msg)
  { 
    new_meas << msg->pose.pose.position.x, msg->pose.pose.position.y, imu_yaw; // set odom measurement
    odom_cov.diagonal() << 0.5*msg->pose.covariance[0], 0.5*msg->pose.covariance[7],  0.5*msg->pose.covariance[35]; // set odom covariance
    if (!new_odom_){
      odom_meas = new_meas - odom_prev_pose + (_noise*Eigen::VectorXd::Random(3, 1));

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
  if(!my_filter_.isInitialized() || (msg->header.stamp.toSec()- _prev_time_stamp > _time_between_cloud_points && !new_scan_ && change_.norm()>dist_threshold_)){//only add cloud points if the time between them is greater than the _time_between_cloud_points threshold
    _prev_time_stamp = msg->header.stamp.toSec();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    ROS_INFO("new scan at  %f %f %f", new_meas(0), new_meas(1), new_meas(2));


    my_filter_.addScans(cloud);
    new_scan_ = true;

  }   

}



  // filter loop
  void PEKFSLAMNode::start(const ros::TimerEvent& e)
  { 
    tf2::Quaternion q;
    change_+=odom_meas; 

    if (new_scan_ && my_filter_.isInitialized()){
      new_scan_ = false;
      int size = my_filter_.getScansVectorSize();
      // if(size<=1) 
      if (size>=2 && change_.norm()>dist_threshold_){
        // ROS_INFO("Update");
        new_odom_ = false;
        change_.setZero(); //new pose added reset counter
        my_filter_.addNewPose(odom_meas, odom_cov);

        Hp.clear();
        z_vec.clear();
        z_cov_vec.clear();
        my_filter_.overLappingScans(new_meas, z_vec, z_cov_vec, Hp);
        // ROS_INFO("HP");
        Eigen::VectorXd y; Eigen::MatrixXd R; Eigen::MatrixXd H;
        if (Hp.size()>0){
          my_filter_.observationMatrix(z_vec, z_cov_vec, Hp, y, R, H);
          my_filter_.update(y, R, H, Hp);
          my_filter_.getPoses(poses, covs);
          
          nav_msgs::Odometry poses_;

          for (int i=0; i<poses.rows(); i+=3){
            q.setRPY(0, 0, poses(i+2, 0));
            poses_.header.stamp = filter_stamp_;
            poses_.header.frame_id = "odom";
            poses_.header.seq = 0;//i;
            poses_.pose.pose.position.x = poses(i,0);
            poses_.pose.pose.position.y = poses(i+1,1);
            poses_.pose.pose.position.z = 0;
            poses_.pose.pose.orientation.x = q.x();
            poses_.pose.pose.orientation.y = q.y();
            poses_.pose.pose.orientation.z = q.z();
            poses_.pose.pose.orientation.w = q.w();
            poses_.pose.covariance[0] = covs(i,i);
            poses_.pose.covariance[7] = covs(i+1,i+1);
            poses_.pose.covariance[35] = covs(i+2,i+2);
            // poses_msg.poses.push_back(poses_);
            updated_pose_pub_.publish(poses_);
          }
         
        }

        my_filter_.getPose(pose, cov);

        geometry_msgs::PoseWithCovarianceStamped cur_pose;
        q.setRPY(0, 0, pose(2));
        cur_pose.header.stamp = ros::Time::now();
        cur_pose.header.frame_id = "odom";
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
        poses_pub_.publish(cur_pose);

      }
    } else if(new_odom_ && new_scan_ && !my_filter_.isInitialized()) { 
          ROS_INFO("Initialize"); 
          my_filter_.initialize(odom_meas, filter_stamp_);
          new_odom_ = false;
          new_scan_ = false; //scann added reser for new scan
          change_.setZero();

    }else if(new_odom_ && my_filter_.isInitialized()) { 
          my_filter_.predict(odom_meas, odom_cov,  pred_X, pred_P);
          new_odom_ = false;

    };

    ROS_INFO("change_ %f, %d", change_.norm(), new_scan_);


    if(my_filter_.isInitialized()){
      // ROS_INFO("Publishing");
      nav_msgs::Odometry pose_msg;
      q.setRPY(0, 0, pred_X(2));
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = "odom";
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
      predicted_pose_pub_.publish(pose_msg);

      

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
