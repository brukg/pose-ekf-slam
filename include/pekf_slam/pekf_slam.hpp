#ifndef PEKFSLAM_H
#define PEKFSLAM_H

// #include <pcl_conversions/pcl_conversions.hpp>
#include <eigen3/Eigen/Dense>

// TF
// #include <tf/tf.hpp>
#include <tf2/LinearMath/Transform.h>

// msgs
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/registration/gicp.h> //generalized iterative closest point algorithm from pcl
#include <pcl/registration/icp.h> //generalized iterative closest point algorithm from pcl

// log files
#include <fstream>
#include <math.h>
#include <rclcpp/rclcpp.hpp>

namespace pekfslam
{

class PEKFSLAM
{
public:
  /// constructor
  PEKFSLAM();

  /// destructor
  virtual ~PEKFSLAM();




  void initialize(const Eigen::VectorXd &new_meas);

  bool isInitialized() {return is_initialized;};
  void predict(const Eigen::VectorXd &dis_pose, const Eigen::MatrixXd &Q,  
                        Eigen::VectorXd &pred_X, Eigen::MatrixXd &pred_P);
  void addNewPose(const Eigen::VectorXd &dis_pose, const Eigen::MatrixXd &R);
  void update(Eigen::VectorXd &y, Eigen::MatrixXd &R, Eigen::MatrixXd &H,  std::vector<int> &Hp);

  void calculate_Jfx(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &JFx);
  void calculate_Jfw(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &JFw);
  void calculate_Jhx(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &Jhx);
  void calculate_Jhv(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &Jhv);
  void expected_hx(Eigen::VectorXd &xs, Eigen::VectorXd &xk, Eigen::VectorXd &hx);

  void addScans(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan);
  void overLappingScans(const Eigen::VectorXd& pose, std::vector<Eigen::Matrix4f>& trans, std::vector<double>& fitnesses, std::vector<int>& Hp);
  void registerPointCloud(Eigen::Matrix4f &initial, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source, Eigen::Matrix4f &transform, double& fitness);
  void setICPParams(double &_max_correspondence_distance, double &_transformation_epsilon, 
                            int &_max_iteration, double &_euclidean_fitness_epsilon, 
                            int &_ransac_iterations, double &_ransac_outlier_rejection_threshold);

  void observationMatrix(std::vector<Eigen::Matrix4f>& z_vec, std::vector<double>& z_cov_vec,  std::vector<int>& Hp,
                          Eigen::VectorXd &y, Eigen::MatrixXd &R, Eigen::MatrixXd &H);
  
  /*
  * @brief: wrap angle between -PI and PI
  * @param: a: angle to be wrapped
  * @return: wrapped angle
  */
  double wrapAngle(double& a);

  // void getEstimate(MatrixWrapper::ColumnVector& estimate);
  int getScansVectorSize();
  void getPoses(Eigen::VectorXd &Xs, Eigen::MatrixXd &Pks);
  void getPose(Eigen::Vector3d &Xk, Eigen::Matrix3d &Pk);

  //to be implemented
  void setOutputFrame(const std::string& output_frame);
  void setBaseFootprintFrame(const std::string& base_frame);

private:
 /*
 * @brief decompose transformation matrix into rotation and translation
 * @input: transform: transformation matrix
 * @output: rotation: rotation matrix(yaw)
 * @output: x, y
 */
  void decomposeTransform(const Eigen::Matrix4f& trans,
			  double& x, double& y, double& yaw);
  /*
  * @brief 
  */
  void composeTransform(const Eigen::Vector3d& t1, 
            Eigen::Matrix4f& trans);

  Eigen::VectorXd X, x;
  Eigen::MatrixXd P;
  Eigen::Matrix3d I;
  Eigen::VectorXd prior_X;
  Eigen::MatrixXd prior_P;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scans_vector;
  int index;

  bool is_initialized, odom_initialized_, imu_initialized_, vo_initialized_;
  double max_correspondence_distance, transformation_epsilon, euclidean_fitness_epsilon, ransac_iterations, ransac_outlier_rejection_threshold;
  int max_iteration;
  

  std::string output_frame_;
  std::string base_footprint_frame_;
}; // class

}; // namespace

#endif
