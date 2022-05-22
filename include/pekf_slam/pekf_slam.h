#ifndef PEKFSLAM_H
#define PEKFSLAM_H

#include <pcl_conversions/pcl_conversions.h>
#include <eigen3/Eigen/Dense>

// TF
#include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>

// msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/registration/gicp.h> //generalized iterative closest point algorithm from pcl

// log files
#include <fstream>
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pclXYZPtr;
typedef boost::shared_ptr<sensor_msgs::PointCloud2 const> PointCloud2Ptr;
namespace pekfslam
{

class PEKFSLAM
{
public:
  /// constructor
  PEKFSLAM();

  /// destructor
  virtual ~PEKFSLAM();




  void initialize(const Eigen::VectorXd &new_meas, const ros::Time& time);

  bool isInitialized() {return is_initialized;};
  void predict(const Eigen::VectorXd &dis_pose, const Eigen::MatrixXd &Q);
  void addNewPose(const Eigen::VectorXd &dis_pose, const Eigen::MatrixXd &R);
  void update(std::vector<Eigen::Matrix4f>& z, std::vector<double>& z_cov_vec, std::vector<int>& Hp);

  void calculate_Jfx(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &JFx);
  void calculate_Jfw(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &JFw);
  void calculate_Jhx(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &Jhx);
  void calculate_Jhv(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &Jhv);
  void expected_hx(Eigen::VectorXd &xs, Eigen::VectorXd &xk, Eigen::VectorXd &hx);

  void addScans(const pclXYZPtr& scan);
  void overLappingScans(const Eigen::VectorXd& pose, std::vector<Eigen::Matrix4f>& trans, std::vector<double>& fitnesses, std::vector<int>& Hp);
  void registerPointCloud(const pclXYZPtr& target, const pclXYZPtr& source, Eigen::Matrix4f &transform, double& fitness);
  void observationMatrix(const pclXYZPtr& target, const pclXYZPtr& source, Eigen::Matrix4d &transform);
  double wrapAngle(double& a);

  // void getEstimate(MatrixWrapper::ColumnVector& estimate);
  int getScansVectorSize();
  void getPoses(Eigen::VectorXd &Xs, Eigen::MatrixXd &Pks);
  void getPose(Eigen::Vector3d &Xk, Eigen::Matrix3d &Pk);
  void setOutputFrame(const std::string& output_frame);

  void setBaseFootprintFrame(const std::string& base_frame);

private:
  /// wrap angle between -PI and PI


  void decomposeTransform(const Eigen::Matrix4f& trans,
			  double& x, double& y, double& yaw);

  Eigen::VectorXd X, x;
  Eigen::MatrixXd P;
  Eigen::Matrix3d I;

  std::vector<pclXYZPtr> scans_vector;
  int index;


  // tf::Transform filter_estimate_old_, old_X, odom_prev_pose;
  // tf::StampedTransform odom_meas_, odom_meas_old_, imu_meas_, imu_meas_old_, vo_meas_, vo_meas_old_;
  ros::Time filter_time_old_;
  bool is_initialized, odom_initialized_, imu_initialized_, vo_initialized_;


  // tf transformer
  tf::Transformer transformer_;

  std::string output_frame_;
  std::string base_footprint_frame_;
}; // class

}; // namespace

#endif
