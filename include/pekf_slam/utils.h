#include <pekf_slam/pekf_slam.h>
namespace pekfslam
{
// correct for angle overflow
double PEKFSLAM::wrapAngle(double& a)
{ a = fmod(a, (2*M_PI));
  while ((a) >  M_PI) a -= 2*M_PI;
  return a;
  // while ((a) < -M_PI) a += 2*M_PI;
};

int PEKFSLAM::getScansVectorSize()
{
  return scans_vector.size();
};
void PEKFSLAM::addScans(const pclXYZPtr& scan)
{
  scans_vector.push_back(scan);
  // std::cout<<"scan added";
};
// decompose Odom msg into x,y,z,roll,pitch,yaw
void PEKFSLAM::decomposeTransform(const Eigen::Matrix4f& trans,
			  double& x, double& y, double& yaw){
  x = trans(0,3);   
  y = trans(1,3); 
  yaw = atan2(trans(1,0), trans(0,0));
};



  
  void PEKFSLAM::registerPointCloud(const pclXYZPtr& target, const pclXYZPtr& source, Eigen::Matrix4f &transform, double& fitness)
  {

    pclXYZPtr target_cloud (new pcl::PointCloud<pcl::PointXYZ>(*target));
    pclXYZPtr source_cloud (new pcl::PointCloud<pcl::PointXYZ>(*source));
    pclXYZPtr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setTransformationEpsilon(0.08);
    icp.setMaximumIterations(30);
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setEuclideanFitnessEpsilon(0.001);
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.align(*transformed_cloud_ptr);

    // icp.align(*transformed_cloud_ptr);
    transform = icp.getFinalTransformation();
    fitness = icp.getFitnessScore();

  };

  void PEKFSLAM::overLappingScans(const Eigen::VectorXd& pose, std::vector<Eigen::Matrix4f>& trans, std::vector<double>& fitnesses, std::vector<int>& Hp)
  {
    // loop over scans_vector and get overlapping scans and their indices
    double fitness;
    Eigen::Matrix4f t;
    pclXYZPtr last_cloud = scans_vector.back();

    Eigen::Vector2d dis, cur_pose, pose_to_check;
    cur_pose<<pose(0), pose(1);
    ROS_INFO("scan_size %ld", scans_vector.size());
    for (int i=0; i<scans_vector.size()-1; i++){
      pose_to_check <<X.block<2,1>(i*3, 0);
      dis = pose_to_check - cur_pose;
      if (dis.norm() < 2.0){
        registerPointCloud(last_cloud, scans_vector[i], t, fitness);

        ROS_INFO("dis %f", dis.norm());
        ROS_INFO("fitness %f", fitness);

        if (fitness <= 0.001){
          ROS_INFO("pose_to_check %f %f", pose_to_check(0), pose_to_check(1));
          ROS_INFO("t %f %f ", t(0,3), t(1,3));
          Hp.push_back(i);
          trans.push_back(t);
          fitnesses.push_back(pow(fitness, 1)); // used for R matrix
        }
      }
    }

  };



void PEKFSLAM::setOutputFrame(const std::string& output_frame){
output_frame_ = output_frame;
};

void PEKFSLAM::setBaseFootprintFrame(const std::string& base_frame){
base_footprint_frame_ = base_frame;
};

// void PEKFSLAM::overlappingScans(std::vector<pclXYZPtr>& map_vector){

// }
}