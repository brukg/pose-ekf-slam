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

// decompose Odom msg into x,y,z,roll,pitch,yaw
void PEKFSLAM::composeTransform(const Eigen::Vector3d& t1, 
            Eigen::Matrix4f& trans){
     double x,y,yaw;         
      trans = Eigen::Matrix4f::Identity();
      x = t1(0,3);   
      y = t1(1,3); 
      yaw = t1(2);
      trans(0,0) = cos(yaw); trans(0,1) = -sin(yaw);  trans(0,3) = x; 
      trans(1,0) = sin(yaw); trans(1,1) = cos(yaw);  trans(1,3) = y; 

  };

  void PEKFSLAM::setICPParams(double &_max_correspondence_distance, double &_transformation_epsilon, 
                              int &_max_iteration, double &_euclidean_fitness_epsilon, 
                              double &_ransac_iterations, double &_ransac_outlier_rejection_threshold)
  {
    this->max_correspondence_distance = _max_correspondence_distance;
    this->transformation_epsilon = _transformation_epsilon;
    this->max_iteration = _max_iteration;
    this->euclidean_fitness_epsilon = _euclidean_fitness_epsilon;
    this->ransac_iterations = _ransac_iterations;
    this->ransac_outlier_rejection_threshold = _ransac_outlier_rejection_threshold;
  };


  void PEKFSLAM::registerPointCloud(Eigen::Matrix4f &initial, const pclXYZPtr& target, const pclXYZPtr& source, Eigen::Matrix4f &transform, double& fitness)
  {

    pclXYZPtr target_cloud (new pcl::PointCloud<pcl::PointXYZ>(*target));
    pclXYZPtr source_cloud (new pcl::PointCloud<pcl::PointXYZ>(*source));
    pclXYZPtr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);
    gicp.setMaxCorrespondenceDistance(max_correspondence_distance);
    gicp.setMaximumIterations(max_iteration);
    gicp.setTransformationEpsilon(transformation_epsilon);
    gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    // gicp.setMaximumOptimizerIterations(100);
    gicp.setRANSACIterations(ransac_iterations);
    gicp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold);
    // gicp.setUseReciprocalCorrespondences(false);

    gicp.align(*transformed_cloud_ptr, initial);
    
    transform = gicp.getFinalTransformation();
    // gicp.computeTransformation(*transformed_cloud_ptr, initial);

    if (!gicp.hasConverged() || gicp.getFitnessScore()>1)
    {
      std::cout << "ICP has not converged." << std::endl;
      fitness = 100;
    }else{
      ROS_INFO("has converged: %d", gicp.hasConverged());
      fitness = gicp.getFitnessScore();
    }
  };

  void PEKFSLAM::overLappingScans(const Eigen::VectorXd& pose, std::vector<Eigen::Matrix4f>& trans, std::vector<double>& fitnesses, std::vector<int>& Hp)
  {
    // loop over scans_vector and get overlapping scans and their indices
    double fitness =100;
    Eigen::Matrix4f t, initial;
    pclXYZPtr last_cloud = scans_vector.back();

    Eigen::Vector3d dis, cur_pose, pose_to_check;
    cur_pose<<pose(0), pose(1), pose(2);
    ROS_INFO("scan_size %ld", scans_vector.size());
    for (int i=0; i<scans_vector.size()-1; i++){
      pose_to_check <<X.block<3,1>(i*3, 0);
      dis = cur_pose - pose_to_check; dis(2) = wrapAngle(dis(2));
      if (dis.norm() < 3){
        composeTransform(dis, initial); // initial geuss from displacement for icp 
        registerPointCloud(initial, scans_vector[i], last_cloud, t, fitness);

        ROS_INFO("dis %f", dis.norm());
        ROS_INFO("fitness %f", fitness);

        if (fitness < 1){
          ROS_INFO("pose_to_check %f %f", pose_to_check(0), pose_to_check(1));
          ROS_INFO("t %f %f ", t(0,3), t(1,3));
          Hp.push_back(i);
          trans.push_back(t);
          if(0.5<=fitness<=1)   fitnesses.push_back(5*fitness); // used for R matrix
          else if(0.1<=fitness<0.5)   fitnesses.push_back(pow(fitness,4)); // used for R matrix
          else if(0.05<=fitness<0.1)   fitnesses.push_back(pow(fitness,16)); // used for R matrix
          else if(0.01<=fitness<0.05)   fitnesses.push_back(pow(fitness,28)); // used for R matrix
          else if(0.001<=fitness<0.01)   fitnesses.push_back(pow(fitness,38)); // used for R matrix
          else    fitnesses.push_back(pow(fitness,48)); // used for R matrix
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