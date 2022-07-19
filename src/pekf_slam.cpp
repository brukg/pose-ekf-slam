#include <pekf_slam/pekf_slam.h>
#include <pekf_slam/utils.h>

// using namespace MatrixWrapper;
// using namespace BFL;
// using namespace tf;
using namespace std;
// using namespace ros;


namespace pekfslam
{
  // constructor
  PEKFSLAM::PEKFSLAM()
  {
    X.setZero(3000,1); //to store 1000 poses
    P.setZero(3000,3000); //to store 1000 covariance matrices
    is_initialized = false; scans_vector.reserve(100);
    I = Eigen::Matrix3d::Identity(); //3x3 identity matrix
    prior_X.setZero(3,1); prior_P.setZero(3,3);
  };



  // destructor
  PEKFSLAM::~PEKFSLAM(){
  };


  // initialize prior density of filter 
  void PEKFSLAM::initialize(const Eigen::VectorXd &new_meas, const ros::Time& time)
  {

    X.block<3,1>(0,0) << new_meas;
    ROS_INFO("X %f, %f, %f ", X(0,0), X(1,0), X(2,0));

    P.diagonal().head(3) << 1e-9, 1e-9, 1e-9;
    ROS_INFO("P %f, %f, %f ", P(0,0), P(1,1), P(2,2));    
    prior_X = X.block<3,1>(0,0);
    prior_P = P.block<3,3>(0,0);
    

    // filter initialized
    is_initialized = true;
    index = 3;
  };



  
  void PEKFSLAM::calculate_Jfx(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &JFx){
    // set JFx to 3x3 identity matrix
    JFx = I;
  }

  void PEKFSLAM::calculate_Jfw(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &JFw){
    // set JFw to 3x3 identity matrix
    JFw = I;
  }

  void PEKFSLAM::calculate_Jhx(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &Jhx){
    // set Jhx to 3x3 identity matrix
    Jhx = I;
  }  

  void PEKFSLAM::calculate_Jhv(const Eigen::VectorXd &new_meas, Eigen::MatrixXd &Jhv){
    // set JFv to 3x3 identity matrix
    Jhv = I;
  } 

  void PEKFSLAM::expected_hx(Eigen::VectorXd &xs, Eigen::VectorXd &xk, Eigen::VectorXd &hx){
    hx = xk - xs; hx(2) = wrapAngle(hx(2));

  }
 
  void PEKFSLAM::observationMatrix(std::vector<Eigen::Matrix4f>& z_vec, std::vector<double>& z_cov_vec,  std::vector<int>& Hp,
                                    Eigen::VectorXd &y, Eigen::MatrixXd &R, Eigen::MatrixXd &H){

   
    int vec_size = index; 
    int hp_size = 3*Hp.size();
    int num_scans = 3*scans_vector.size(), f, e; f = (num_scans-1)*3; e = (num_scans-2)*3;
   
   
    Eigen::VectorXd z(hp_size,1), hxs(hp_size,1);  // z
    Eigen::VectorXd xs(3,1),xk(3,1), hx(3,1); 
    H.setZero(hp_size, vec_size); z.setZero(hp_size,1);
    R.setZero(hp_size,hp_size); 
    xs.setZero(); xk.setZero();
    hx.setZero(); y.setZero(hp_size,1);
    hxs.setZero(hp_size,1); y.setZero();

    xk << X.block<3,1>(index-3, 0); //current pose
    ROS_INFO("xk %f, %f, %f", xk(0), xk(1), xk(2));
    for (int i=0; i<Hp.size();i++){
      ROS_INFO("state_vector_size %d, matches %ld, id %d, mapsize %ld", vec_size, Hp.size(), Hp[i], scans_vector.size());

      decomposeTransform(z_vec.at(i), z(3*i,0), z(3*i+1,0), z(3*i+2,0));

      ROS_INFO("z %f, %f, %f", z(3*i), z(3*i+1), z(3*i+2));

      xs << X.block<3,1>(3*Hp[i], 0); //pose of robot at the  matching scan
      ROS_INFO("xs %f, %f, %f", xs(0), xs(1), xs(2));

      expected_hx(xs, xk, hx);
      hxs.block<3,1>(3*i,0) = hx;
      ROS_INFO("hx %f, %f, %f", hx(0), hx(1), hx(2));
      
      for (int j=0; j<Hp.size();j++){ 
        if(i==j) H.block<3,3>(3*j,3*Hp[i]) << -1*I; }      
      
      R.block<3,3>(3*i, 3*i) << z_cov_vec[i], 0,0,0,z_cov_vec[i],0,0,0, 2*z_cov_vec[i];

      
    }
    y << z - hxs;  // y = z - x' innovation

    for (int j=0; j<Hp.size();j++){ 
      H.block<3,3>(3*j,index-3) << I; 
      y(3*j+2) = wrapAngle(y(3*j+2)); // wrap angle to [-pi, pi]
    }


  };

  void PEKFSLAM::getPoses(Eigen::VectorXd &Xks, Eigen::MatrixXd &Pks){
    Xks= Eigen::VectorXd::Zero(index,1); Pks = Eigen::MatrixXd::Zero(index,index);

    Xks = this->X.block(0,0,index,1);
    Pks = this->P.block(0,0,index,index);
  }

  void PEKFSLAM::getPose(Eigen::Vector3d &Xk, Eigen::Matrix3d &Pk){
    // int i = scans_vector.size();
    Xk = this->X.block<3,1>(index-3, 0);
    Pk = this->P.block<3,3>(index-3,index-3);
  }
  
  void PEKFSLAM::predict(const Eigen::VectorXd &new_meas, const Eigen::MatrixXd &Q,
                          Eigen::VectorXd &pred_X, Eigen::MatrixXd &pred_P)
  {
    ROS_INFO("predict %d", index-3);

    int prev_index = index-3, end = index+3;
    // Eigen::VectorXd odom_meas(3);
    Eigen::MatrixXd JFx(3,3), JFw(3,3), P_(3,3), P_H(3,index);

    calculate_Jfx(new_meas, JFx);
    calculate_Jfw(new_meas, JFw);
    // odom_meas = new_meas - odom_prev_pose;  odom_meas(2) = wrapAngle(odom_meas(2));
    // ROS_INFO("Predic P %f,", P.sum());
    	  
    // ekf prediction
    prior_X << prior_X + new_meas;
    pred_X = prior_X;
    ROS_INFO("prior_X predict  %f, %f, %f", prior_X(0), prior_X(1), prior_X(2));
    prior_X(2) = wrapAngle(prior_X(2));
    // ROS_INFO("X %f, %f, %f", X(index-3), X(index-2), X(index-1));
    // P = P + Q;
    pred_P << JFx*prior_P *JFx.transpose() + JFw*Q*JFw.transpose();
    prior_P << pred_P;



  };

  void PEKFSLAM::addNewPose(const Eigen::VectorXd &new_meas, const Eigen::MatrixXd &R)
  { 

    ROS_INFO("newpose %d", index);
	  Eigen::VectorXd meas(3);
    int prev_index = index-3, end = index+3;

    Eigen::MatrixXd JFx(3,3), P_(3,3), P_H(3,index);
    P_H.setZero(); P_.setZero(); JFx.setZero();
    calculate_Jfx(new_meas, JFx);
    // meas = new_meas - odom_prev_pose; 
    // meas(2) = wrapAngle(meas(2));

    // ekf prediction
    x = prior_X + new_meas; 
    x(2) = wrapAngle(x(2));
    X.block<3,1>(index,0) = x;
    ROS_INFO("X %f, %f, %f", X(index), X(index+1), X(index+2));

    
    P.block<3,3>(index,index) = JFx*prior_P *JFx.transpose() + JFx*R*JFx.transpose();
    // ROS_INFO("PP before 2  %f, %ld, %ld", P.sum(), P.rows(), P.cols());

    P_H = JFx*P.block(prev_index, 0, index, index);
    
    P.block(index, 0, index+3, index) << P_H;

    
    P.block(0, index, index, index+3) << P_H.transpose();

    // ROS_INFO("P %f, %f", P(0,0), P(index,index));
    // ROS_INFO("PP add %f, %ld, %ld", P.sum(), P.rows(), P.cols());
    prior_X << X.block<3,1>(index,0);
    prior_P <<P.block<3,3>(index,index);
    index+=3;

  };


  // update filter
  void PEKFSLAM::update(Eigen::VectorXd &y, Eigen::MatrixXd &R, Eigen::MatrixXd &H,  std::vector<int> &Hp)
  {
    //
    ROS_INFO("update %d", index);
    int vec_size = index; 
    int hp_size = 3*Hp.size();
    int num_scans = 3*scans_vector.size(), f, e; f = (num_scans-1)*3; e = (num_scans-2)*3;
    // int index = 3*(scans_vector.size()-1);

    Eigen::VectorXd X_(vec_size,1);;  // z
    Eigen::MatrixXd Z(hp_size,hp_size),  Z_inv(hp_size,hp_size), E(hp_size,hp_size);  // R, Z, Z_inv, E=H*P*H'
    Eigen::MatrixXd K(vec_size, hp_size), PHt(vec_size,hp_size);       // K, P*H^T

    Eigen::MatrixXd P_(vec_size, vec_size), PP_(vec_size, vec_size);
    Z.setZero(); K.setZero();  X_.setZero(); PHt.setZero();  P_.setZero(); PP_.setZero(); E.setZero(); 
    PP_ << P.block(0,0,vec_size,vec_size);
    X_.setZero();  
    PHt << PP_*H.transpose();
    E << H*PHt;
    Z << E + R;
    Z_inv << Z.completeOrthogonalDecomposition().pseudoInverse();
    K << PHt* Z_inv;
    X_ << K*y;
    X.block(0,0,vec_size,0) << X.block(0,0,vec_size,0) + X_; // X = X + K*y;
    ROS_INFO("X %f, %F, %F", X(index-3),  X(index-2), X(index-1));
    P_ << K*Z*K.transpose(); // P = P - K*Z*K';
    P.block(0,0, vec_size, vec_size) << PP_ - P_; // update covariance 
  };




}; // namespace
