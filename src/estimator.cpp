#include "estimator.h"

#define Gravity 9.81

double dT, dT_2, dT_3;
double sigma_x, sigma_x_sq;
double lastTime;
double X_pred, P_pred, z_pred;

 // Prediction
    
    Eigen::VectorXd X(6);   // State vector
    Eigen::MatrixXd F(6,6); // Transition matrix
    Eigen::MatrixXd G(6,3); // Input matrix
    Eigen::VectorXd u(3);     // Input vector
    Eigen::MatrixXd Q(6,6); // Process noise covariance matrix
    Eigen::MatrixXd P(6,6); // System covariance matrix

  // Update

    Eigen::MatrixXd K(6,6); // Kalman filter gain
    Eigen::MatrixXd H(3,6); // Measurement matrix
    Eigen::VectorXd z(3);   // Measurement vector
    Eigen::Matrix3d R;    // Measurement noise covariance matrix
  

    Eigen::Vector4d ground_truth_pose;

EstimatorNode::EstimatorNode() {


  ros::NodeHandle nh;

  pose_sub_ = nh.subscribe("/firefly/fake_gps/pose",  1, &EstimatorNode::PoseCallback, this);
  imu_sub_  = nh.subscribe("/firefly/imu",            1, &EstimatorNode::ImuCallback, this);
  ground_truth_sub_ = nh.subscribe("/firefly/ground_truth/pose", 1, &EstimatorNode::GroundTruthCallback, this);

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly/pose", 1);

  timer_ = nh.createTimer(ros::Duration(0.1), &EstimatorNode::TimedCallback, this);

  // Variables
  lastTime = ros::Time::now().toSec();

  sigma_x = 0.1;
  sigma_x_sq = pow(sigma_x,2);
  

  // Initial state
  X(0) = 0;
  X(1) = 0;
  X(2) = 0;
  X(3) = 0;
  X(4) = 0;
  X(5) = 0;

  u(0) = 0;
  u(1) = 0;
  u(2) = 0;

  z(0) = 0;
  z(1) = 0;
  z(2) = 0;


  // Matrices
  // Transition matrix
  F << 1, 0, 0, dT, 0, 0,
       0, 1, 0, 0, dT, 0,
       0, 0, 1, 0, 0, dT,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

  // Input matrix
  G << dT_2, 0, 0,
       0, dT_2, 0,
       0, 0, dT_2,
         dT, 0, 0,
         0, dT, 0,
         0, 0, dT;

  // Process noise covariance matrix
  Q << dT_3, 0, 0, dT_2, 0, 0,
       0, dT_3, 0, 0, dT_2, 0,
       0, 0, dT_3, 0, 0, dT_2,
       dT_2, 0, 0, dT_2, 0, 0,
       0, dT_2, 0, 0, dT_2, 0,
       0, 0, dT_2, 0, 0, dT_2;

  Q = sigma_x_sq * Q;


  // Measurement matrix
  H << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0;

  // Measurement noise covariance matrix
  R << sigma_x_sq, 0, 0,
       0, sigma_x_sq, 0,
       0, 0, sigma_x_sq;

  // System covariance matrix
  P << sigma_x, 0, 0, 0, 0, 0,
       0, sigma_x, 0, 0, 0, 0,
       0, 0, sigma_x, 0, 0, 0,
       0, 0, 0, sigma_x, 0, 0,
       0, 0, 0, 0, sigma_x, 0,
       0, 0, 0, 0, 0, sigma_x;
                      
}

EstimatorNode::~EstimatorNode() { }

void EstimatorNode::Publish()
{
  //publish your data
  ROS_INFO("Publishing ...");
  msgPose_.pose.position.x = X(0);
  msgPose_.pose.position.y = X(1);
  msgPose_.pose.position.z = X(2);
  msgPose_.pose.orientation.x = ground_truth_pose(0);
  msgPose_.pose.orientation.y = ground_truth_pose(1);
  msgPose_.pose.orientation.z = ground_truth_pose(2);
  msgPose_.pose.orientation.w = ground_truth_pose(3);

  pose_pub.publish(msgPose_);
}

void EstimatorNode::PoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {

  // Updating the pose acording to a measurement vector from the pose_msg

  ROS_INFO_ONCE("Estimator got first pose message.");
  msgPose_.header.stamp = pose_msg->header.stamp;
  msgPose_.header.seq = pose_msg->header.seq;
  msgPose_.header.frame_id =pose_msg->header.frame_id;

  // Data adquisition to update the measurement vector
  z(0) = pose_msg->pose.position.x;
  z(1) = pose_msg->pose.position.y;
  z(2) = pose_msg->pose.position.z;

  // Update or correction (based on measurements)
  K = P_pred * H.transpose() * (H * P_pred * H.transpose() + R).inverse();
  X = X_pred + K * (z - z_pred);
  P = P_pred - K * H * P_pred;
}

void EstimatorNode::ImuCallback(
    const sensor_msgs::ImuConstPtr& imu_msg) {

  // Predicting the posa thanks to IMU's data
  ROS_INFO_ONCE("Estimator got first IMU message.");

  msgPose_.header.stamp = imu_msg->header.stamp;
  msgPose_.header.seq = imu_msg->header.seq;
  msgPose_.header.frame_id =imu_msg->header.frame_id;

  // Update input vector
  u(0) = imu_msg->linear_acceleration.x;
  u(1) = imu_msg->linear_acceleration.y;

  // Gravity compensation
  u(3) = imu_msg->linear_acceleration.z - Gravity;

  // Prediction
  X_pred = F * X + G * u;
  P_pred = F * P * F.transpose() + Q;
  z_pred = H * X_pred;
}

void EstimatorNode::TimedCallback(
      const ros::TimerEvent& e){
   ROS_INFO_ONCE("Timer initiated.");

   dT = ros::Time::now().toSec() - lastTime;
   dT_2 = 0.5*pow(dT,2);
   dT_3 = 0.5*pow(dT,3);

   lastTime = ros::Time::now().toSec();

   Publish();
}

void EstimatorNode::GroundTruthCallback(
      const geometry_msgs::PoseStampedConstPtr &pose_msg){

    ROS_INFO_ONCE("Ground truth initiated.");

    ground_truth_pose(0) = pose_msg->pose.orientation.x;
    ground_truth_pose(1) = pose_msg->pose.orientation.y;
    ground_truth_pose(2) = pose_msg->pose.orientation.z;
    ground_truth_pose(3) = pose_msg->pose.orientation.w;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "estimator");

  EstimatorNode estimator_node;

  ros::spin();

  return 0;
}