
#ifndef ESTIMATOR_NODE_H
#define ESTIMATOR_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <ros/ros.h>

// #include <ros/TimerEvent.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>


class EstimatorNode {
 public:
  EstimatorNode();
  ~EstimatorNode();

  void Publish();

 private:

  // Define here the matrices and vectors of the Kalman filter
  
  // Prediction
	  
	  Eigen::VectorXd X(6);		// State vector
	  Eigen::MatrixXd F(6,6);	// Transition matrix
  	  Eigen::MatrixXd G(6,3);	// Input matrix
 	  Eigen::VectorXd u(3);	  	// Input vector
 	  Eigen::MatrixXd Q(6,6);	// Process noise covariance matrix
  	  Eigen::MatrixXd P(6,6);	// System covariance matrix

  // Update

 	  Eigen::MatrixXd K(6,6);	// Kalman filter gain
 	  Eigen::MatrixXd H(3,6);	// Measurement matrix
 	  Eigen::VectorXd z(3);		// Measurement vector
 	  Eigen::Matrix3d R;		// Measurement noise covariance matrix
  

 	  Eigen::Vector4d ground_truth_pose;

  // subscribers
  ros::Subscriber pose_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber ground_truth_sub_;

  ros::Publisher pose_pub;
  geometry_msgs::PoseStamped msgPose_;
  

  ros::Timer timer_;

  void PoseCallback(
      const geometry_msgs::PoseStampedConstPtr& pose_msg);

  void ImuCallback(
      const sensor_msgs::ImuConstPtr& imu_msg);

  void TimedCallback(
      const ros::TimerEvent& e);

  void EstimatorNode::GroundTruthCallback(
  	  const geometry_msgs::PoseStampedConstPtr &pose_msg);
};

#endif // ESTIMATOR_NODE_H