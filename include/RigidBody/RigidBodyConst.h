#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODYCONST_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODYCONST_H_

#include <Eigen/Dense>

const Eigen::MatrixXd kDGDqs = (Eigen::MatrixXd(3, 4) << 0.0, 1.0, 0.0, 0.0,
														 0.0, 0.0, 1.0, 0.0,
														 0.0, 0.0, 0.0, 1.0).finished();
const Eigen::MatrixXd kDGDqx = (Eigen::MatrixXd(3, 4) << -1.0, 0.0, 0.0, 0.0,
														 0.0, 0.0, 0.0, 1.0, 
														 0.0, 0.0, -1.0, 0.0).finished();
const Eigen::MatrixXd kDGDqy = (Eigen::MatrixXd(3, 4) << 0.0, 0.0, 0.0, -1.0, 
														 -1.0, 0.0, 0.0, 0.0,
														 0.0, 1.0, 0.0, 0.0).finished();
const Eigen::MatrixXd kDGDqz = (Eigen::MatrixXd(3, 4) << 0.0, 0.0, 1.0, 0.0, 
														 0.0, -1.0, 0.0, 0.0, 
														 -1.0, 0.0, 0.0, 0.0).finished();

const double kAlpha = 1000.0;
const double kBeta = 1000.0;

const double kG = 9.80665;

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODYCONST_H_