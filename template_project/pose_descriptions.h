/**
 * @file pose_descriptions.h
 * @brief Contains joint angle descriptions for yoga poses.
 * 
 */

const int NONE = 0;
const int CHAIR_POSE = 1;
const int TREE = 2;
const int WARRIOR_1 = 3;
const int WARRIOR_2 = 4;
const int WARRIOR_3 = 5;
// const int TRIANGLE = 6;

const Eigen::VectorXd chair = Eigen::VectorXd(0.5, -1.5, 1.4, 0.0, 0.0, 0.0, 0.0, -1.4, 1.5, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
const Eigen::VectorXd tree = Eigen::VectorXd(0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -0.85, 1.9, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.1, 1.5, -0.6125, 0.125, 0.0, -1.5, -3.0, -0.1, -1.5, -0.6125, 0.125, 0.0, 1.5, 0.0, 0.0, 0.0);
const Eigen::VectorXd warrior_1 = Eigen::VectorXd(0.0, -1.3, 1.5, 0.0, 0.0, 0.0, 0.0, 0.6, 0.2, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
const Eigen::VectorXd warrior_2 = Eigen::VectorXd(0.0, -1.3, 1.5, 0.0, 1.5, -1.5, 0.0, 0.6, 0.2, -1.0, 0.0, 1.0, 0.2, 0.0, 0.0, -1.5, 1.5, 1.5, 0.0, 0.0, 0.0, 0.0, -1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5);
const Eigen::VectorXd warrior_3 = Eigen::VectorXd(0.0, -0.2, 1.4, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 1., 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
// const Eigen::VectorXd triangle = Eigen::VectorXd(-0.65,0.0,0.9,0.0,1.5,-1.5,0.0,0.35,0.0,-0.5,0.0,1.0,0.0,0.0,0.0,-1.5,1.5,1.5,0.0,0.0,0.0,0.0,-1.5,-1.5,-1.5,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
