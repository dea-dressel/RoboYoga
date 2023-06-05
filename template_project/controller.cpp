/**
 * @file controller.cpp
 * @brief Controller file
 *
 */
/*bring out the initialization of tasks and put everything in callback, possibily reversing sync code*/
#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

#include <signal.h>
bool runloop = false;
void sighandler(int) { runloop = false; }
bool fSimulationLoopDone = false;
bool initialized = false;
bool fControllerLoopDone = false;

// function for converting string to bool
bool string_to_bool(const std::string &x);

// function for converting bool to string
inline const char *const bool_to_string(bool b);

#define RAD(deg) ((double)(deg)*M_PI / 180.0)

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
// const string robot_file = "./resources/stanbot.urdf";
const string human_file = "./resources/human.urdf";

const int NONE = 0;
const int CHAIR_POSE = 1;
const int TREE = 2;
const int WARRIOR_1 = 3;
const int WARRIOR_2 = 4;
const int WARRIOR_3 = 5;

// const int TRIANGLE = 6;

enum State
{
	POSTURE = 0,
	MOTION
};

int main()
{
	string controller_status = "1";

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	auto redis_client2 = RedisClient();
	redis_client2.connect("192.168.1.70");

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// add human
	auto human = new Sai2Model::Sai2Model(human_file, false);
	human->updateModel();

	// prepare controllers
	int dof = human->dof();

	int human_dof = human->dof();
	VectorXd human_command_torques = VectorXd::Zero(human_dof);
	MatrixXd human_N_prec = MatrixXd::Identity(human_dof, human_dof);

	Eigen::Vector3d left_foot_pos;
	Eigen::Vector3d init_kinect_left_foot_pos;
	Eigen::Matrix3d init_kinect_left_foot_ori;
	Eigen::Matrix3d left_foot_ori;

	Eigen::Vector3d right_hand_pos;
	Eigen::Vector3d init_kinect_right_hand_pos;
	Eigen::Matrix3d init_kinect_right_hand_ori;
	Eigen::Matrix3d right_hand_ori;

	Eigen::Vector3d left_hand_pos;
	Eigen::Vector3d init_kinect_left_hand_pos;
	Eigen::Matrix3d init_kinect_left_hand_ori;
	Eigen::Matrix3d left_hand_ori;

	Eigen::Vector3d pelvis_pos;
	Eigen::Matrix3d pelvis_ori;
	Eigen::Vector3d init_kinect_pelvis_pos;
	Eigen::Matrix3d init_kinect_pelvis_ori;

	Eigen::Vector3d chest_pos;
	Eigen::Matrix3d init_kinect_chest_ori;
	Eigen::Vector3d init_kinect_chest_pos;
	Eigen::Matrix3d chest_ori;

	Eigen::Vector3d head_pos;
	Eigen::Vector3d init_kinect_head_pos;

	redis_client.setEigenMatrixJSON(INIT_PELVIS_POS_KEY, Matrix3d::Identity());

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, HUMAN_JOINT_ANGLES_KEY, human->_q);
	redis_client.addEigenToReadCallback(0, HUMAN_JOINT_VELOCITIES_KEY, human->_dq);
	redis_client.addEigenToReadCallback(0, INIT_LEFT_FOOT_POS_KEY, init_kinect_left_foot_pos);
	redis_client.addEigenToReadCallback(0, INIT_RIGHT_HAND_POS_KEY, init_kinect_right_hand_pos);
	redis_client.addEigenToReadCallback(0, INIT_LEFT_HAND_POS_KEY, init_kinect_left_hand_pos);
	redis_client.addEigenToReadCallback(0, INIT_HEAD_POS_KEY, init_kinect_head_pos);
	redis_client.addEigenToReadCallback(0, INIT_PELVIS_POS_KEY, init_kinect_pelvis_pos);
	redis_client.addEigenToReadCallback(0, INIT_CHEST_POS_KEY, init_kinect_chest_pos);

	redis_client.addEigenToReadCallback(0, INIT_LEFT_FOOT_ORI_KEY, init_kinect_left_foot_ori);
	redis_client.addEigenToReadCallback(0, INIT_RIGHT_HAND_ORI_KEY, init_kinect_right_hand_ori);
	redis_client.addEigenToReadCallback(0, INIT_LEFT_HAND_ORI_KEY, init_kinect_left_hand_ori);
	redis_client.addEigenToReadCallback(0, INIT_CHEST_ORI_KEY, init_kinect_chest_ori);
	redis_client.addEigenToReadCallback(0, INIT_PELVIS_ORI_KEY, init_kinect_pelvis_ori);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, HUMAN_JOINT_TORQUES_COMMANDED_KEY, human_command_torques);

	redis_client2.createReadCallback(0);
	redis_client2.addEigenToReadCallback(0, LEFT_FOOT_POS_KEY, left_foot_pos);
	redis_client2.addEigenToReadCallback(0, RIGHT_HAND_POS_KEY, right_hand_pos);
	redis_client2.addEigenToReadCallback(0, LEFT_HAND_POS_KEY, left_hand_pos);
	redis_client2.addEigenToReadCallback(0, PELVIS_POS_KEY, pelvis_pos);
	redis_client2.addEigenToReadCallback(0, CHEST_POS_KEY, chest_pos);
	// CHANGE THIS TO HEAD

	redis_client2.addEigenToReadCallback(0, HEAD_POS_KEY, head_pos);
	redis_client2.addEigenToReadCallback(0, LEFT_FOOT_ORI_KEY, left_foot_ori);
	redis_client2.addEigenToReadCallback(0, RIGHT_HAND_ORI_KEY, right_hand_ori);
	redis_client2.addEigenToReadCallback(0, LEFT_HAND_ORI_KEY, left_hand_ori);
	redis_client2.addEigenToReadCallback(0, CHEST_ORI_KEY, chest_ori);
	redis_client2.addEigenToReadCallback(0, PELVIS_ORI_KEY, pelvis_ori);

	// // create a timer
	// LoopTimer timer;
	// timer.initializeTimer();
	// timer.setLoopFrequency(1000);
	// double start_time = timer.elapsedTime(); // secs
	// bool fTimerDidSleep = true;

	/*******************************************************************************************************************************/

	// initialize pose descriptions
	VectorXd chair = VectorXd::Zero(dof);
	chair << 0.5, -1.5, 1.4, 0.0, 0.0, 0.0, 0.0, -1.4, 1.5, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	VectorXd tree = VectorXd::Zero(dof);
	tree << 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -0.85, 1.9, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.1, 1.5, -0.6125, 0.125, 0.0, -1.5, -3.0, -0.1, -1.5, -0.6125, 0.125, 0.0, 1.5, 0.0, 0.0, 0.0;

	VectorXd warrior_1 = VectorXd::Zero(dof);
	warrior_1 << 0.0, -1.3, 1.5, 0.0, 0.0, 0.0, 0.0, 0.6, 0.2, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	VectorXd warrior_2 = VectorXd::Zero(dof);
	warrior_2 << 0.0, -1.3, 1.5, 0.0, 1.5, -1.5, 0.0, 0.6, 0.2, -1.0, 0.0, 1.0, 0.2, 0.0, 0.0, -1.5, 1.5, 1.5, 0.0, 0.0, 0.0, 0.0, -1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5;

	VectorXd warrior_3 = VectorXd::Zero(dof);
	warrior_3 << 0.0, -0.2, 1.4, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 1., 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	/***********************************************************************************************************************************/
	string control_link;
	Vector3d control_point = Vector3d(0, 0, 0);
	Vector3d x_pos;

	Eigen::Matrix3d kinect_to_world = AngleAxisd(-M_PI / 2, Vector3d::UnitX()).toRotationMatrix().matrix();

	// pose task for left foot
	Vector3d old_left_foot_pos;
	Vector3d init_left_foot_pos;
	Matrix3d init_left_foot_ori;

	old_left_foot_pos = redis_client2.getEigenMatrixJSON(LEFT_FOOT_POS_KEY);

	control_link = "left_foot";
	Matrix3d rotation_offset_left_foot = AngleAxisd(M_PI, Vector3d(1, 0, 0)).toRotationMatrix() * AngleAxisd(M_PI / 2, Vector3d(0, 1, 0)).toRotationMatrix();
	auto posori_task_left_foot = new Sai2Primitives::PosOriTask(human, control_link, control_point, rotation_offset_left_foot);
	// auto posori_task_left_foot = new Sai2Primitives::PositionTask(human, control_link, control_point);

	posori_task_left_foot->_use_interpolation_flag = false;
	posori_task_left_foot->_use_velocity_saturation_flag = false;
	human->positionInWorld(init_left_foot_pos, control_link, control_point);
	human->rotationInWorld(init_left_foot_ori, control_link);
	posori_task_left_foot->_desired_position = init_left_foot_pos;
	posori_task_left_foot->_desired_orientation = init_left_foot_ori;

	VectorXd posori_task_torques_left_foot = VectorXd::Zero(human_dof);
	posori_task_left_foot->_kp_pos = 100.0;
	posori_task_left_foot->_kv_pos = 20.0;
	posori_task_left_foot->_kp_ori = 100.0;
	posori_task_left_foot->_kv_ori = 20.0;

	// posori_task_left_foot->_kp = 100.0;
	// posori_task_left_foot->_kv = 20.0;

	// pose task for right hand
	Vector3d old_right_hand_pos;
	Vector3d init_right_hand_pos;
	Matrix3d init_right_hand_ori;

	old_right_hand_pos = redis_client2.getEigenMatrixJSON(RIGHT_HAND_POS_KEY);

	control_link = "right_hand";
	// auto posori_task_right_hand = new Sai2Primitives::PosOriTask(human, control_link, control_point);
	auto posori_task_right_hand = new Sai2Primitives::PositionTask(human, control_link, control_point);

	posori_task_right_hand->_use_interpolation_flag = false;
	posori_task_right_hand->_use_velocity_saturation_flag = false;
	human->positionInWorld(init_right_hand_pos, control_link, control_point);
	// human->rotationInWorld(init_right_hand_ori, control_link);
	posori_task_right_hand->_desired_position = init_right_hand_pos;
	// posori_task_right_hand->_desired_orientation = init_right_hand_ori;

	VectorXd posori_task_torques_right_hand = VectorXd::Zero(human_dof);
	// posori_task_right_hand->_kp_pos = 100.0;
	// posori_task_right_hand->_kv_pos = 20.0;
	// posori_task_right_hand->_kp_ori = 100.0;
	// posori_task_right_hand->_kv_ori = 20.0;

	posori_task_right_hand->_kp = 100.0;
	posori_task_right_hand->_kv = 20.0;

	// pose task for left hand
	Vector3d old_left_hand_pos;
	Vector3d init_left_hand_pos;
	Matrix3d init_left_hand_ori;

	old_left_hand_pos = redis_client2.getEigenMatrixJSON(LEFT_HAND_POS_KEY);

	control_link = "left_hand";
	// auto posori_task_left_hand = new Sai2Primitives::PosOriTask(human, control_link, control_point);
	auto posori_task_left_hand = new Sai2Primitives::PositionTask(human, control_link, control_point);

	posori_task_left_hand->_use_interpolation_flag = false;
	posori_task_left_hand->_use_velocity_saturation_flag = false;
	human->positionInWorld(init_left_hand_pos, control_link, control_point);
	// human->rotationInWorld(init_left_hand_ori, control_link);
	posori_task_left_hand->_desired_position = init_left_hand_pos;
	// posori_task_left_hand->_desired_orientation = init_left_hand_ori;

	VectorXd posori_task_torques_left_hand = VectorXd::Zero(human_dof);
	// posori_task_left_hand->_kp_pos = 100.0;
	// posori_task_left_hand->_kv_pos = 20.0;
	// posori_task_left_hand->_kp_ori = 100.0;
	// posori_task_left_hand->_kv_ori = 20.0;

	posori_task_left_hand->_kp = 100.0;
	posori_task_left_hand->_kv = 20.0;

	// pose task for pelvis
	Vector3d old_pelvis_pos;
	Vector3d init_pelvis_pos;
	Matrix3d init_pelvis_ori;

	old_pelvis_pos = redis_client2.getEigenMatrixJSON(CHEST_POS_KEY);

	control_link = "hip";
	// auto posori_task_pelvis = new Sai2Primitives::PositionTask(human, control_link, control_point);
	Matrix3d rotation_offset_pelvis = AngleAxisd(M_PI, Vector3d(1, 0, 0)).toRotationMatrix() * AngleAxisd(M_PI / 2, Vector3d(0, 1, 0)).toRotationMatrix();
	auto posori_task_pelvis = new Sai2Primitives::OrientationTask(human, control_link, control_point, rotation_offset_pelvis);

	posori_task_pelvis->_use_interpolation_flag = false;
	posori_task_pelvis->_use_velocity_saturation_flag = false;
	// human->positionInWorld(init_pelvis_pos, control_link, control_point);
	human->rotationInWorld(init_pelvis_ori, control_link);
	// posori_task_pelvis->_desired_position = init_pelvis_pos;
	posori_task_pelvis->_desired_orientation = init_pelvis_ori;

	VectorXd posori_task_torques_pelvis = VectorXd::Zero(human_dof);
	posori_task_pelvis->_kp = 100.0;
	posori_task_pelvis->_kv = 20.0;

	// pose task for chest
	Vector3d old_chest_pos;
	Vector3d init_chest_pos;
	Matrix3d init_chest_ori;

	old_chest_pos = redis_client2.getEigenMatrixJSON(CHEST_POS_KEY);

	control_link = "chest";
	// auto posori_task_chest = new Sai2Primitives::PositionTask(human, control_link, control_point);
	Matrix3d rotation_offset = AngleAxisd(M_PI, Vector3d(1, 0, 0)).toRotationMatrix() * AngleAxisd(M_PI / 2, Vector3d(0, 1, 0)).toRotationMatrix();
	auto posori_task_chest = new Sai2Primitives::PosOriTask(human, control_link, control_point, rotation_offset);

	posori_task_chest->_use_interpolation_flag = false;
	posori_task_chest->_use_velocity_saturation_flag = false;
	human->positionInWorld(init_chest_pos, control_link, control_point);
	human->rotationInWorld(init_chest_ori, control_link);
	posori_task_chest->_desired_position = init_chest_pos;
	posori_task_chest->_desired_orientation = init_chest_ori;

	VectorXd posori_task_torques_chest = VectorXd::Zero(human_dof);
	posori_task_chest->_kp_pos = 100.0;
	posori_task_chest->_kv_pos = 20.0;
	posori_task_chest->_kp_ori = 100.0;
	posori_task_chest->_kv_ori = 20.0;

	// pose task for head
	Vector3d old_head_pos;
	Vector3d init_head_pos;
	Matrix3d init_head_ori;

	old_head_pos = redis_client2.getEigenMatrixJSON(HEAD_POS_KEY);

	control_link = "head";
	auto posori_task_head = new Sai2Primitives::PositionTask(human, control_link, control_point);
	posori_task_head->setDynamicDecouplingNone();

	posori_task_head->_use_interpolation_flag = false;
	posori_task_head->_use_velocity_saturation_flag = true;
	human->positionInWorld(init_head_pos, control_link, control_point);
	posori_task_head->_desired_position = init_head_pos;

	VectorXd posori_task_torques_head = VectorXd::Zero(human_dof);
	posori_task_head->_kp = 100.0;
	posori_task_head->_kv = 20.0;

	// joint task
	auto human_joint_task = new Sai2Primitives::JointTask(human);
	human_joint_task->_use_interpolation_flag = false;
	human_joint_task->_use_velocity_saturation_flag = true;

	VectorXd human_joint_task_torques = VectorXd::Zero(human_dof);
	human_joint_task->_kp = 0.1;
	human_joint_task->_kv = 20;

	float scale = 0.001;
	unsigned long long counter = 0;
	runloop = true;

	int current_pose = NONE;
	try
	{
		current_pose = stoi(redis_client.get(POSE_SELECTION_KEY));
	}
	catch (...)
	{
		cout << "No pose selected" << endl;
	}

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); // secs
	bool fTimerDidSleep = true;

	while (runloop)
	{
		// wait for next scheduled loop
		// timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// fTimerDidSleep = timer.waitForNextLoop(); // commented out to let current controller loop finish before next loop

		// read simulation state
		fSimulationLoopDone = string_to_bool(redis_client.get(SIMULATION_LOOP_DONE_KEY));
		initialized = string_to_bool(redis_client.get(INITIALIZED_KEY));

		// run controller loop when simulation loop is done
		if (fSimulationLoopDone)
		{
			if (initialized)
			{
				redis_client2.executeReadCallback(0);
				redis_client.executeReadCallback(0);
				/******************************************************************************************************/
				/********************************************KINECT HUMAN**********************************************/
				/******************************************************************************************************/

				// left foot desired
				if ((left_foot_pos - old_left_foot_pos).norm() < 4)
				{
					// posori_task_left_foot -> _desired_position += Vector3d(0.1, 0.1 , 0.1);
					// posori_task_left_foot->_desired_position = init_left_foot_pos;
					posori_task_left_foot->_desired_position = init_left_foot_pos + (kinect_to_world * (left_foot_pos - init_kinect_left_foot_pos)) * scale * 0.25;
					// if (posori_task_left_foot->_desired_position(2) < init_left_foot_pos(2) + 0.03)
					// {
					// 	posori_task_left_foot->_desired_position(2) = init_left_foot_pos(2) + 0.03;
					// }
				}
				// posori_task_left_foot->_desired_orientation = init_left_foot_ori;
				// posori_task_left_foot->_desired_orientation = kinect_to_world * left_foot_ori * init_left_foot_ori.transpose() * init_kinect_left_foot_ori * AngleAxisd(-M_PI, Vector3d::UnitX()).toRotationMatrix().matrix();
				posori_task_left_foot->_desired_orientation = kinect_to_world * left_foot_ori;
				cout << "left:" << (left_foot_pos).transpose() << endl;
				cout << "init left:" << (init_kinect_left_foot_pos).transpose() << endl;
				cout << "init left foot:" << (init_left_foot_pos).transpose() << endl;
				cout << "desired: " << (posori_task_left_foot->_desired_position).transpose() << endl;
				cout << "delta:" << ((kinect_to_world * (left_foot_pos - init_kinect_left_foot_pos)) * scale * 0.25).transpose() << endl;
				old_left_foot_pos = left_foot_pos;

				// right hand desired
				if ((right_hand_pos - old_right_hand_pos).norm() < 4)
				{
					posori_task_right_hand->_desired_position = init_right_hand_pos + (kinect_to_world * (right_hand_pos - init_kinect_right_hand_pos)) * scale;
				}
				// posori_task_right_hand->_desired_orientation = kinect_to_world*right_hand_ori*init_kinect_right_hand_ori.transpose()*init_right_hand_ori;
				// posori_task_right_hand->_desired_orientation = kinect_to_world * right_hand_ori * init_right_hand_ori.transpose() * init_kinect_right_hand_ori * AngleAxisd(-M_PI, Vector3d::UnitX()).toRotationMatrix().matrix();
				old_right_hand_pos = right_hand_pos;

				// left hand desired
				if ((left_hand_pos - old_left_hand_pos).norm() < 4)
				{
					posori_task_left_hand->_desired_position = init_left_hand_pos + (kinect_to_world * (left_hand_pos - init_kinect_left_hand_pos)) * scale;
				}
				// posori_task_left_hand->_desired_orientation = init_left_hand_ori;
				// posori_task_left_hand->_desired_orientation = kinect_to_world * left_hand_ori * init_left_hand_ori.transpose() * init_kinect_left_hand_ori * AngleAxisd(-M_PI, Vector3d::UnitX()).toRotationMatrix().matrix();
				old_left_hand_pos = left_hand_pos;

				// pelvis desired
				// if ((pelvis_pos - old_pelvis_pos).norm() < 4)
				// {
				// 	posori_task_pelvis->_desired_position = init_pelvis_pos + (kinect_to_world * (pelvis_pos - init_kinect_pelvis_pos)) * scale;
				// }
				posori_task_pelvis->_desired_orientation = kinect_to_world * pelvis_ori;
				old_pelvis_pos = pelvis_pos;

				posori_task_chest->_desired_position = init_chest_pos + kinect_to_world * (chest_pos - init_kinect_chest_pos) * scale;
				posori_task_chest->_desired_orientation = kinect_to_world * chest_ori;

				old_chest_pos = chest_pos;

				// // head desired
				// if ((head_pos - old_head_pos).norm() < 4)
				// {
				// 	posori_task_head->_desired_position = init_head_pos + (kinect_to_world * (head_pos - init_kinect_head_pos)) * scale;
				// }
				// posori_task_head->_desired_orientation = init_head_ori;
				// old_head_pos = head_pos;

				// joint task
				// set desired joint posture to be the initial robot configuration
				VectorXd human_q_init_desired = human->_q;
				// human_joint_task->_desired_position = human_q_init_desired;
				// switch (current_pose)
				// {
				// case CHAIR_POSE:
				// 	human_joint_task->_desired_position = chair;
				// 	break;
				// case TREE:
				// 	human_joint_task->_desired_position = tree;
				// 	break;
				// case WARRIOR_1:
				// 	human_joint_task->_desired_position = warrior_1;
				// 	break;
				// case WARRIOR_2:
				// 	human_joint_task->_desired_position = warrior_2;
				// 	break;
				// case WARRIOR_3:
				// 	human_joint_task->_desired_position = warrior_3;
				// 	break;
				// }

				/***********************************************CALCULATING TORQUES***************************************/

				// // update model
				human->updateModel();

				// // calculate torques for head
				// human_N_prec.setIdentity();
				// posori_task_head->updateTaskModel(human_N_prec);
				// posori_task_head->computeTorques(posori_task_torques_head);

				// // calculate torques for pelvis
				// human_N_prec = posori_task_head->_N;
				// posori_task_pelvis->updateTaskModel(human_N_prec);
				// posori_task_pelvis->computeTorques(posori_task_torques_pelvis);

				// calculate torques for left foot
				human_N_prec.setIdentity();
				posori_task_left_foot->updateTaskModel(human_N_prec);
				posori_task_left_foot->computeTorques(posori_task_torques_left_foot);

				// calculate torques for chest
				human_N_prec = posori_task_left_foot->_N;
				// human_N_prec.setIdentity();
				posori_task_chest->updateTaskModel(human_N_prec);
				posori_task_chest->computeTorques(posori_task_torques_chest);

				// // calculate torques for left foot
				// human_N_prec = posori_task_chest->_N;
				// posori_task_left_foot->updateTaskModel(human_N_prec);
				// posori_task_left_foot->computeTorques(posori_task_torques_left_foot);

				// // calculate torques for left hand
				// human_N_prec = posori_task_chest->_N;
				// posori_task_left_hand->updateTaskModel(human_N_prec);
				// posori_task_left_hand->computeTorques(posori_task_torques_left_hand);

				// calculate torques for right hand
				human_N_prec = posori_task_left_foot->_N;
				posori_task_right_hand->updateTaskModel(human_N_prec);
				posori_task_right_hand->computeTorques(posori_task_torques_right_hand);

				// calculate torques to keep joint space
				// human_N_prec = posori_task_pelvis->_N;
				human_N_prec = posori_task_right_hand->_N;
				human_joint_task->updateTaskModel(human_N_prec);
				human_joint_task->computeTorques(human_joint_task_torques);

				human_command_torques = 0 * posori_task_torques_chest + human_joint_task_torques + posori_task_torques_left_foot + 0 * posori_task_torques_right_hand + 0 * posori_task_torques_left_hand + 0 * posori_task_torques_pelvis + 0 * posori_task_torques_head;
				// execute redis write callback
				redis_client.executeWriteCallback(0);

				// ask for next simulation loop
				fSimulationLoopDone = false;
				redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));

				counter++;
			}
			else
			{
				human_command_torques = 0 * posori_task_torques_chest;
			}
		}

		// controller loop is done
		fControllerLoopDone = true;
		redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));
	}

	// controller loop is turned off
	fControllerLoopDone = false;
	redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Controller Loop frequency : " << timer.elapsedCycles() / end_time << "Hz\n";

	// redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques); // back to floating

	return 0;
}

//------------------------------------------------------------------------------

bool string_to_bool(const std::string &x)
{
	assert(x == "false" || x == "true");
	return x == "true";
}

//------------------------------------------------------------------------------

inline const char *const bool_to_string(bool b)
{
	return b ? "true" : "false";
}