/**
 * @file controller.cpp
 * @brief Controller file
 *
 */

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
bool fControllerLoopDone = false;

// function for converting string to bool
bool string_to_bool(const std::string& x);

// function for converting bool to string
inline const char * const bool_to_string(bool b);

#define RAD(deg) ((double)(deg)*M_PI / 180.0)

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/stanbot.urdf";
const string human_file = "./resources/human.urdf";

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

	// initial state
	int state = POSTURE;
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

	// load robots, read current state and update the model
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();

	auto human = new Sai2Model::Sai2Model(human_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	human->updateModel();

	// prepare controllers
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	int human_dof = human->dof();
	VectorXd human_command_torques = VectorXd::Zero(human_dof);
	MatrixXd human_N_prec = MatrixXd::Identity(human_dof, human_dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_interpolation_flag = false;
	joint_task->_use_velocity_saturation_flag = false;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 400.0;
	joint_task->_kv = 40.0;

	VectorXd q_init_desired = robot->_q;
	joint_task->_desired_position = q_init_desired;

	// containers
	Vector3d ee_pos;
	Matrix3d ee_rot;

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToReadCallback(0, HUMAN_JOINT_ANGLES_KEY, human->_q);
	redis_client.addEigenToReadCallback(0, HUMAN_JOINT_VELOCITIES_KEY, human->_dq);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.addEigenToWriteCallback(0, HUMAN_JOINT_TORQUES_COMMANDED_KEY, human_command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); // secs
	bool fTimerDidSleep = true;

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

	// VectorXd triangle = VectorXd::Zero(dof);
	// triangle <<  -0.65,0.0,0.9,0.0,1.5,-1.5,0.0,0.35,0.0,-0.5,0.0,1.0,0.0,0.0,0.0,-1.5,1.5,1.5,0.0,0.0,0.0,0.0,-1.5,-1.5,-1.5,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

	unsigned long long counter = 0;

	runloop = true;

	// while (counter < 5000)
	// {
	// 	// wait for next scheduled loop
	// 	timer.waitForNextLoop();
	// 	double time = timer.elapsedTime() - start_time;

	// 	// execute redis read callback
	// 	redis_client.executeReadCallback(0);

	// 	// get user selected pose
	// 	// int current_pose = 5;
	// 	int current_pose = stoi(redis_client.get("PoseSelection"));
	// 	if (current_pose == CHAIR_POSE)
	// 	{
	// 		joint_task->_desired_position = chair;
	// 	}
	// 	else if (current_pose == TREE)
	// 	{
	// 		joint_task->_desired_position = tree;
	// 	}
	// 	else if (current_pose == WARRIOR_1)
	// 	{
	// 		joint_task->_desired_position = warrior_1;
	// 	}
	// 	else if (current_pose == WARRIOR_2)
	// 	{
	// 		joint_task->_desired_position = warrior_2;
	// 	}
	// 	else if (current_pose == WARRIOR_3)
	// 	{
	// 		joint_task->_desired_position = warrior_3;
	// 	}
	// 	else{
	// 		joint_task->_desired_position = chair;
	// 	}

	// 	// update model
	// 	robot->updateModel();

	// 	if (state == POSTURE)
	// 	{
	// 		// update task model and set hierarchy
	// 		N_prec.setIdentity();
	// 		joint_task->updateTaskModel(N_prec);

	// 		// compute torques
	// 		joint_task->computeTorques(joint_task_torques);
	// 		command_torques = joint_task_torques;

	// 		if ((robot->_q - q_init_desired).norm() < 0.15)
	// 		{
	// 			cout << "Posture To Motion" << endl;
	// 			joint_task->reInitializeTask();
	// 			// posori_task->reInitializeTask();
	// 			// robot->position(ee_pos, control_link, control_point);
	// 			// posori_task->_desired_position = ee_pos - Vector3d(-0.1, -0.1, 0.1);
	// 			// posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
	// 			// posori_task->_desired_orientation = AngleAxisd(0.0000000000000001, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

	// 			state = MOTION;
	// 		}
	// 	}
	// 	else if (state == MOTION)
	// 	{
	// 		// update task model and set hierarchy
	// 		N_prec.setIdentity();
	// 		// posori_task->updateTaskModel(N_prec);
	// 		// N_prec = posori_task->_N;
	// 		joint_task->updateTaskModel(N_prec);

	// 		// compute torques
	// 		// posori_task->computeTorques(posori_task_torques);
	// 		joint_task->computeTorques(joint_task_torques);
	// 		// command_torques = posori_task_torques + joint_task_torques;
	// 		command_torques = joint_task_torques;
	// 	}

	// 	// execute redis write callback
	// 	redis_client.executeWriteCallback(0);

	// 	counter++;
	// }

	// Eigen::Vector3d head_pos = redis_client.getEigenMatrixJSON(HEAD_POS_KEY);

	// // pose task
	// // rotate around x 90 and rotate around z 180
	// const string control_link = "head";
	// const Vector3d control_point = Vector3d(0, 0, 0.07);
	// auto posori_task = new Sai2Primitives::PosOriTask(human, control_link, control_point);
	// posori_task->_use_interpolation_flag = true;
	// posori_task->_use_velocity_saturation_flag = true;

	// VectorXd posori_task_torques = VectorXd::Zero(dof);
	// posori_task->_kp_pos = 400.0;
	// posori_task->_kv_pos = 40.0;
	// posori_task->_kp_ori = 400.0;
	// posori_task->_kv_ori = 40.0;

	// while (counter < 5000)
	// {
	// 	// wait for next scheduled loop
	// 	timer.waitForNextLoop();
	// 	double time = timer.elapsedTime() - start_time;

	// 	// execute redis read callback
	// 	redis_client.executeReadCallback(0);

	// 	// update model
	// 	robot->updateModel();

	// 	if (state == POSTURE)
	// 	{
	// 		// update task model and set hierarchy
	// 		N_prec.setIdentity();
	// 		joint_task->updateTaskModel(N_prec);

	// 		// compute torques
	// 		joint_task->computeTorques(joint_task_torques);
	// 		command_torques = joint_task_torques;

	// 		if ((robot->_q - q_init_desired).norm() < 0.15)
	// 		{
	// 			posori_task->reInitializeTask();
	// 			robot->position(ee_pos, control_link, control_point);
	// 			posori_task->_desired_position = ee_pos - Vector3d(-0.1, -0.1, 0.1);
	// 			posori_task->_desired_orientation = AngleAxisd(M_PI / 6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
	// 			posori_task->_desired_orientation = AngleAxisd(0.0000000000000001, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

	// 			state = MOTION;
	// 		}
	// 	}
	// 	else if (state == MOTION)
	// 	{
	// 		// update task model and set hierarchy
	// 		N_prec.setIdentity();
	// 		posori_task->updateTaskModel(N_prec);
	// 		N_prec = posori_task->_N;
	// 		joint_task->updateTaskModel(N_prec);

	// 		// compute torques
	// 		// posori_task->computeTorques(posori_task_torques);
	// 		joint_task->computeTorques(joint_task_torques);
	// 		// command_torques = posori_task_torques + joint_task_torques;
	// 		command_torques = joint_task_torques;
	// 	}

	// 	// execute redis write callback
	// 	redis_client.executeWriteCallback(0);

	// 	counter++;
	// }

	Eigen::Matrix3d kinect_to_world = AngleAxisd(-M_PI / 2, Vector3d::UnitX()).toRotationMatrix().matrix();

	// pose task for chest
	// string control_link = "chest";
	// Vector3d control_point = Vector3d(0, 0, 0);
	// auto posori_task_chest = new Sai2Primitives::PosOriTask(human, control_link, control_point);

	// posori_task_chest->_use_interpolation_flag = false;
	// posori_task_chest->_use_velocity_saturation_flag = false;

	// VectorXd posori_task_torques_chest = VectorXd::Zero(human_dof);
	// posori_task_chest->_kp_pos = 400.0;
	// posori_task_chest->_kv_pos = 40.0;
	// posori_task_chest->_kp_ori = 400.0;
	// posori_task_chest->_kv_ori = 40.0;

	// // set desired position and orientation to the initial configuration
	Vector3d x_pos;
	// human->positionInWorld(x_pos, control_link, control_point);
	Matrix3d x_ori;
	// human->rotationInWorld(x_ori, control_link);
	// posori_task_chest->_desired_position = x_pos;
	// posori_task_chest->_desired_orientation = x_ori;

	// pose task for left foot
	string control_link = "left_foot";
	Vector3d control_point = Vector3d(0, 0, 0);
	auto posori_task_left_foot = new Sai2Primitives::PosOriTask(human, control_link, control_point);

	posori_task_left_foot->_use_interpolation_flag = false;
	posori_task_left_foot->_use_velocity_saturation_flag = false;

	VectorXd posori_task_torques_left_foot = VectorXd::Zero(human_dof);
	posori_task_left_foot->_kp_pos = 400.0;
	posori_task_left_foot->_kv_pos = 40.0;
	posori_task_left_foot->_kp_ori = 400.0;
	posori_task_left_foot->_kv_ori = 40.0;

	// set desired position and orientation to the initial configuration
	human->position(x_pos, control_link, control_point);
	human->rotation(x_ori, control_link);
	// Eigen::Vector3d left_foot_pos = redis_client2.getEigenMatrixJSON(LEFT_FOOT_POS_KEY);
	// posori_task_left_foot->_desired_position = (kinect_to_world * left_foot_pos)/1000;
	// cout << posori_task_left_foot->_desired_position << endl;
	posori_task_left_foot->_desired_position = x_pos + Vector3d(0.3, 0, 0.1);
	posori_task_left_foot->_desired_orientation = x_ori;

	// pose task for right hand
	control_link = "right_hand";
	control_point = Vector3d(0, 0, 0);
	auto posori_task_right_hand = new Sai2Primitives::PosOriTask(human, control_link, control_point);

	posori_task_right_hand->_use_interpolation_flag = true;
	posori_task_right_hand->_use_velocity_saturation_flag = true;

	VectorXd posori_task_torques_right_hand = VectorXd::Zero(human_dof);
	posori_task_right_hand->_kp_pos = 400.0;
	posori_task_right_hand->_kv_pos = 40.0;
	posori_task_right_hand->_kp_ori = 400.0;
	posori_task_right_hand->_kv_ori = 40.0;

	// set two goal positions/orientations
	human->position(x_pos, control_link, control_point);
	human->rotation(x_ori, control_link);
	Eigen::Vector3d right_hand_pos = redis_client2.getEigenMatrixJSON(RIGHT_HAND_POS_KEY);
	// posori_task_right_hand->_desired_position = (kinect_to_world * right_hand_pos)/1000;
	posori_task_right_hand->_desired_position = x_pos + Vector3d(-0.3, 0, 0.1);
	// posori_task_right_hand->_desired_orientation = AngleAxisd(-3*M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori;
	// posori_task_right_hand->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori;

	// pose task for left hand
	control_link = "left_hand";
	control_point = Vector3d(0, 0, 0);
	auto posori_task_left_hand = new Sai2Primitives::PosOriTask(human, control_link, control_point);

	posori_task_left_hand->_use_interpolation_flag = false;
	posori_task_left_hand->_use_velocity_saturation_flag = false;

	VectorXd posori_task_torques_left_hand = VectorXd::Zero(dof);
	posori_task_left_hand->_kp_pos = 400.0;
	posori_task_left_hand->_kv_pos = 40.0;
	posori_task_left_hand->_kp_ori = 400.0;
	posori_task_left_hand->_kv_ori = 40.0;

	// set two goal positions/orientations
	human->position(x_pos, control_link, control_point);
	human->rotation(x_ori, control_link);
	Eigen::Vector3d left_hand_pos = redis_client2.getEigenMatrixJSON(LEFT_HAND_POS_KEY);
	// posori_task_left_hand->_desired_position = (kinect_to_world * left_hand_pos)/1000;
	posori_task_left_hand->_desired_position = x_pos + Vector3d(0.3, 0, 0.1);
	// posori_task_left_hand->_desired_orientation = AngleAxisd(-3*M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori;
	// posori_task_right_hand->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori;

	// pose task for head
	control_link = "head";
	control_point = Vector3d(0, 0, 0);
	auto posori_task_head = new Sai2Primitives::PosOriTask(human, control_link, control_point);
	posori_task_head->_use_interpolation_flag = false;
	posori_task_head->_use_velocity_saturation_flag = false;

	VectorXd posori_task_torques_head = VectorXd::Zero(human_dof);
	posori_task_head->_kp_pos = 400.0;
	posori_task_head->_kv_pos = 40.0;
	posori_task_head->_kp_ori = 400.0;
	posori_task_head->_kv_ori = 40.0;

	// set two goal positions/orientations
	human->position(x_pos, control_link, control_point);
	human->rotation(x_ori, control_link);
	Eigen::Vector3d head_pos = redis_client2.getEigenMatrixJSON(HEAD_POS_KEY);
	// posori_task_head->_desired_position = (kinect_to_world * head_pos)/1000;
	posori_task_head->_desired_position = x_pos;
	posori_task_head->_desired_orientation = AngleAxisd(M_PI / 3, Vector3d::UnitZ()).toRotationMatrix() * x_ori;

	// joint task
	auto human_joint_task = new Sai2Primitives::JointTask(human);
	human_joint_task->_use_interpolation_flag = false;
	human_joint_task->_use_velocity_saturation_flag = true;

	VectorXd human_joint_task_torques = VectorXd::Zero(human_dof);
	human_joint_task->_kp = 100;
	human_joint_task->_kv = 20;

	// set desired joint posture to be the initial robot configuration
	VectorXd human_q_init_desired = human->_q;
	human_joint_task->_desired_position = q_init_desired;

	// // setup redis callback
	// redis_client.createReadCallback(0);
	// redis_client.createWriteCallback(0);

	// // add to read callback
	// redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, human->_q);
	// redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, human->_dq);

	// // add to write callback
	// redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	// redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	while (runloop)
	{
		// wait for next scheduled loop
		// timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// fTimerDidSleep = timer.waitForNextLoop(); // commented out to let current controller loop finish before next loop

        // read simulation state
        fSimulationLoopDone = string_to_bool(redis_client.get(SIMULATION_LOOP_DONE_KEY));

        // run controller loop when simulation loop is done
        if (fSimulationLoopDone) {
			// execute redis read callback
			redis_client.executeReadCallback(0);

			// update model
			human->updateModel();

			// calculate torques for left foot
			human_N_prec.setIdentity();
			posori_task_left_foot->updateTaskModel(human_N_prec);
			posori_task_left_foot->computeTorques(posori_task_torques_left_foot);

			// // calculate torques to move right hand
			// human_N_prec = posori_task_left_foot->_N;
			// posori_task_right_hand->updateTaskModel(human_N_prec);
			// posori_task_right_hand->computeTorques(posori_task_torques_right_hand);

			// // calculate torques to move left hand
			// human_N_prec = posori_task_right_hand->_N;
			// posori_task_left_hand->updateTaskModel(human_N_prec);
			// posori_task_left_hand->computeTorques(posori_task_torques_left_hand);

			// // calculate torques to move head
			// human_N_prec = posori_task_left_hand->_N;
			// posori_task_head->updateTaskModel(human_N_prec);
			// posori_task_head->computeTorques(posori_task_torques_head);

			// // calculate torques to keep joint space
			human_N_prec = posori_task_head->_N;
			human_joint_task->updateTaskModel(human_N_prec);
			human_joint_task->computeTorques(human_joint_task_torques);

			// // calculate torques
			// command_torques = posori_task_torques_left_foot + posori_task_torques_right_hand + posori_task_torques_left_hand + posori_task_torques_head + joint_task_torques; // gravity compensation handled in sim
			human_command_torques = posori_task_torques_left_foot + human_joint_task_torques; // gravity compensation handled in sim
			// human_command_torques = posori_task_torques_head + human_joint_task_torques; // gravity compensation handled in sim

			// execute redis write callback
			redis_client.executeWriteCallback(0);

			// ask for next simulation loop
            fSimulationLoopDone = false;
            redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));

			counter++;
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

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques); // back to floating

	return 0;
}

//------------------------------------------------------------------------------

bool string_to_bool(const std::string& x) {
    assert(x == "false" || x == "true");
    return x == "true";
}

//------------------------------------------------------------------------------

inline const char * const bool_to_string(bool b)
{
    return b ? "true" : "false";
}
