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

#define RAD(deg) ((double)(deg)*M_PI / 180.0)

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/stanbot.urdf";

const int CHAIR_POSE = 1; // YES
const int TREE = 2;
const int WARRIOR_1 = 3;	
const int WARRIOR_2 = 4;
const int WARRIOR_3 = 5;
const int TRIANGLE = 6;		// YES
const int FORWARD_FOLD = 7;	// YES
const int STAR = 8; 		// YES
const int HORSE = 9;


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

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = true;

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

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); // secs
	bool fTimerDidSleep = true;

	VectorXd chair = VectorXd::Zero(dof);
	chair << 0.0, 0.5, -1.5, 1.4, 0.0, 0.0, 0.0, 0.0, -1.4, 1.5, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	VectorXd tree = VectorXd::Zero(dof);
	tree << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -0.85, 1.9, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.1, 1.5, -0.6125, 0.125, 0.0, -1.5, -3.0, -0.1, -1.5, -0.6125, 0.125, 0.0, 1.5, 0.0, 0.0, 0.0;

	VectorXd warrior_1 = VectorXd::Zero(dof);
	warrior_1 << 0.0, 0.0, -1.3, 1.5, 0.0, 0.0, 0.0, 0.0, 0.6, 0.2, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	VectorXd warrior_2 = VectorXd::Zero(dof);
	// forward view 
	// warrior_2 << 0.0, 0.0, -1.3, 1.5, 0.0, 1.5, -1.5, 0.0, 0.6, 0.2, -1.0, 0.0, 1.0, 0.2, 0.0, 0.0, -1.5, 1.5, 1.5, 0.0, 0.0, 0.0, 0.0, -1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5;
	// side view
	warrior_2 << RAD(45), 0.0, RAD(1), 0.0, RAD(-45), 0.0, RAD(90), 0.0, RAD(-70), RAD(80), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5, 1.5, 1.5, 0.0, 0.0, 0.0, 0.0, -1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5;

	VectorXd warrior_3 = VectorXd::Zero(dof);
	warrior_3 << 0.0, 0.0, -0.2, 1.4, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 1., 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	VectorXd triangle = VectorXd::Zero(dof);
	triangle << RAD(25), 0.0, 0.0, 0.0, RAD(15), 0.0, 0.0, RAD(-80), 0.0, 0.0, 0.0, RAD(40), RAD(90), RAD(35), 0.0, 0.0, 0.0,RAD(90),0.0,0.0,0.0,0.0,0.0,0.0,RAD(-92),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
	
	VectorXd forward_fold = VectorXd::Zero(dof);
	forward_fold << 0.0, 0.0, 0.0, RAD(45), 0.0, 0.0, 0.0, 0.0, RAD(-45), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, RAD(40), RAD(-90),0.0,0.0,0.0,0.0,0.0,0.0,RAD(-90),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

	VectorXd star = VectorXd::Zero(dof);
	star << RAD(25), 0.0, 0.0, 0.0, RAD(-25), 0.0, 0.0, RAD(-25),0.0, 0.0, 0.0, RAD(25), 0.0, 0.0, 0.0, 0.0, 0.0, RAD(90), 0.0,0.0,0.0,0.0,0.0,0.0,RAD(-90),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
	
	VectorXd horse = VectorXd::Zero(dof);
	horse << RAD(25), 0.0, RAD(-45), RAD(45), RAD(-45), 0.0, 0.0, RAD(-45),RAD(-45), RAD(45), 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, RAD(90), 0.0,0.0,0.0,0.0,0.0,0.0,RAD(-90),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
	
	
	unsigned long long counter = 0;

	runloop = true;

	while (runloop)
	{
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// get user selected pose
		// int current_pose = redis_client.get("PoseSelection");
		int current_pose = WARRIOR_2;
		if (current_pose == CHAIR_POSE) {
			joint_task->_desired_position = chair;
		} else if (current_pose == TREE) {
			joint_task->_desired_position = tree;
		} else if (current_pose == WARRIOR_1) {
			joint_task->_desired_position = warrior_1;
		} else if (current_pose == WARRIOR_2) {
			joint_task->_desired_position = warrior_2;
		} else if (current_pose == WARRIOR_3) {
			joint_task->_desired_position = warrior_3;
		} else if (current_pose == TRIANGLE) {
			joint_task->_desired_position = triangle;
		} else if (current_pose == FORWARD_FOLD) {
			joint_task->_desired_position = forward_fold;
		} else if (current_pose == STAR) {
			joint_task->_desired_position = star;
		} else if (current_pose == HORSE) {
			joint_task->_desired_position = horse;
		}
		


		// update model
		robot->updateModel();

		if (state == POSTURE)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques;

			if ((robot->_q - q_init_desired).norm() < 0.15)
			{
				cout << "Posture To Motion" << endl;
				joint_task->reInitializeTask();
				state = MOTION;
			}
		}
		else if (state == MOTION)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques;
		}

		// execute redis write callback
		redis_client.executeWriteCallback(0);

		counter++;
	}


	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Controller Loop frequency : " << timer.elapsedCycles() / end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques); // back to floating

	return 0;
}
