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

const std::string HEAD_POS_KEY = "kinect::pos::head";

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
	human->_q = redis_client.getEigenMatrixJSON(HUMAN_JOINT_ANGLES_KEY);
	human->_dq = redis_client.getEigenMatrixJSON(HUMAN_JOINT_VELOCITIES_KEY);
	human->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	int human_dof = human->dof();
	VectorXd human_command_torques = VectorXd::Zero(human_dof);
	MatrixXd human_N_prec = MatrixXd::Identity(human_dof, human_dof);

	Eigen::Vector3d head_pos;
	redis_client.setEigenMatrixJSON(HEAD_POS_KEY, head_pos);

	// pose task
	// const string control_link = "link7";
	// const Vector3d control_point = Vector3d(0, 0, 0.07);
	// auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	// posori_task->_use_interpolation_flag = true;
	// posori_task->_use_velocity_saturation_flag = true;

	// VectorXd posori_task_torques = VectorXd::Zero(dof);
	// posori_task->_kp_pos = 400.0;
	// posori_task->_kv_pos = 40.0;
	// posori_task->_kp_ori = 400.0;
	// posori_task->_kv_ori = 40.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 400.0;
	joint_task->_kv = 40.0;

	VectorXd q_init_desired = robot->_q;

	// chair pose
	q_init_desired << 0.454621, -1.599603, 1.473881, -0.171505, -0.235412, 0.327871, 0.210719, -1.404628, 1.601027, -0.508144, -0.008615, -0.098502, 0.336433, -3.321765, -0.447239, 0.059882, -0.036452, 0.422319, 0.770947, 1.522959, -3.299092, -0.266798, -0.112313, -0.106552, 0.390400, -0.107298, -0.506217, -0.787140, 0.192233, 0.269256;
	// q_init_desired *= M_PI/180.0;
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
	chair << 0.5,-1.5,1.4,0.0,0.0,0.0,0.0,-1.4,1.5,-0.5,0.0,0.0,0.0,0.0,0.0,-3.0,0.0,0.0,0.0,0.0,0.0,0.0,-3.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

	VectorXd tree = VectorXd::Zero(dof);
	tree <<  0.0,0.0,0.0,0.0,0.0,1.5,0.0,-0.85,1.9,0.5,0.0,0.0,0.0,0.0,0.0,-3.0,0.1,1.5,-0.6125,0.125,0.0,-1.5,-3.0,-0.1,-1.5,-0.6125,0.125,0.0,1.5,0.0,0.0,0.0;

	VectorXd warrior_1 = VectorXd::Zero(dof);
	warrior_1 << 0.0,-1.3,1.5,0.0,0.0,0.0,0.0,0.6,0.2,-1.0,0.0,0.0,0.0,0.0,0.0,-3.0,0.0,0.0,0.0,0.0,0.0,0.0,-3.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

	VectorXd warrior_2 = VectorXd::Zero(dof);
	warrior_2 << 0.0,-1.3,1.5,0.0,1.5,-1.5,0.0,0.6,0.2,-1.0,0.0,1.0,0.2,0.0,0.0,-1.5,1.5,1.5,0.0,0.0,0.0,0.0,-1.5,-1.5,-1.5,0.0,0.0,0.0,0.0,0.0,0.0,-1.5;

	VectorXd warrior_3 = VectorXd::Zero(dof);
	warrior_3 <<  0.0,-0.2,1.4,0.0,0.0,0.0,0.0,0.5,0.0,1.,0.0,0.0,0.0,0.0,0.0,-3.0,0.0,0.0,0.0,0.0,0.0,0.0,-3.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

	// VectorXd triangle = VectorXd::Zero(dof);
	// triangle <<  -0.65,0.0,0.9,0.0,1.5,-1.5,0.0,0.35,0.0,-0.5,0.0,1.0,0.0,0.0,0.0,-1.5,1.5,1.5,0.0,0.0,0.0,0.0,-1.5,-1.5,-1.5,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

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
		int current_pose = 5;
		if (current_pose == CHAIR_POSE) {
			joint_task->_desired_position = chair;
		} else if (current_pose == TREE){
			joint_task->_desired_position = tree;
		} else if (current_pose == WARRIOR_1){
			joint_task->_desired_position = warrior_1;
		} else if (current_pose == WARRIOR_2){
			joint_task->_desired_position = warrior_2;
		} else if (current_pose == WARRIOR_3){
			joint_task->_desired_position = warrior_3;
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
				// posori_task->reInitializeTask();
				// robot->position(ee_pos, control_link, control_point);
				// posori_task->_desired_position = ee_pos - Vector3d(-0.1, -0.1, 0.1);
				// posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
				// posori_task->_desired_orientation = AngleAxisd(0.0000000000000001, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

				state = MOTION;
			}
		}
		else if (state == MOTION)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			// posori_task->updateTaskModel(N_prec);
			// N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			// posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			// command_torques = posori_task_torques + joint_task_torques;
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
