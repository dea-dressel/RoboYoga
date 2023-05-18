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
void sighandler(int){runloop = false;}

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/stanbot.urdf";

enum State 
{
	POSTURE = 0, 
	MOTION
};

int main() {

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

	VectorXd q_init_desired(dof);

	// chair pose
	q_init_desired << 0.454621,-1.599603,1.473881,-0.171505,-0.235412,0.327871,0.210719,-1.404628,1.599603,-0.508144,-0.008615,-0.098502,0.336433,-3.321765,-0.447239,0.059882,-0.036452,0.422319,0.770947,1.522959,-3.299092,-0.266798,-0.112313,-0.106552,0.390400,-0.107298,-0.506217,-0.787140,0.192233,0.269256;
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

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long counter = 0;

	runloop = true;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// get user selected pose
		// int current_pose = redis_client.get("PoseSelection");
		int current_pose = 1;
		// cout << "current pose: " << current_pose << endl;
		

		// update model
		robot->updateModel();
	
		if (state == POSTURE) {
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques;

			if ( (robot->_q - q_init_desired).norm() < 0.15 ) {
				cout << "Posture To Motion" << endl;
				joint_task->reInitializeTask();
				// posori_task->reInitializeTask();
				// robot->position(ee_pos, control_link, control_point);
				// posori_task->_desired_position = ee_pos - Vector3d(-0.1, -0.1, 0.1);
				// posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
				// posori_task->_desired_orientation = AngleAxisd(0.0000000000000001, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

				state = MOTION;
			}
		} else if (state == MOTION) {
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
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
