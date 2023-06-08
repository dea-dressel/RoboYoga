/**
 * @file simviz.cpp
 * @brief Simulation an visualization of panda robot
 *
 */

#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h> // must be loaded after loading opengl/glew
#include <signal.h>
#include "../include/object.h"

bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }
bool fSimulationLoopDone = false;
bool fControllerLoopDone = false;

#include "redis_keys.h"

using namespace std;
using namespace Eigen;

// specify urdf and robots
const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/stanbot.urdf";
const string robot_name = "stanbot";
const string human_file = "./resources/human.urdf";
const string human_name = "human";
const string camera_name = "camera_fixed";
int count_init = 0;
bool init = false;

// redis client
RedisClient redis_client;
RedisClient redis_client2;
RedisClient redis_client_graphics;

// simulation thread
void simulation(Sai2Model::Sai2Model *robot, Sai2Model::Sai2Model *human, Simulation::Sai2Simulation *sim);

// function for converting string to bool
bool string_to_bool(const std::string &x);

// function for converting bool to string
inline const char *const bool_to_string(bool b);

// callback to print glfw errors
void glfwError(int error, const char *description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow *window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow *window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;

// containers to average init positions
vector<Vector3d> init_pos(6, Vector3d::Zero());
int n_init_samples = 50;

int main()
{
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	redis_client_graphics = RedisClient();
	redis_client_graphics.connect();

	redis_client2 = RedisClient();
	redis_client2.connect("192.168.1.70");

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	graphics->_world->setBackgroundColor(66.0 / 255, 135.0 / 255, 245.0 / 255); // set blue background
	graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);				// set the near and far clipping planes
	// graphics->showLinkFrame("chest", "human");

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	auto human = new Sai2Model::Sai2Model(human_file, false);
	human->_q(1) = M_PI / 16;
	human->_q(2) = -M_PI / 8;
	human->_q(9) = M_PI / 8;
	human->_q(3) = M_PI / 16;
	human->_q(8) = -M_PI / 16;
	human->_q(10) = -M_PI / 16;
	human->_q(17) = M_PI / 32;
	human->_q(19) = -M_PI / 16;
	human->_q(24) = -M_PI / 32;
	human->_q(26) = -M_PI / 16;

	// human->_q(0) = M_PI / 16;
	// human->_q(1) = -M_PI / 8;
	// human->_q(8) = M_PI / 8;
	// human->_q(2) = M_PI / 16;
	// human->_q(7) = -M_PI / 16;
	// human->_q(9) = -M_PI / 16;
	// human->_q(16) = M_PI / 32;
	// human->_q(18) = -M_PI / 16;
	// human->_q(23) = -M_PI / 32;
	// human->_q(25) = -M_PI / 16;

	robot->updateModel();
	human->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setJointPositions(robot_name, robot->_q);
	sim->setJointVelocities(robot_name, robot->_dq);

	sim->setJointPositions(human_name, human->_q);
	sim->setJointVelocities(human_name, human->_dq);

	// set co-efficient of restition to zero for force control
	sim->setCollisionRestitution(0.0);

	// set co-efficient of friction
	sim->setCoeffFrictionStatic(0.0);
	sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor *primary = glfwGetPrimaryMonitor();
	const GLFWvidmode *mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow *window = glfwCreateWindow(windowW, windowH, "Robo Yoga", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;

	// init redis client values
	redis_client.set(CONTROLLER_RUNNING_KEY, "0");
	redis_client.set(INITIALIZED_KEY, bool_to_string(false));
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.setEigenMatrixJSON(HUMAN_JOINT_ANGLES_KEY, human->_q);
	redis_client.setEigenMatrixJSON(HUMAN_JOINT_VELOCITIES_KEY, human->_dq);
	redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));
	redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));

	// start simulation thread
	thread sim_thread(simulation, robot, human, sim);

	// initialize glew
	glewInitialize();

	// while window is open:
	int count = 0;
	double bar_width = 2.3;
	int time_end = 20;
	int num_balls = 20;
	int time_inc = time_end / num_balls;
	Vector3d start_pos = Vector3d(-1 * bar_width / 2, -2.2, 0.1);
	Vector3d end_pos = Vector3d(bar_width / 2, -2.2, 0.1);
	double space_inc = (end_pos(0) - start_pos(0)) / (num_balls);
	auto color_grey = new chai3d::cColorf(0.5, 0.5, 0.5, 1);
	auto color_green = new chai3d::cColorf(0, 1, 0, 1);
	auto color_yellow = new chai3d::cColorf(1, 1, 0, 1);
	auto color_red = new chai3d::cColorf(1, 0, 0, 1);
	chai3d::cMesh *ball;
	chai3d::cMesh *balls[num_balls];

	int progress_counter;
	bool show_progress_bar = false;

	for (int i = 0; i < num_balls; i++)
	{
		balls[i] = addSphere_tyler(graphics, "greyLight", start_pos, Quaterniond(1, 0, 0, 0), space_inc / 2, Vector4d(0.5, 0.5, 0.5, 1));
		start_pos(0) += space_inc;
	}
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		if (count == time_end + 10)
		{
			for (int i = 0; i < num_balls; i++)
			{
				ball = balls[i];
				ball->m_material->setColor(*color_grey);
			}
		}
		if (progress_counter % time_inc == 0 & progress_counter < time_end)
		{
			ball = balls[int(progress_counter / time_inc)];
			if (progress_counter >= time_end / 2 & progress_counter < 4 * time_end / 5)
			{
				ball->m_material->setColor(*color_yellow);
			}
			else if (progress_counter >= 4 * time_end / 5)
			{
				ball->m_material->setColor(*color_red);
			}
			else
			{
				ball->m_material->setColor(*color_green);
			}
		}
		

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->updateGraphics(human_name, human);
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; // TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();

		if (fTransXp)
		{
			camera_pos = camera_pos + 0.05 * cam_roll_axis;
			camera_lookat = camera_lookat + 0.05 * cam_roll_axis;
		}
		if (fTransXn)
		{
			camera_pos = camera_pos - 0.05 * cam_roll_axis;
			camera_lookat = camera_lookat - 0.05 * cam_roll_axis;
		}
		if (fTransYp)
		{
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05 * cam_up_axis;
			camera_lookat = camera_lookat + 0.05 * cam_up_axis;
		}
		if (fTransYn)
		{
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05 * cam_up_axis;
			camera_lookat = camera_lookat - 0.05 * cam_up_axis;
		}
		if (fTransZp)
		{
			camera_pos = camera_pos + 0.1 * cam_depth_axis;
			camera_lookat = camera_lookat + 0.1 * cam_depth_axis;
		}
		if (fTransZn)
		{
			camera_pos = camera_pos - 0.1 * cam_depth_axis;
			camera_lookat = camera_lookat - 0.1 * cam_depth_axis;
		}
		if (fRotPanTilt)
		{
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			// TODO: might need to re-scale from screen units to physical units
			double compass = 0.006 * (cursorx - last_cursorx);
			double azimuth = 0.006 * (cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt;
			m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt * (camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan;
			m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan * (camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);

		if (count_init == n_init_samples && init == true)
		{
			redis_client_graphics.setEigenMatrixJSON(INIT_LEFT_FOOT_POS_KEY, init_pos[0]);
			redis_client_graphics.setEigenMatrixJSON(INIT_RIGHT_HAND_POS_KEY, init_pos[1]);
			redis_client_graphics.setEigenMatrixJSON(INIT_LEFT_HAND_POS_KEY, init_pos[2]);
			redis_client_graphics.setEigenMatrixJSON(INIT_HEAD_POS_KEY, init_pos[3]);
			redis_client_graphics.setEigenMatrixJSON(INIT_CHEST_POS_KEY, init_pos[4]);
			redis_client_graphics.setEigenMatrixJSON(INIT_PELVIS_POS_KEY, init_pos[5]);
			redis_client_graphics.setEigenMatrixJSON(INIT_LEFT_FOOT_ORI_KEY, redis_client2.getEigenMatrixJSON(LEFT_FOOT_ORI_KEY));
			redis_client_graphics.setEigenMatrixJSON(INIT_RIGHT_HAND_ORI_KEY, redis_client2.getEigenMatrixJSON(RIGHT_HAND_ORI_KEY));
			redis_client_graphics.setEigenMatrixJSON(INIT_LEFT_HAND_ORI_KEY, redis_client2.getEigenMatrixJSON(LEFT_HAND_ORI_KEY));
			redis_client_graphics.setEigenMatrixJSON(INIT_HEAD_ORI_KEY, redis_client2.getEigenMatrixJSON(HEAD_ORI_KEY));
			redis_client_graphics.setEigenMatrixJSON(INIT_CHEST_ORI_KEY, redis_client2.getEigenMatrixJSON(CHEST_ORI_KEY));
			redis_client_graphics.setEigenMatrixJSON(INIT_PELVIS_ORI_KEY, redis_client2.getEigenMatrixJSON(PELVIS_ORI_KEY));
			redis_client_graphics.set(INITIALIZED_KEY, bool_to_string(true));
			redis_client_graphics.set(INITIALIZED_KEY, bool_to_string(true));
			progress_counter = 0;
			cout << "initialized";
			show_progress_bar = true;
			init = false;
		}
		else if (init)
		{
			init_pos[0] += (1. / n_init_samples) * redis_client2.getEigenMatrixJSON(LEFT_FOOT_POS_KEY);
			init_pos[1] += (1. / n_init_samples) * redis_client2.getEigenMatrixJSON(RIGHT_HAND_POS_KEY);
			init_pos[2] += (1. / n_init_samples) * redis_client2.getEigenMatrixJSON(LEFT_HAND_POS_KEY);
			init_pos[3] += (1. / n_init_samples) * redis_client2.getEigenMatrixJSON(HEAD_POS_KEY);
			init_pos[4] += (1. / n_init_samples) * redis_client2.getEigenMatrixJSON(CHEST_POS_KEY);
			init_pos[5] += (1. / n_init_samples) * redis_client2.getEigenMatrixJSON(PELVIS_POS_KEY);

			for (auto val : init_pos)
			{
				cout << val.transpose() << "\n";
			}
			cout << endl;
		}
		// cout << count_init << "\n";

		if (init)
			count_init++;
		if (show_progress_bar)
			progress_counter ++;

		count++;
	}

	// wait for simulation to finish
	fSimulationRunning = false;
	fSimulationLoopDone = false;
	redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------

void simulation(Sai2Model::Sai2Model *robot, Sai2Model::Sai2Model *human, Simulation::Sai2Simulation *sim)
{
	// prepare simulation
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	VectorXd g = VectorXd::Zero(dof);

	int human_dof = human->dof();
	VectorXd human_command_torques = VectorXd::Zero(human_dof);
	redis_client.setEigenMatrixJSON(HUMAN_JOINT_TORQUES_COMMANDED_KEY, human_command_torques);
	VectorXd human_g = VectorXd::Zero(human_dof);

	string controller_status = "0";
	double kv = 10; // can be set to 0 if no damping is needed

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addStringToReadCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.addEigenToReadCallback(0, HUMAN_JOINT_TORQUES_COMMANDED_KEY, human_command_torques);

	// add to write callback
	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToWriteCallback(0, HUMAN_JOINT_ANGLES_KEY, human->_q);
	redis_client.addEigenToWriteCallback(0, HUMAN_JOINT_VELOCITIES_KEY, human->_dq);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime();
	double last_time = start_time;

	// start simulation
	fSimulationRunning = true;
	while (fSimulationRunning)
	{
		// fTimerDidSleep = timer.waitForNextLoop(); // commented out to let current simulation loop finish before next loop

		// run simulation loop when control loop is done
		if (fControllerLoopDone)
		{
			// execute redis read callback
			redis_client.executeReadCallback(0);

			// apply gravity compensation
			robot->gravityVector(g);
			human->gravityVector(human_g);

			// set joint torques
			if (controller_status == "1")
			{
				sim->setJointTorques(robot_name, command_torques - robot->_M * (kv * robot->_dq));
				sim->setJointTorques(human_name, human_command_torques - human->_M * (kv * human->_dq));
			}
			else
			{
				sim->setJointTorques(robot_name, -robot->_M * (kv * robot->_dq));
				sim->setJointTorques(human_name, -human->_M * (kv * human->_dq));
			}

			// for (auto robot: sim->_world->m_dynamicObjects){
			// 	if(robot->m_name == "stanbot"){
			// 		robot->enableDynamics(false);
			// 	}
			// }

			// integrate forward
			double curr_time = timer.elapsedTime();
			double loop_dt = curr_time - last_time;
			// sim->integrate(loop_dt);
			sim->integrate(0.001);

			// read joint positions, velocities, update model
			sim->getJointPositions(robot_name, robot->_q);
			sim->getJointVelocities(robot_name, robot->_dq);
			robot->updateModel();

			sim->getJointPositions(human_name, human->_q);
			sim->getJointVelocities(human_name, human->_dq);
			human->updateModel();

			// simulation loop is done
			fSimulationLoopDone = true;

			// ask for next control loop
			fControllerLoopDone = false;

			// execute redis write callback
			redis_client.executeWriteCallback(0);
			redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));
			// redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));

			// update last time
			last_time = curr_time;
		}

		// read controller state
		fControllerLoopDone = string_to_bool(redis_client.get(CONTROLLER_LOOP_DONE_KEY));
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles() / end_time << "Hz\n";
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

//------------------------------------------------------------------------------

void glfwError(int error, const char *description)
{
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize()
{
	bool ret = false;
#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK)
	{
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	}
	else
	{
		ret = true;
	}
#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch (key)
	{
	case GLFW_KEY_ESCAPE:
		// exit application
		fSimulationRunning = false;
		glfwSetWindowShouldClose(window, GL_TRUE);
		break;
	case GLFW_KEY_RIGHT:
		fTransXp = set;
		break;
	case GLFW_KEY_LEFT:
		fTransXn = set;
		break;
	case GLFW_KEY_UP:
		fTransYp = set;
		break;
	case GLFW_KEY_DOWN:
		fTransYn = set;
		break;
	case GLFW_KEY_A:
		fTransZp = set;
		break;
	case GLFW_KEY_Z:
		fTransZn = set;
		break;
	case GLFW_KEY_I:
		count_init = 0;
		init = true;
		break;
	default:
		break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow *window, int button, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	// TODO: mouse interaction with robot
	switch (button)
	{
	// left click pans and tilts
	case GLFW_MOUSE_BUTTON_LEFT:
		fRotPanTilt = set;
		// NOTE: the code below is recommended but doesn't work well
		// if (fRotPanTilt) {
		// 	// lock cursor
		// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		// } else {
		// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		// }
		break;
	// if right click: don't handle. this is for menu selection
	case GLFW_MOUSE_BUTTON_RIGHT:
		// fRobotLinkSelect = set;
		break;
	// if middle click: don't handle. doesn't work well on laptops
	case GLFW_MOUSE_BUTTON_MIDDLE:
		break;
	default:
		break;
	}
}
