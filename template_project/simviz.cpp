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
#include <GLFW/glfw3.h>  // must be loaded after loading opengl/glew
#include <signal.h>

bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

#include "redis_keys.h"
#include "../include/object.h"

using namespace std;
using namespace Eigen;

// specify urdf and robots 
const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/stanbot.urdf";
const string robot_name = "stanbot";
const string camera_name = "camera_fixed";

// redis client 
RedisClient redis_client; 

// simulation thread
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	graphics->_world->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 	
	graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes 

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	// robot->_q = VectorXd::Zero(7);
	// robot->_dq = VectorXd::Zero(7);
	robot->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setJointPositions(robot_name, robot->_q);
	sim->setJointVelocities(robot_name, robot->_dq);


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
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "Panda Example", NULL, NULL);
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
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq); 

	// start simulation thread
	thread sim_thread(simulation, robot, sim);

	// initialize glew
	glewInitialize();

	// // while window is open: (OLD VERSION)
	// int count = 0;
	// double bar_width = 2.5;
	// int time_end = 50;
	// int time_inc = 2;
	// Vector3d start_pos = Vector3d(-1*bar_width/2, -2.2, 0.1);
	// Vector3d end_pos = Vector3d(bar_width/2, -2.2, 0.1);
	// double space_inc = (end_pos(0)-start_pos(0))/(time_end/time_inc);

	// while window is open: (NEW VERSION)
	int count = 0;
	double bar_width = 2.3;
	int time_end = 20;
	int num_balls = 20;
	int time_inc = time_end/num_balls;
	Vector3d start_pos = Vector3d(-1*bar_width/2, -2.2, 0.1);
	Vector3d end_pos = Vector3d(bar_width/2, -2.2, 0.1);
	double space_inc = (end_pos(0)-start_pos(0))/(num_balls);
	auto color_grey = new chai3d::cColorf(0.5,0.5,0.5,1);
	auto color_green = new chai3d::cColorf(0,1,0,1);
	auto color_yellow = new chai3d::cColorf(1,1,0,1);
	auto color_red = new chai3d::cColorf(1,0,0,1);
	chai3d::cMesh* ball;
	chai3d::cMesh* balls[num_balls];

	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		if (count == 0) {
			for (int i = 0; i<num_balls; i++) {
				balls[i] = addSphere_tyler(graphics, "greyLight", start_pos, Quaterniond(1,0,0,0),space_inc/2,Vector4d(0.5,0.5,0.5,1));
				start_pos(0) += space_inc;
			}
		}
		if (count % time_inc == 0 & count < time_end) {
			ball = balls[int(count/time_inc)];
			if (count >= time_end/2 & count < 4*time_end/5) {
				ball->m_material->setColor(*color_yellow);
			} else if (count >= 4*time_end/5) {
				ball->m_material->setColor(*color_red);
			} else {
				ball->m_material->setColor(*color_green);
			}
		}
		if (count == time_end + 10) {
			for (int i = 0; i<num_balls; i++) {
				ball = balls[i];
				ball->m_material->setColor(*color_grey);
			}
		}

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot); 
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
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();

		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);
		
		count++;
	}

	// wait for simulation to finish
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------

void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim)
{
	// prepare simulation
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	VectorXd g = VectorXd::Zero(dof);
	string controller_status = "0";
	double kv = 10;  // can be set to 0 if no damping is needed

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addStringToReadCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// add to write callback
	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime();
	double last_time = start_time;

	// start simulation 
	fSimulationRunning = true;	
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// apply gravity compensation 
		robot->gravityVector(g);

		// set joint torques
		if (controller_status == "1") {
			sim->setJointTorques(robot_name, command_torques + g);
		} else {
			sim->setJointTorques(robot_name, g - robot->_M * (kv * robot->_dq));
		}

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		// execute redis write callback
		redis_client.executeWriteCallback(0);		

		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}


//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
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
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
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
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

