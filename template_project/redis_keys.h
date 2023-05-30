/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 * 
 */
const std::string POSE_SELECTION_KEY = "pose::selection";
const std::string INITIALIZED_KEY = "initialized";

const std::string INIT_LEFT_FOOT_POS_KEY = "init::pos::left_foot";
const std::string INIT_RIGHT_HAND_POS_KEY = "init::pos::right_hand";
const std::string INIT_LEFT_HAND_POS_KEY = "init::pos::left_hand";
const std::string INIT_HEAD_POS_KEY = "init::pos::head";
const std::string INIT_CHEST_POS_KEY = "init::pos::chest";

const std::string JOINT_ANGLES_KEY = "sai2::sim::stanbot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::sim::stanbot::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::stanbot::actuators::fgc";
const std::string HUMAN_JOINT_ANGLES_KEY = "sai2::sim::human::sensors::q";
const std::string HUMAN_JOINT_VELOCITIES_KEY = "sai2::sim::human::sensors::dq";
const std::string HUMAN_JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::human::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai2::sim::stanbot::controller_running";

const std::string SIMULATION_LOOP_DONE_KEY = "sai2::project::simulation::done";
const std::string CONTROLLER_LOOP_DONE_KEY = "sai2::project::controller::done";

const std::string HEAD_POS_KEY = "kinect::pos::head";
const std::string HEAD_ORI_KEY = "kinect::ori::head";
const std::string NECK_POS_KEY = "kinect::pos::neck";
const std::string NECK_ORI_KEY = "kinect::ori::neck";

const std::string RIGHT_FOOT_POS_KEY = "kinect::pos::foot_right";
const std::string RIGHT_FOOT_ORI_KEY = "kinect::ori::foot_right";
const std::string LEFT_FOOT_POS_KEY = "kinect::pos::foot_left";
const std::string LEFT_FOOT_ORI_KEY = "kinect::ori::foot_left";
const std::string RIGHT_KNEE_POS_KEY = "kinect::pos::knee_right";
const std::string RIGHT_KNEE_ORI_KEY = "kinect::ori::knee_right";
const std::string LEFT_KNEE_POS_KEY = "kinect::pos::knee_left";
const std::string LEFT_KNEE_ORI_KEY = "kinect::ori::knee_left";

const std::string RIGHT_SHOULDER_POS_KEY = "kinect::pos::shoulder_right";
const std::string RIGHT_SHOULDER_ORI_KEY = "kinect::ori::shoulder_right";
const std::string LEFT_SHOULDER_POS_KEY = "kinect::pos::shoulder_left";
const std::string LEFT_SHOULDER_ORI_KEY = "kinect::ori::shoulder_left";
const std::string RIGHT_HAND_POS_KEY = "kinect::pos::hand_right";
const std::string RIGHT_HAND_ORI_KEY = "kinect::ori::hand_right";
const std::string LEFT_HAND_POS_KEY = "kinect::pos::hand_left";
const std::string LEFT_HAND_ORI_KEY = "kinect::ori::hand_left";

const std::string NAVAL_POS_KEY = "kinect::pos::spine_naval";
const std::string NAVAL_ORI_KEY = "kinect::ori::spine_naval";
const std::string CHEST_POS_KEY = "kinect::pos::spine_chest";
const std::string CHEST_ORI_KEY = "kinect::ori::spine_chest";

