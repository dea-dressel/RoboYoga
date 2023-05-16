/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 * 
 */

const std::string JOINT_ANGLES_KEY = "sai2::sim::stanbot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::sim::stanbot::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::stanbot::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai2::sim::stanbot::controller";

