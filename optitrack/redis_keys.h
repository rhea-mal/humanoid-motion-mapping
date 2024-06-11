/**
 * @file redis_keys.h
 * @author William Chong (williamchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

const std::string PANDA_JOINT_ANGLES_KEY = "sai2::sim::mmp_panda::sensors::q";
const std::string PANDA_JOINT_VELOCITIES_KEY = "sai2::sim::mmp_panda::sensors::dq";
const std::string PANDA_JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::mmp_panda::actuators::fgc";
const std::string PANDA_CONTROLLER_RUNNING_KEY = "sai2::sim::mmp_panda::controller";

const std::string TORO_JOINT_ANGLES_KEY = "sai2::sim::toro::sensors::q";
const std::string TORO_JOINT_VELOCITIES_KEY = "sai2::sim::toro::sensors::dq";
const std::string TORO_JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::toro::actuators::fgc";
const std::string TORO_CONTROLLER_RUNNING_KEY = "sai2::sim::toro::controller";

// Tells simvis to throw another ball
const std::string THROW_BALL_KEY = "test1";
// Tells controller that ball has been hit
const std::string BALL_HIT_KEY = "test2";

const std::string BALL_INIT_POS = "test3";
const std::string BALL_INIT_VEL = "test4";

const std::string POSITION_FROM_CAMERA = "sai2::animation::camera::position";
const std::string RESET_BUTTON = "sai2::animation::reset";
const std::string READY_BUTTON = "sai2::animation::user_ready";
const std::string OPTITRACK_POS_KEY = "sai2::optitrack::pos_rigid_bodies";
const std::string OPTITRACK_ORI_KEY = "sai2::optitrack::ori_rigid_bodies";
const std::string USER_READY_KEY = "sai2::optitrack::user_ready";

const std::string OPTI_POS_PREFIX = "sai2::optitrack::rigid_body_pos::";
const std::string OPTI_ORI_PREFIX = "sai2::optitrack::rigid_body_ori::";
std::vector<std::string> OPTITRACK_POS_KEYS = \
    {OPTI_POS_PREFIX + "1", OPTI_POS_PREFIX + "2", OPTI_POS_PREFIX + "3", OPTI_POS_PREFIX + "4"}
;


const std::string HEAD_POS = "sai2::sim::toro::head_pos";
const std::string HEAD_VERT_AXIS = "sai2::sim::toro::head_vert_axis";
const std::string HEAD_LOOK_AT = "sai2::sim::toro::head_look_at";

const std::string GLOBAL_CAM = "sai2::sim::toro::global_cam";