/**
 * @file simviz.cpp
 * @brief Simulation and visualization of panda robot and toro playing volleyball 
 * 
 */

#include <math.h>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <random>

#include "Sai2Graphics.h"
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"

bool fSimulationRunning = true;
void sighandler(int){fSimulationRunning = false;}
// bool camera_tracking = false;
bool testing_without_toro = false;

#include "redis_keys.h"

using namespace Eigen;
using namespace std;

// mutex and globals
VectorXd panda_ui_torques;
VectorXd toro_ui_torques;

mutex mutex_torques, mutex_update;

// specify urdf and robots 
static const string panda_name = "mmp_panda";
static const string toro_name = "toro";

static const string camera_name = "camera_fixed";


// dynamic objects information
const vector<std::string> object_names = {"ball"};
vector<Affine3d> object_poses;
vector<VectorXd> object_velocities;
const int n_objects = object_names.size();

// simulation thread
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

int main() {	
	static const string panda_file = "./resources/model/mmp_panda.urdf";
	static const string toro_file = "./resources/model/toro.urdf";

	static const string world_file = "./resources/world/world_volley.urdf";
	std::cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, camera_name, false);
	graphics->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 	
	// graphics->showLinkFrame(true, robot_name, "link7", 0.15);  // can add frames for different links
	graphics->getCamera(camera_name)->setClippingPlanes(0.1, 2000);  // set the near and far clipping planes 

	//graphics->addUIForceInteraction(panda_name);
	graphics->addUIForceInteraction(toro_name);

	// load robots
	auto panda = std::make_shared<Sai2Model::Sai2Model>(panda_file, false);
	auto toro = std::make_shared<Sai2Model::Sai2Model>(toro_file, false);

	panda->updateModel();
	toro->updateModel();

	panda_ui_torques = VectorXd::Zero(panda->dof());
	toro_ui_torques = VectorXd::Zero(toro->dof());

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);
	sim->setJointPositions(panda_name, panda->q());
	sim->setJointVelocities(panda_name, panda->dq());

	sim->setJointPositions(toro_name, toro->q());
	sim->setJointVelocities(toro_name, toro->dq());



	// fill in object information 
	for (int i = 0; i < n_objects; ++i) {
		object_poses.push_back(sim->getObjectPose(object_names[i]));
		object_velocities.push_back(sim->getObjectVelocity(object_names[i]));
	}

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(1.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// init redis client values 
	redis_client.setEigen(PANDA_JOINT_ANGLES_KEY, panda->q()); 
	redis_client.setEigen(PANDA_JOINT_VELOCITIES_KEY, panda->dq()); 
	redis_client.setEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY, 0 * panda->q());

	redis_client.setEigen(TORO_JOINT_ANGLES_KEY, toro->q()); 
	redis_client.setEigen(TORO_JOINT_VELOCITIES_KEY, toro->dq()); 
	redis_client.setEigen(TORO_JOINT_TORQUES_COMMANDED_KEY, 0 * toro->q());

	redis_client.setBool(THROW_BALL_KEY, false);
	redis_client.setBool(BALL_HIT_KEY, false);
	redis_client.setBool(GLOBAL_CAM, true);

	string link_name = "neck_link2"; // head link
	Eigen::Affine3d transform = toro->transformInWorld(link_name); // p_base = T * p_link
	MatrixXd rot = transform.rotation();
	VectorXd pos = transform.translation();
	VectorXd vert_axis = rot.col(2); // straight up from head (pos z)
	VectorXd lookat = rot.col(0); // straight ahead of head (pos x)

	VectorXd offset(3);
	offset << -2.8, 0.0, -1.1; // x = 1.6
	pos += offset;

	redis_client.setEigen(HEAD_POS, pos);
	redis_client.setEigen(HEAD_VERT_AXIS, vert_axis);
	redis_client.setEigen(HEAD_LOOK_AT, lookat + pos);

	bool conmove = true;

	// start simulation thread
	thread sim_thread(simulation, sim);

	// while window is open:
	while (graphics->isWindowOpen() && fSimulationRunning) {
        graphics->updateRobotGraphics(panda_name, redis_client.getEigen(PANDA_JOINT_ANGLES_KEY));
		graphics->updateRobotGraphics(toro_name, redis_client.getEigen(TORO_JOINT_ANGLES_KEY));
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				graphics->updateObjectGraphics(object_names[i], object_poses[i]);
			}
		}

		if (redis_client.getBool(GLOBAL_CAM) && conmove){
			Eigen::Vector3d head_pos;
			head_pos << 2.0, -0.8, 6.0;
			Eigen::Vector3d head_vert_axis;
			head_vert_axis << 0.0, 0.0, 1.0;
			Eigen::Vector3d head_look_at;
			head_look_at << 0.0, 0.0, 0.0;

			graphics->setCameraPose(camera_name,
									head_pos,
									head_vert_axis,
									head_look_at);
			conmove = false;
		} else if (redis_client.getBool(GLOBAL_CAM) && !conmove) {

		} else {
			graphics->setCameraPose(camera_name,
							redis_client.getEigen(HEAD_POS),
							redis_client.getEigen(HEAD_VERT_AXIS),
							redis_client.getEigen(HEAD_LOOK_AT));
			conmove = true;
		}
		
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			
			panda_ui_torques = graphics->getUITorques(panda_name);
			toro_ui_torques = graphics->getUITorques(toro_name);
		}
	}

    // stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}

bool toro_hit = false;
double time_at_toro_hit = 0;

//------------------------------------------------------------------------------
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	// fSimulationRunning = true;

    // create redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

	// create a timer
	double sim_freq =2000;
	Sai2Common::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);
    sim->enableGravityCompensation(true);

	sim->enableJointLimits(panda_name);
	sim->enableJointLimits(toro_name);

	auto ball = sim->getDynamicWorld()->getBaseNode("ball");
	double prev_ball_z_vel = ball->m_dynamicJoints[2]->getVel();
	bool first_throw = true;

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		VectorXd panda_control_torques = redis_client.getEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY);
		VectorXd toro_control_torques = redis_client.getEigen(TORO_JOINT_TORQUES_COMMANDED_KEY);
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(panda_name, panda_control_torques + panda_ui_torques);
			sim->setJointTorques(toro_name, toro_control_torques + toro_ui_torques);
		}
		sim->integrate();
        redis_client.setEigen(PANDA_JOINT_ANGLES_KEY, sim->getJointPositions(panda_name));
        redis_client.setEigen(PANDA_JOINT_VELOCITIES_KEY, sim->getJointVelocities(panda_name));

		redis_client.setEigen(TORO_JOINT_ANGLES_KEY, sim->getJointPositions(toro_name));
        redis_client.setEigen(TORO_JOINT_VELOCITIES_KEY, sim->getJointVelocities(toro_name));


		// Detect if toro hits ball
		if (!redis_client.getBool(BALL_HIT_KEY) && !redis_client.getBool(THROW_BALL_KEY) && !first_throw){	// If panda is not actively hitting the ball (and not first throw)
			if (prev_ball_z_vel < 0){							// If previous ball velocity is negative (falling)
				if (ball->m_dynamicJoints[2]->getVel() > 0){	// If current ball velocity if positive (ball has been hit)
					time_at_toro_hit = time;					// Record hit time
					toro_hit = true;							// Say ball has been hit
					cout << "toro hit" << endl;
				}
			} 
		}

		if (toro_hit){										// If ball has been hit
			if (time-time_at_toro_hit > 0.5){				// If 0.5 seconds have passed since hitting
				redis_client.setBool(BALL_HIT_KEY, true);	// Tell Panda that ball has been hit
				toro_hit = false;							// Toro hit is no longer active
				cout << "test" << endl;
			}
		}
		
		chai3d::cVector3d om(0.0, 0.0, 0.0);
		// Ball throwing
		if (redis_client.getBool(THROW_BALL_KEY)){

			ball->m_dynamicJoints[0]->setVel(-2.2);
			ball->m_dynamicJoints[1]->setVel(0.0);
			ball->m_dynamicJoints[2]->setVel(4);
			ball->m_dynamicJoints[3]->setVelSpherical(om);

			ball->m_dynamicJoints[0]->setPos(3.0);
			ball->m_dynamicJoints[1]->setPos(0.0);
			ball->m_dynamicJoints[2]->setPos(1.5);

			redis_client.setBool(THROW_BALL_KEY, false);
			first_throw = false;
			cout << "ball thrown" << endl;
		}
		
		redis_client.setEigen(BALL_INIT_POS, Vector3d(ball->m_dynamicJoints[0]->getPos(),
														ball->m_dynamicJoints[1]->getPos(),
														ball->m_dynamicJoints[2]->getPos()));
		redis_client.setEigen(BALL_INIT_VEL, Vector3d(ball->m_dynamicJoints[0]->getVel(),
														ball->m_dynamicJoints[1]->getVel(),
														ball->m_dynamicJoints[2]->getVel()));


		prev_ball_z_vel = ball->m_dynamicJoints[2]->getVel();
	
		// Test without someone controlling toro
		if (ball->m_dynamicJoints[0]->getPos() < -3 && testing_without_toro){

			double x_vel = (rand() % 200 + 100)*0.01;
			double y_vel = (rand() % 300 - 150)*0.01;
			double z_vel = (rand() % 200 + 250)*0.01;

			ball->m_dynamicJoints[0]->setVel(x_vel);
			ball->m_dynamicJoints[1]->setVel(y_vel);
			ball->m_dynamicJoints[2]->setVel(z_vel);
			ball->m_dynamicJoints[3]->setVelSpherical(om);

			ball->m_dynamicJoints[0]->setPos(-2.9);
			ball->m_dynamicJoints[1]->setPos(0.0);
			ball->m_dynamicJoints[2]->setPos(1.5);
		}

		// update object information 
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				object_poses[i] = sim->getObjectPose(object_names[i]);
				object_velocities[i] = sim->getObjectVelocity(object_names[i]);
			}
		}
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}
