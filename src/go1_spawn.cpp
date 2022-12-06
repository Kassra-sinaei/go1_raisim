#include "iostream"
#include "vector"
#include "math.h"

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

using namespace std;
void run_sim(int time_steps, double dt, raisim::World world);

int main(int argc, char* argv[]){
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

    
    raisim::World world;
    double time_step = 0.001;
    world.setTimeStep(time_step);
    raisim::RaisimServer server(&world);
    auto ground = world.addGround();
    ground->setAppearance("steel");
    server.launchServer();

    // Spawning the Robot
    auto robot = world.addArticulatedSystem(
        "/home/kassra/PhD-RA/unitree_raisim/robot/Go1.urdf");

    server.focusOn(robot);
    robot->setBasePos(Eigen::Vector3d(0, 0, 0.45));
    Eigen::Matrix3d init_rot;
    init_rot = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1));
    cout << init_rot << endl;
    robot->setBaseOrientation_e(init_rot);
    robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    
    cout << "Object DoF = " << robot->getDOF() << endl;
    cout << "Generalized Coordinate Dimention = " << robot->getGeneralizedCoordinateDim() << endl;
    vector<string> name;
    name = robot->getMovableJointNames();
    for(int i = 0 ; i < name.size(); i++){
        cout << i << "th Joint Name: " << name[i]<< endl;
    }
    //cout << robot->getGeneralizedCoordinate() << endl;
    
    Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
    Eigen::VectorXd jointPgain(robot->getDOF()), jointDgain(robot->getDOF());
    jointPgain.tail(12) << 100.0, 300.0, 300, 100.0, 300.0, 300, 100.0, 300.0, 300, 100.0, 300.0, 300;
    jointDgain.tail(12) << 5.0, 8.0, 8.0, 5.0, 8.0, 8.0, 5.0, 8.0, 8.0, 5.0, 8.0, 8.0;

    sleep(4);
    robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
    robot->setPdGains(jointPgain, jointDgain);
    jointNominalConfig = robot->getGeneralizedCoordinate().e();
    jointVelocityTarget.tail(12).setConstant(0);
    robot->setPdTarget(jointNominalConfig, Eigen::VectorXd::Zero(robot->getDOF()));
    
    // Run Simulation for 100s
    for (int i=0; i<pow(10, 5); i++) {
        //world.integrate();
        //std::this_thread::sleep_for(std::chrono::seconds(size_t(time_step)));
        server.integrateWorldThreadSafe();
        raisim::MSLEEP(1);
    }

    server.killServer();
    // cout << "Build Successful!" << endl;
    return 0;
}
