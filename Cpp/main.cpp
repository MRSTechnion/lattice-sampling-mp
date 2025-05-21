#define FMT_HEADER_ONLY

#pragma region includes

/* ompl includes */

// general
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/samplers/deterministic/PrecomputedSequence.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/config.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/tools/benchmark/Benchmark.h>

// planners
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>

/* omplapp includes */

// #include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <omplapp/apps/SE2MultiRigidBodyPlanning.h>
#include <omplapp/apps/SE2RigidBodyPlanning.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/geometry/RigidBodyGeometry.h>
#include <omplapp/config.h>
#include <sys/stat.h>

/* general includes */

#include <limits>
#include <regex>
#include <iostream>
#include <tuple>
#include <fstream>
#include <utility>
#include <fmt/core.h>
// exception catching
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <filesystem>

/* my includes */

#include "include/SampleLimitTerminationCondition.h"
#include "include/FullPRM.h"
#include "include/LatticePRM.h"
#include "include/LazyFullPRM.h"
#include "include/ImplicitRandomPRM.h"
// #include "include/DeterministicPRM.h"
// #include "include/Lattices.h"
#include <ompl/base/GenericParam.h>
// #include <float.h>

// #include "include/DeterministicSampler.h"

#include "include/ImplicitPRM.h"
#include "include/Lattices.h"
// #include "include/Lattices.h"
#include "include/VAMPTests.h"

using LatticeType = ompl::geometric::ImplicitPRM::LatticeType;

/* namespaces */
namespace ob = ompl::base;
namespace og = ompl::geometric;

#pragma endregion includes

#pragma region general_use

/// maximize clearance
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }

    ob::Cost stateCost(const ob::State* s) const
    {
        // return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};

/// create an SO(3) object using 3 angles
void AngleToQuaternion(double u1, double u2, double u3, ompl::base::SE3StateSpace::StateType* state) {
    state->rotation().w = std::cos(0.5*u1) * std::cos(0.5*u2) * std::cos(0.5*u3) -
    std::sin(0.5*u1) * std::sin(0.5*u2) * std::sin(0.5*u3);
    state->rotation().x = std::sin(0.5*u1) * std::cos(0.5*u2) * std::cos(0.5*u3) +
        std::cos(0.5*u1) * std::sin(0.5*u2) * std::sin(0.5*u3);
    state->rotation().y = std::cos(0.5*u1) * std::sin(0.5*u2) * std::cos(0.5*u3) -
        std::sin(0.5*u1) * std::cos(0.5*u2) * std::sin(0.5*u3);
    state->rotation().z = std::cos(0.5*u1) * std::cos(0.5*u2) * std::sin(0.5*u3) +
        std::sin(0.5*u1) * std::sin(0.5*u2) * std::cos(0.5*u3);
}

/// extract the R2 coords from a state
std::tuple<float, float> getR2Coords(const ob::State *state) {
    const ob::CompoundState* R2_state = state->as<ob::CompoundState>();
    double x = R2_state->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double y = R2_state->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
    return {x, y};
}

double GetMedian(const std::vector<double>& vals) {
    int N = vals.size();
    std::vector<double> sortedVals(vals);
    std::sort(sortedVals.begin(), sortedVals.end());
    if (vals.empty()) return -1;
    if (N % 2 == 1) {
        return sortedVals[(N/2)];
    }
    return (sortedVals[(N/2) - 1] + sortedVals[(N/2)]) / 2.0;
}

template<typename T>
double GetAverage(const std::vector<T> & vals) {
    auto size = vals.size();
    auto sum = accumulate(vals.begin(), vals.end(), 0.0);
    return static_cast<double>(sum) / static_cast<double>(size);
}

#pragma endregion general_use

#pragma region unused_planners

// /// A function that matches the ompl::base::PlannerAllocator type,
// /// to be used later to allocate an instance of EST.
// /// This is part of the BENCHMARKING example code.
// ompl::base::PlannerPtr myConfiguredPlanner(const ompl::base::SpaceInformationPtr &si)
// {
//     og::DeterministicPRM *detSampler = new og::DeterministicPRM(si);
//     return ompl::base::PlannerPtr(detSampler);
// }
//
// void Benchmark() {
//     ob::StateSpacePtr X(new ob::RealVectorStateSpace(1));
//     auto X_RVecSp = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(X);
//     // set the bounds for the space
//     ob::RealVectorBounds bounds(1);
//     bounds.setLow(0);
//     bounds.setHigh(1);
//     X_RVecSp->setBounds(bounds);
//     ob::StateSpacePtr Y(new ob::RealVectorStateSpace(1));
//     auto Y_RVecSp = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(Y);
//     Y_RVecSp->setBounds(bounds);
//     auto space = X + Y;
//     // construct an instance of  space information from this state space
//     // auto si(std::make_shared<ob::SpaceInformation>(space));
//     // Create a state space for the space we are planning in
//     ompl::geometric::SimpleSetup ss(space);
//
//     // First we create a benchmark class:
//     ompl::tools::Benchmark b(ss, "my experiment");
//
//     // Optionally, specify some benchmark parameters (doesn't change how the benchmark is run)
//     b.addExperimentParameter("num_dofs", "INTEGER", "2");
//     b.addExperimentParameter("num_obstacles", "INTEGER", "10");
//
//     // We add the planners to evaluate.
//     b.addPlanner(ob::PlannerPtr(new og::PRM(ss.getSpaceInformation())));
//     // etc
//
//     // For planners that we want to configure in specific ways,
//     // the ompl::base::PlannerAllocator should be used:
//     b.addPlannerAllocator(std::bind(&myConfiguredPlanner, std::placeholders::_1));
//     // etc.
//
//     // Now we can benchmark: 5 second time limit for each plan computation,
//     // 100 MB maximum memory usage per plan computation, 50 runs for each planner
//     // and true means that a text-mode progress bar should be displayed while
//     // computation is running.
//     ompl::tools::Benchmark::Request req;
//     req.maxTime = 5.0;
//     req.maxMem = 100.0;
//     req.runCount = 50;
//     req.displayProgress = true;
//     b.benchmark(req);
//
//     // This will generate a file of the form ompl_host_time.log
//     b.saveResultsToFile();
// }
//
// /// EXAMPLE: multiple disks, on 3D maps, using LatticePRM
// std::tuple<int, double> plan_lazy_det_with_maze_3d(LatticeType lattice) {
//     // // construct the state space we are planning in
//     // ob::StateSpacePtr X(new ob::RealVectorStateSpace(1));
//     // auto X_RVecSp = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(X);
//     // // set the bounds for the space
//     // ob::RealVectorBounds bounds(1);
//     // bounds.setLow(0);
//     // bounds.setHigh(1);
//     // X_RVecSp->setBounds(bounds);
//     // ob::StateSpacePtr Y(new ob::RealVectorStateSpace(1));
//     // auto Y_RVecSp = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(Y);
//     // Y_RVecSp->setBounds(bounds);
//     // auto space = X + Y;
//     // // construct an instance of  space information from this state space
//     // auto si(std::make_shared<ob::SpaceInformation>(space));
//
//     // plan in SE2
//
//     ompl::app::SE3RigidBodyPlanning setup;
//     // load the robot and the environment
//     // std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/spirelli_robot.dae";
//     // std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/spirelli_env.dae";
//     std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
//     std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
//     // std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Easy_robot.dae";
//     // std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Easy_env.dae";
//     // std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
//
//     setup.setRobotMesh(robot_fname);
//     setup.setEnvironmentMesh(env_fname);
//     // infer bounds from the environment we added
//     setup.inferEnvironmentBounds();
//
//     auto si(setup.getSpaceInformation());
//     auto svc(setup.allocStateValidityChecker(si, setup.getGeometricStateExtractor(), false));
//
//     si->setStateValidityChecker(svc);
//     // set state validity checking for this space
//     // si->setStateValidityChecker(isStateValid);
//     // get the samples
//     // si->setValidStateSamplerAllocator(allocValidDeterministicSampler);
//     // set edge validator
//     si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
//     si->setup();
//
//     // create a planner for the defined space
//     auto planner(std::make_shared<og::LatticePRM>(si));
//     // spirelli
//     // start->setX(64.59); // randompoly 1
//     // start->setY(22.21); // randompoly 1
//     // start->setZ(1.84);
//     // planner->changeAngleToQuaternion(0, -EIGEN_PI / 2.0, 0, start.get());
//     // Eigen::VectorXd startEigen(6);
//     // startEigen << 64.59, 22.21, 1.84, 0, -EIGEN_PI / 2.0, 0; // delta = 2
//
//     // twistycool
//     // start->setX(270.0); // randompoly 1
//     // start->setY(160.0); // randompoly 1
//     // start->setZ(-250.0);
//     // planner->changeAngleToQuaternion(0, 0, 0, start.get());
//     // Eigen::VectorXd startEigen(6);
//     // startEigen << 270.0, 160.0, -250.0, 0, 0, 0; // delta = 2
//
//     // define goal state //
//     //spirelli
//     // ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(start);
//     // goal->setX(2.59); // randompoly 1
//     // goal->setY(23.21); // randompoly 1
//     // goal->setZ(-32.16);
//     // planner->changeAngleToQuaternion(0, 0, EIGEN_PI, goal.get());
//     // Eigen::VectorXd goalEigen(6);
//     // goalEigen << 64.59, 22.21, 1.84, 0, 0, EIGEN_PI; // delta = 2
//
//     // easybot + smaller bbox + easy
//     // start
//     // double x_start = 270.0;
//     // double y_start = 160.0;
//     // double z_start = -250.0;
//     // double u1_start = 0.0;
//     // double u2_start = 0.0;
//     // double u3_start = 0.0;
//     // // goal
//     // double x_goal = 270.0;
//     // double y_goal = 160.0;
//     // double z_goal = -350.0;
//     // double u1_goal = 0.0;
//     // double u2_goal = 0.0;
//     // double u3_goal = 0;
//
//     // quad + smaller bbox + easwy : delta = 40, angle = 200
//     // start
//     // double x_start = 265.0;
//     // double y_start = 200.0;
//     // double z_start = -250.0;
//     // double u1_start = 0.0;
//     // double u2_start = 0.0;
//     // double u3_start = 0.0;
//     // goal
//     // double x_goal = 265.0;
//     // double y_goal = 200.0;
//     // double z_goal = -350.0;
//     // double u1_goal = 0.0;
//     // double u2_goal = 0.0;
//     // double u3_goal = 0.0;
//
//     // cubicle_bot + smaller bbox + easy : delta = 35, angle = 60
//     // start
//     // double x_start = 265.0;
//     // double y_start = 200.0;
//     // double z_start = -250.0;
//     // double u1_start = 0.0;
//     // double u2_start = 0.0;
//     // double u3_start = 0.0;
//     // // goal
//     // double x_goal = 265.0;
//     // double y_goal = 200.0;
//     // double z_goal = -350.0;
//     // double u1_goal = 0.0;
//     // double u2_goal = 0.0;
//     // double u3_goal = 0.0;
//
//     // Twistycool_robot + smaller bbox + Twistycool: delta = 24, angle = 60
//     // start
//     double x_start = 270.0;
//     double y_start = 160.0;
//     double z_start = -200.0;
//     double u1_start = 0.0;
//     double u2_start = 0.0;
//     double u3_start = 0.0;
//     // goal
//     double x_goal = 270.0;
//     double y_goal = 160.0;
//     double z_goal = -400.0;
//     double u1_goal = 0.0;
//     double u2_goal = 0.0;
//     double u3_goal = 0.0;
//
//     ompl::base::ScopedState<ompl::base::SE3StateSpace> start(si);
//     Eigen::VectorXd startEigen(6);
//     start->setX(x_start); // randompoly 1
//     start->setY(y_start); // randompoly 1
//     start->setZ(z_start);
//     planner->changeAngleToQuaternion(u1_start, u2_start, u3_start, start.get());
//     startEigen << x_start, y_start, z_start, u1_start, u2_start, u3_start; // delta = 2
//     ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(start);
//     Eigen::VectorXd goalEigen(6);
//     goal->setX(x_goal); // randompoly 1
//     goal->setY(y_goal); // randompoly 1
//     goal->setZ(z_goal);
//     planner->changeAngleToQuaternion(u1_goal, u2_goal, u3_goal, goal.get());
//     goalEigen << x_goal, y_goal, z_goal, u1_goal, u2_goal, u3_goal; // delta = 2
//
//     // from rrt*
//     double delta = 24.0;
//     double angle = 60;
//     // easy map bounds
//     std::vector<double> low = {175, 75, -400, -angle, -angle, -angle};
//     std::vector<double> high = {350, 250, -190, angle, angle, angle};
//     ob::RealVectorBounds customBounds(6);
//     for (int i = 0; i < 6; ++i) {
//         customBounds.setLow(i, low[i]);
//         customBounds.setHigh(i, high[i]);
//     }
//     planner->setBounds(customBounds);
//     planner->setAngleFix(EIGEN_PI / angle);
//     // create a problem instance
//     auto pdef(std::make_shared<ob::ProblemDefinition>(si));
//
//     // set the start and goal states
//     pdef->setStartAndGoalStates(start, goal);
//
//
//     // set the problem we are trying to solve for the planner
//     planner->setProblemDefinition(pdef);
//
//     // set the lattice type
//     // planner->setDeterministicType(File,
//         // "/home/itai/ompl3/ompl-1.6.0/tests/resources/halton/halton_2d.txt");
//     // planner->setDeterministicType(Zn, 3);
//     // planner->setLatticeType(lattice, 2.2, 1); // uniquemaze
//     // planner->setLatticeType(lattice, 2, 1); // randompoly1
//     // planner->setLatticeType(lattice, 3.5, 1); // randompoly2
//     planner->setLatticeType(lattice, delta, 10); // 3D-easy
//     planner->setStartAndGoalEigen(startEigen, goalEigen);
//     // perform setup steps for the planner
//     planner->setup();
//
//
//     // print the settings for this space
//     // si->printSettings(std::cout);
//     // return;
//     // print the problem settings
//     pdef->print(std::cout);
//
//     // auto sptc = Lattices::SampleLimitTerminationCondition(nullptr);
//     // attempt to solve the problem within one second of planning time
//     ob::PlannerStatus solved = planner->ob::Planner::solve(10000.0);
//     std::cout << solved << std::endl;
//     if (solved) {
//         // get the goal representation from the problem definition (not the same as the goal state)
//         // and inquire about the found path
//         ob::PathPtr path = pdef->getSolutionPath();
//         std::cout << "Found solution:" << std::endl;
//         double length =path->length();
//         auto pathGeo = path->as<ompl::geometric::PathGeometric>();
//         // for (auto v: path2) {
//             // std::cout << v << std::endl;
//         // }
//         // print the path to screen
//         path->print(std::cout);
//         std::ofstream outfile;
//         outfile.open("testpath.path");
//         pathGeo->interpolate(200);
//         pathGeo->printAsMatrix(outfile);
//         outfile.close();
//         ompl::tools::SelfConfig config = ompl::tools::SelfConfig(si);
//         outfile.open("testpath.cfg");
//         config.print(outfile);
//         outfile.close();
//         // get the path file
//         // planner->getSolutionPath().asGeometric().printAsMatrix(std::cout);
//         return {planner->getStateCount(), path->length()};
//     } else
//         std::cout << "No solution found" << std::endl;
//     return {-1, -1};
// }
//
// /// single disk, on 3D maps, using RRT/RRT* (random sampling)
// std::tuple<int, double> plan_with_maze_3d() {
//     // // construct the state space we are planning in
//     // ob::StateSpacePtr X(new ob::RealVectorStateSpace(1));
//     // auto X_RVecSp = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(X);
//     // // set the bounds for the space
//     // ob::RealVectorBounds bounds(1);
//     // bounds.setLow(0);
//     // bounds.setHigh(1);
//     // X_RVecSp->setBounds(bounds);
//     // ob::StateSpacePtr Y(new ob::RealVectorStateSpace(1));
//     // auto Y_RVecSp = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(Y);
//     // Y_RVecSp->setBounds(bounds);
//     // auto space = X + Y;
//     // // construct an instance of  space information from this state space
//     // auto si(std::make_shared<ob::SpaceInformation>(space));
//
//     // plan in SE2
//
//     ompl::app::SE3RigidBodyPlanning setup;
//     // load the robot and the environment
//     // std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/spirelli_robot.dae";
//     // std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/spirelli_env.dae";
//     // std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
//     // std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
//     // std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Easy_robot.dae";
//     std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
//     std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Easy_env.dae";
//     // std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/quadrotor.dae";
//     // std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Easy_env.dae";
//
//
//     setup.setRobotMesh(robot_fname);
//     setup.setEnvironmentMesh(env_fname);
//     // infer bounds from the environment we added
//     setup.inferEnvironmentBounds();
//
//     auto si(setup.getSpaceInformation());
//     auto svc(setup.allocStateValidityChecker(si, setup.getGeometricStateExtractor(), false));
//
//     si->setStateValidityChecker(svc);
//     // set state validity checking for this space
//     // si->setStateValidityChecker(isStateValid);
//     // get the samples
//     // si->setValidStateSamplerAllocator(allocValidDeterministicSampler);
//     // set edge validator
//     si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
//     si->setup();
//
//     // create a planner for the defined space
//     auto planner(std::make_shared<og::RRTstar>(si));
//
//     ompl::base::ScopedState<ompl::base::SE3StateSpace> start(si);
//     // spirelli
//     // start->setX(64.59); // randompoly 1
//     // start->setY(22.21); // randompoly 1
//     // start->setZ(1.84);
//     // planner->changeAngleToQuaternion(0, -EIGEN_PI / 2.0, 0, start.get());
//     // Eigen::VectorXd startEigen(6);
//     // startEigen << 64.59, 22.21, 1.84, 0, -EIGEN_PI / 2.0, 0; // delta = 2
//
//     // twistycool
//     start->setX(270.0); // randompoly 1
//     start->setY(160.0); // randompoly 1
//     start->setZ(-250.0);
//     start->rotation().setIdentity();
//     // planner->changeAngleToQuaternion(0, 0, 0, start.get());
//     // Eigen::VectorXd startEigen(6);
//     // startEigen << 270.0, 160.0, -200.0, 0, 0, 0; // delta = 2
//
//     // define goal state //
//     //spirelli
//     ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(start);
//     // goal->setX(2.59); // randompoly 1
//     // goal->setY(23.21); // randompoly 1
//     // goal->setZ(-32.16);
//     // planner->changeAngleToQuaternion(0, 0, EIGEN_PI, goal.get());
//     // Eigen::VectorXd goalEigen(6);
//     // goalEigen << 64.59, 22.21, 1.84, 0, 0, EIGEN_PI; // delta = 2
//
//     // twistycool
//     goal->setX(270.0); // randompoly 1
//     goal->setY(160.0); // randompoly 1
//     goal->setZ(-400.0);
//     goal->rotation().setIdentity();
//     // planner->changeAngleToQuaternion(0, 0, 0, goal.get());
//     // Eigen::VectorXd goalEigen(6);
//     // goalEigen << 270.0, 160.0, -200.0, 0, 0, 0; // delta = 2
//
//     // create a problem instance
//     auto pdef(std::make_shared<ob::ProblemDefinition>(si));
//
//     // set the start and goal states
//     pdef->setStartAndGoalStates(start, goal);
//
//
//     // set the problem we are trying to solve for the planner
//     planner->setProblemDefinition(pdef);
//
//     // set the lattice type
//     // planner->setDeterministicType(File,
//         // "/home/itai/ompl3/ompl-1.6.0/tests/resources/halton/halton_2d.txt");
//     // planner->setDeterministicType(Zn, 3);
//     // planner->setLatticeType(lattice, 2.2, 1); // uniquemaze
//     // planner->setLatticeType(lattice, 2, 1); // randompoly1
//     // planner->setLatticeType(lattice, 3.5, 1); // randompoly2
//     // planner->setStartAndGoalEigen(startEigen, goalEigen);
//     pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ClearanceObjective(si)));
//
//     // perform setup steps for the planner
//     planner->setup();
//     std::cout << planner->getGoalBias() << std::endl;
//      // auto sptc = Lattices::SampleLimitTerminationCondition(nullptr);
//     // attempt to solve the problem within one second of planning time
//     // print the path to screen
//     std::stringstream ss;
//     // pdef->print(ss);
//     // std::string myString = ss.str();
//     std::string line;
//
//     std::streambuf* org = std::cout.rdbuf(); // Remember std::cout's old state
//     std::cout.rdbuf(ss.rdbuf()); // Bind it to the output file stream
//
//     ob::PlannerStatus solved = planner->ob::Planner::solve(300.0);
//
//     std::cout.rdbuf(org); // Reset std::cout's old state
//
//     if (solved) {
//         // get the goal representation from the problem definition (not the same as the goal state)
//         // and inquire about the found path
//         ob::PathPtr path = pdef->getSolutionPath();
//         double length =path->length();
//         std::cout << "Found solution:" << std::endl;
//         double clear = path->as<og::PathGeometric>()->clearance();
//         fmt::print("clearance={}", clear);
//         // print the path to screen
//         // std::stringstream ss;
//         pdef->print(std::cout);
//         path->print(std::cout);
//         // std::string myString = ss.str();
//         // std::cout << myString << std::endl;
//         std::string line;
//
//         while (std::getline(ss, line)) {
//             // Define the regex pattern
//             std::regex pattern(R"(Created (\d+) states)");  // This pattern matches 'int=' followed by one or more digits
//             std::smatch match;
//
//             // Iterate over all matches in the text
//             auto words_begin = std::sregex_iterator(line.begin(), line.end(), pattern);
//             auto words_end = std::sregex_iterator();
//
//             for (std::sregex_iterator i = words_begin; i != words_end; ++i) {
//                 std::smatch match = *i;
//                 std::string match_str = match.str(1);  // Get the first captured group
//                 return {std::stoi(match_str), path->length()};
//             }
//         }
//     } else
//         std::cout << "No solution found" << std::endl;
//
//
//     return {-1, -1};
// }
//
// /// single robot, on 2D maps, using RRT/RRT* (random sampling)
// std::tuple<int, double> plan_with_maze() {
//     // // construct the state space we are planning in
//     // // this is an EXAMPLE of how to construct custom spaces. not used here.
//     // ob::StateSpacePtr X(new ob::RealVectorStateSpace(1));
//     // auto X_RVecSp = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(X);
//     // // set the bounds for the space
//     // ob::RealVectorBounds bounds(1);
//     // bounds.setLow(0);
//     // bounds.setHigh(1);
//     // X_RVecSp->setBounds(bounds);
//     // ob::StateSpacePtr Y(new ob::RealVectorStateSpace(1));
//     // auto Y_RVecSp = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(Y);
//     // Y_RVecSp->setBounds(bounds);
//     // auto space = X + Y;
//     // // construct an instance of  space information from this state space
//     // auto si(std::make_shared<ob::SpaceInformation>(space));
//
//     /** MP type **/
//
//     ompl::app::SE2RigidBodyPlanning setup;
//
//     /** Model options (env/robot) **/
//
//     // std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car1_planar_robot.dae";
//     // std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Maze_planar_env.dae";
//     // std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_robot.dae";
//     // std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae";
//     // std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/RandomPolygons_planar_env.dae";
//     std::string robot_fname = "/home/itai/ompl3/omplapp-1.6.0-Source/resources/2D/cylinder.dae";
//     std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/H_planar_env.dae";
//
//     /** Model setup actions **/
//
//     setup.setEnvironmentMesh(env_fname);
//     setup.setRobotMesh(robot_fname);
//     // infer bounds from the environment we added
//     setup.inferEnvironmentBounds();
//
//     /** SI actions **/
//
//     auto si(setup.getSpaceInformation());
//     auto svc(setup.allocStateValidityChecker(si, setup.getGeometricStateExtractor(), false));
//     si->setStateValidityChecker(svc);
//     // set state validity checking for this space
//     // si->setStateValidityChecker(isStateValid);
//     // get the samples
//     // si->setValidStateSamplerAllocator(allocValidDeterministicSampler);
//     // set edge validator
//     si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
//     si->setup();
//
//     /** define start/goal states **/
//
//     // start
//     ompl::base::ScopedState<ompl::base::SE2StateSpace> start(si);
//     start->setX(-30.0); // randompoly 1
//     start->setY(-35.0); // randompoly 1
//     // goal
//     ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(start);
//     // goal->setX(45.00); // randompoly 1
//     // goal->setY(45.00); // randompoly 1
//     goal->setX(30.00); // randompoly 1
//     goal->setY(-35.00); // randompoly 1
//
//     /** Define the problem **/
//     auto pdef(std::make_shared<ob::ProblemDefinition>(si));
//     // set the start and goal states
//     pdef->setStartAndGoalStates(start, goal);
//     // set the optimization objective
//     // pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ClearanceObjective(si)));
//
//     /** Define the planner **/
//
//     // auto planner(std::make_shared<og::PRM>(si)); // RRT
//     auto planner(std::make_shared<og::RRTstar>(si)); // RRT*
//     // set the problem we are trying to solve for the planner
//     planner->setProblemDefinition(pdef);
//     // perform setup steps for the planner
//     planner->setup();
//
//     /** log stuff and start the run **/
//
//     // print the settings for this space
//     // si->printSettings(std::cout);
//     // print the problem settings
//     pdef->print(std::cout);
//     std::stringstream ss;
//     std::string line;
//     std::streambuf* org = std::cout.rdbuf(); // Remember std::cout's old state
//     std::cout.rdbuf(ss.rdbuf()); // Bind it to the output file stream
//     ob::PlannerStatus solved = planner->ob::Planner::solve(60.0);
//     std::cout.rdbuf(org); // Reset std::cout's old state
//
//     /** Analyze the results **/
//
//     if (solved) {
//         // get the goal representation from the problem definition (not the same as the goal state)
//         // and inquire about the found path
//         ob::PathPtr path = pdef->getSolutionPath();
//         double length =path->length();
//         std::cout << "Found solution:" << std::endl;
//         double clear = path->as<og::PathGeometric>()->clearance();
//         fmt::print("clearance={}", clear);
//         // print the path to screen
//         // std::stringstream ss;
//         pdef->print(std::cout);
//         path->print(std::cout);
//         // std::string myString = ss.str();
//         // std::cout << myString << std::endl;
//         std::string line;
//
//         while (std::getline(ss, line)) {
//             // Define the regex pattern
//             std::regex pattern(R"(Created (\d+) states)");  // This pattern matches 'int=' followed by one or more digits
//             std::smatch match;
//
//             // Iterate over all matches in the text
//             auto words_begin = std::sregex_iterator(line.begin(), line.end(), pattern);
//             auto words_end = std::sregex_iterator();
//
//             for (std::sregex_iterator i = words_begin; i != words_end; ++i) {
//                 std::smatch match = *i;
//                 std::string match_str = match.str(1);  // Get the first captured group
//                 return {std::stoi(match_str), path->length()};
//             }
//         }
//     } else
//         std::cout << "No solution found" << std::endl;
//     return {-1, -1};
// }

#pragma endregion unused_planners

#pragma region utility

/// used to differenciate between the planners and act accordingly
enum PlannerType {
    PRM_IMPLICIT_LATTICE_REGULAR,
    PRM_IMPLICIT_LATTICE_NN,
    PRM_IMPLICIT_RND,
    RRT,
    RRT_STAR
};

/// a basic structure to keep tests in
class TestData {
public:
    std::vector<double> X;
    std::vector<double> Y;
    int robotCount;
    double delta;
    std::string map;
    std::string setName;
    std::string testName;
    std::map<LatticeType, double> deltaOptimized = {};
    double resolution = 0.2;
    bool random = false;
};

class AnstarVsRndTestResult {
public:
    // Anstar
    double AnstarTotalLoc;
    double AnstarTotalGlo;
    double AnstarSearchGlo;
    double AnstarLength;
    // Rnd
    double RndTotalGlo;
    double RndSearchGlo;
    double RndLength;
    int RndSuccess;
    // Rnd-
    double RndMinusTotalGlo;
    double RndMinusSearchGlo;
    double RndMinusLength;
    int RndMinusSuccess;
};

std::vector<TestData> getTests() {

    // unused tests
    // TestData LatticeCompK1 = {
    //     { 0, 0},
    //     {1.7, -1.7},
    //     2, 0.5,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/kenny_env.dae", "LatticeK1"
    // };
    //
    // TestData LatticeCompK2 = {
    //     { 1, 1, -2},
    //     {-2, 2, 0},
    //     3, 0.5,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/kenny_env.dae", "LatticeK2"
    // };
    //
    // TestData LatticeCompUM1 = {
    //     { -3.7065, -40.6533},
    //     {-41.9224, 33.6204},
    //     2, 5,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae", "LatticeUM1"
    // };
    //
    // TestData LatticeCompUM2 = {
    //     { 20.8317, 7.18125, 37.3669},
    //     {35.6572, -23.4666, 34.1791},
    //     3, 8,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae", "LatticeUM2"
    // };
    //
    // TestData LatticeCompRP1 = {
    //     { -14.3555, 23.8745},
    //     {29.4973, -21.9291},
    //     2, 7,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/RandomPolygons_planar_env.dae", "LatticeRP1"
    // };
    //
    // TestData LatticeCompRP2 = {
    //     { -3.0345, -37.5667, 11.7975},
    //     {33.9878, 35.2388, 31.3037},
    //     3, 10,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/RandomPolygons_planar_env.dae", "LatticeRP2"
    // };
    //
    // TestData UniqueMazeRND = {
    //     {},
    //     {},
    //     3, 8,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae", "UM_RND",
    //     true
    // };
    //

    // TestData bugtrapT1 = {
    //     { 20.2229, 29.2904, 15.6985, 46.8042},
    //     {-28.2066, -7.14469, -30.5382, 30.508},
    //     4, 15,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap1"
    // };

    TestData bugtrapT1B = {
        { -5, -35, -5},
        {10, -10, -10},
        3, 14.7,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap","1B",
        {{LatticeType::AnStar, 16.4}, {LatticeType::Lc1, 13}, {LatticeType::Lc1b, 1}},
    };


    TestData bugtrapT2 = {
        { 10, 10, 5},
        {12, 25, 12},
        3, 16.9,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap", "2",
        {{LatticeType::AnStar, 17.2}, {LatticeType::Lc1, 16.6}, {LatticeType::Lc1b, 1}},
    };

    TestData bugtrapT2B = {
        { 10, 10, 10},
        {-11, 25, 12},
        3, 15.1,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap", "2B",
        {{LatticeType::AnStar, 19.9}, {LatticeType::Lc1, 16.5}, {LatticeType::Lc1b, 1}},
    };

    TestData bugtrapT3 = {
        { 10, 10, -12},
        {12, 35, 12},
        3, 12,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap", "3",
        {{LatticeType::AnStar, 13.4}, {LatticeType::Lc1, 21.2}, {LatticeType::Lc1b, 1}},
    };

    // TestData bugtrapT4 = {
    //     { 5, 5},
    //     {11, -11},
    //     2, 12,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap4"
    // };

    TestData bugtrapT5 = {
        { 5, 5, -5},
        {11, -11, 0},
        3, 15,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap", "5"
    };

    TestData bugtrapT6 = {
        { -29.8617, -38.5279, -31.8874, 30.0814},
        {1.81902, 1.18988, -34.254, -38.0848},
        4, 15,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap", "6",
        {{LatticeType::AnStar, 19.4}, {LatticeType::Lc1, -1}, {LatticeType::Lc2, 20.2}},
    };

    // TestData bugtrapT7 = {
    //     { 30.5001,  25.6474, -8.68071,  44.3456},
    //     { 22.7138, -30.1486, -25.3776, -34.295},
    //     4, 15,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap7"
    // };

    TestData bugtrapT8 = {
        { -8,  -8, -35},
        { 10, -10,   0},
        3, 14.3,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap", "8"
    };

    TestData bugtrapT8B = {
        { -8, -35, 8},
        { 10, -10, -10},
        3, 19,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap", "8B"
    };

    // TestData bugtrapT9 = {
    //     { 11, 11},
    //     { -11, -26},
    //     2, 15,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap9"
    // };

    // TestData bugtrapT10 = {
    //     { 11, -34},
    //     { -11, -11},
    //     2, 15.3,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap10"
    // };

    TestData bugtrapT11 = {
        { 11, 11, 11},
        { -11, -26, 11},
        3, 13,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae", "Bugtrap", "11"
    };

    // // narrow room tests
    // TestData narrowT1 = {
    //     { -2.301, -0.544425, -4.42128, 2.77218, 3.59467},
    //     {4.70545, -4.3204, -2.17076, -3.04564, 3.86121},
    //     5, 5,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BoundingBox_narrowest.dae", "BBN1"
    // };
    // TestData narrowT1B = {
    //     { -4.17253, -2.31971, -0.253293, 0.611664, -4.08302, 3.66713},
    //     {1.10275, -4.85285, 0.116617, 4.48314, 4.65878, -4.63278},
    //     6, 5,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BoundingBox_narrowest.dae", "BBN1B"
    // };
    // TestData narrowT2 = {
    //     { 3.05764, -3.6199, 0.0623748, 4.46321, -1.58111},
    //     {-4.36191, -1.99073, 4.33304, 4.80813, 1.00712},
    //     5, 5.1,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BoundingBox_narrowest.dae", "BBN2"
    // };
    // TestData narrowT2B = {
    //     { 1.21694, -4.82974, 0.278832, 0.717445, 4.23553, 4.88475},
    //     {0.257107, -4.30166, -4.29458, 4.23222, -2.49799, 1.72195},
    //     6, 5,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BoundingBox_narrowest.dae", "BBN2B"
    // };
    // TestData narrowT3 = {
    //     { 4.49547, -4.463, -0.902985, 3.92438, -0.132694},
    //     {-0.663655, 3.29021, 1.82297, 3.35383, -2.8315},
    //     5, 5,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BoundingBox_narrowest.dae", "BBN3"
    // };
    // TestData narrowT4 = {
    //     { 4, -4},
    //     {4, -4},
    //     2, 5.3,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BoundingBox_narrowest.dae", "BBN4"
    // };
    // TestData narrowT5 = {
    //     { -2, 2, -2, 2, -4.5},
    //     {-2, 2, 2, -2, -4.5},
    //     5, 4.4,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BoundingBox_narrowest.dae", "BBN5"
    // };
    // //Zigzag+bypass
    // TestData zigzagT1 = {
    //     { 1, 1},
    //     {24, -23},
    //     2, 2.75,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/room4.dae", "ZZB1"
    // };
    // TestData zigzagT2 = {
    //     { 1, 1},
    //     {24, -23},
    //     2, 2.75,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/room6.dae", "ZZB2"
    // };
    // TestData zigzagT3 = {
    //     { 1, 1},
    //     {24, -23},
    //     2, 2.75,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/room7.dae", "ZZB3"
    // };
    TestData kennyT1 = {
        { 1, 1, -2},
        {-2, 2, 0},
        3, 0.56,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/kenny_env.dae", "Kenny", "K1"
    };

    // TestData kennyT2 = {
    //     { 0, 2.2, 0, -2.2},
    //     { 2.2, 0, -2.2, 0},
    //     4, 0.1,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/kenny_env.dae", "K2"
    // };

    TestData UniqueMaze1 = {
        { 20.8317, 7.18125, 37.3669},
        {35.6572, -23.4666, 34.1791},
        3, 8,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae", "UniqueMaze", "1"
    };
    TestData UniqueMaze2 = {
        { 22, 22, 43},
        {-3, -12, -3},
        3, 9.8,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae", "UniqueMaze", "2"
    };
    TestData UniqueMaze3 = {
        { 22, 22, 43, 43},
        {-3, -12, -3, -12},
        4, 8,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae", "UM", "3",
        {{LatticeType::AnStar, 10.2}, {LatticeType::Lc1, -1}, {LatticeType::Lc2, 11.2}},
    };

    // TestData UniqueMaze4 = {
    //     { -17, -17},
    //     {7, -3},
    //     2, 6.8,
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae", "UM4"
    // };
    //
    // TestData UniqueMaze4B1 = {
    //     { -17, -45},
    //     {7, -13},
    //     2, 6.3, //6
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae", "UM4B1"
    // };
    // TestData UniqueMaze4B2 = {
    //     { -17, -45},
    //     {7, -13},
    //     2, 6, //6
    //     std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae", "UM4B2"
    // };

    TestData UniqueMaze4B3 = {
        { -17, -17, -17},
        {15, 35, 25},
        3, 3,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae", "UniqueMaze", "4B3"
    };

    TestData UniqueMaze5 = {
        { 46.539, 45.8771, 11.3239},
        {-12.1491, -23.8409, 16.4302},
        3, 7.9,
        std::string(OMPLAPP_RESOURCE_DIR) + "/2D/UniqueSolutionMaze_env.dae", "UniqueMaze", "5"
    };

    /// Zn vs Dn* vs An*
    std::vector<TestData> lattice_tests;

    /// d=6
    // lattice_tests.push_back(bugtrapT1B);
    // lattice_tests.push_back(bugtrapT2);
    // lattice_tests.push_back(bugtrapT2B);
    // lattice_tests.push_back(bugtrapT3);
    // lattice_tests.push_back(bugtrapT5);
    // lattice_tests.push_back(bugtrapT6);
    // lattice_tests.push_back(bugtrapT8);
    // lattice_tests.push_back(bugtrapT8B);
    // lattice_tests.push_back(bugtrapT11);

    lattice_tests.push_back(kennyT1);

    // lattice_tests.push_back(UniqueMaze1);
    // lattice_tests.push_back(UniqueMaze2);
    // lattice_tests.push_back(UniqueMaze4B3);
    // lattice_tests.push_back(UniqueMaze5);


    // lattice_tests.push_back(UniqueMaze3);

    // make sure its nice and ordered
    std::sort(lattice_tests.begin(), lattice_tests.end(),
        [](const TestData& t1, const TestData& t2) {
            if (t1.setName < t2.setName) {
                return true;
            }
            if (t1.setName == t2.setName && t1.robotCount < t2.robotCount) {
                return true;
            }
            if (t1.setName == t2.setName && t1.robotCount == t2.robotCount && t1.testName < t2.testName) {
                return true;
            }
            return false;
        });

    return lattice_tests;
}

std::string getUniqueTestFolder() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_time = *std::localtime(&now_c);
    std::string uniqueFolderName = fmt::format("{}{:02}{:02}{}{:02}{:02}",
        local_time.tm_mday, local_time.tm_mon + 1, local_time.tm_year % 100,
        local_time.tm_hour, local_time.tm_min, local_time.tm_sec);
    std::string folderBase = "TestResult/" + uniqueFolderName + "/";
    return folderBase;
}

/// OMPL's default planners use a sum-of-subspaces distance. this is used to get OUR distance value.
/// @param path the robot path (states)
/// @param robotCount number of robots
/// @return euclidean l2 length of path
double getPathLength(const ob::PathPtr& path, int robotCount) {
    auto pathGeo = path->as<ompl::geometric::PathGeometric>();
    const auto& pathStates = pathGeo->getStates();
    double dist = 0;
    for (int i = 0; i < pathStates.size() - 1; ++i) {
        double segDist = 0;
        const auto* aState = pathStates[i];
        const auto* bState = pathStates[i + 1];
        for (int i = 0; i < robotCount; ++i) {
            auto aX = aState->as<ob::CompoundStateSpace::StateType>()->components[i]
                                ->as<ob::SE2StateSpace::StateType>()->getX();
            auto aY = aState->as<ob::CompoundStateSpace::StateType>()->components[i]
                            ->as<ob::SE2StateSpace::StateType>()->getY();
            auto bX = bState->as<ob::CompoundStateSpace::StateType>()->components[i]
                ->as<ob::SE2StateSpace::StateType>()->getX();
            auto bY = bState->as<ob::CompoundStateSpace::StateType>()->components[i]
                            ->as<ob::SE2StateSpace::StateType>()->getY();
            segDist += std::pow(aX - bX, 2) + std::pow(aY - bY, 2);
        }
        dist += std::sqrt(segDist);
    }
    return dist;
}

#pragma endregion utility

#pragma region used_planners

/// iPRM, 2D maps, N>=1 number of robots
/// @param lattice Zn/Dn*/An*
/// @param env_fname see file for a comprehensive list
/// @param testName this name will also appear in the test results
/// @param plannerType Lattice (with or without NN) and RND
/// @param robotCount number of robots
/// @param x list of X values for the robots
/// @param y list of Y values for the robots
/// @param delta the delta from Dayan23
/// @param epsilon the epsilon from Dayan23
/// @param maxSamples for RND: limit the max sample count
/// @param radius for RND: use a different radius (usually the PRM* radius)
/// @param runtime for RND: limit runtime
/// @param countSamplesForRND for Lattice: count all map samples (for RND comparison tests)
/// @param sanityCheckMode unsupported
/// @param buildAllGraph debug option
/// @return a tuple of {total_samples, path_length, NA, constructionTime, astarTime, NA}
std::tuple<int, double, double, signed long, signed long, double> implicit_PRM_2D_multi(
    ompl::geometric::ImplicitPRM::LatticeType lattice, const std::string& env_fname, std::string testName,
    PlannerType plannerType, int robotCount, std::vector<double> x, std::vector<double> y, // general params
    double delta, double epsilon, // lattice params
    unsigned long int maxSamples, double radius, long runtime, // RND params
    bool countSamplesForRND = false, bool sameRadius = false) {

    /** MP type **/

    ompl::app::SE2MultiRigidBodyPlanning setup(robotCount);

    /** env/robot **/

    // custom
    // robots: cylinder, cube, cube_big, kenny_env, car1_planar_robot, UniqueSolutionMaze_robot, Barriers_easy_robot
    std::string robot_fname = "/home/itai/ompl3/omplapp-1.6.0-Source/resources/2D/cylinder.dae";

    /** Model setup actions **/

    setup.setRobotMesh(robot_fname);
    for (int i = 0; i < robotCount - 1; ++i) {
        setup.addRobotMesh(robot_fname);
    }
    setup.setEnvironmentMesh(env_fname);
    // infer bounds from the environment we added
    setup.inferEnvironmentBounds();

    /** SI actions **/

    auto si(setup.getSpaceInformation());
    auto svc(setup.allocStateValidityChecker(si, setup.getGeometricStateExtractor(), true));
    si->setStateValidityChecker(svc);
    // set state validity checking for this space
    // si->setStateValidityChecker(isStateValid);
    if (plannerType == PRM_IMPLICIT_RND) { // optional things for non-lattice planners
        // get the samples
        // si->setValidStateSamplerAllocator(allocValidDeterministicSampler);
    }
    // set edge validator
    si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
    si->setup();

    /** define start/goal states **/

    // this is the setup for N-swap
    ompl::base::ScopedState<ob::CompoundStateSpace> start(si);
    ompl::base::ScopedState<ob::CompoundStateSpace> goal(start);
    for (int i = 0; i < robotCount; ++i) {
        auto *startRobot = start->as<ob::SE2StateSpace::StateType>(i);
        auto *goalRobot = goal->as<ob::SE2StateSpace::StateType>(i);
        startRobot->setX(x[i]);
        startRobot->setY(y[i]);
        startRobot->setYaw(0);
        goalRobot->setX(x[(i + 1) % robotCount]);
        goalRobot->setY(y[(i + 1) % robotCount]);
        goalRobot->setYaw(0);
    }

    Eigen::VectorXd startEigen(2*robotCount);
    Eigen::VectorXd goalEigen(2*robotCount);
    for (int i = 0; i < robotCount; ++i) {
        startEigen[2 * i] = x[i];
        startEigen[2 * i + 1] = y[i];
        goalEigen[2 * i] = x[(i + 1) % robotCount];
        goalEigen[2 * i + 1] = y[(i + 1) % robotCount];
    }

    /** Define the problem **/

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    /** Define the planner **/

    // create a planner for the defined space
    auto planner = std::make_shared<og::ImplicitPRM>(si);

    // set the problem we are trying to solve for the planner
    planner->setSameRadius(sameRadius);
    planner->setProblemDefinition(pdef);
    // set planner options
    planner->setStartAndGoalEigen(startEigen, goalEigen);
    planner->setRobotCount(robotCount);
    if (plannerType == PRM_IMPLICIT_LATTICE_NN || plannerType == PRM_IMPLICIT_LATTICE_REGULAR) {
        planner->setPrmType(ompl::geometric::ImplicitPRM::Lattice);
        if (plannerType == PRM_IMPLICIT_LATTICE_NN) planner->setPrmType(ompl::geometric::ImplicitPRM::LatticeWithNN);
        planner->setCountSamplesForRND(countSamplesForRND); // false == comparison only between lattices, true = compare to RND
        // planner->setbuildAllGraph(buildAllGraph); // this is a debug function, ignore this
        planner->setLatticeType(lattice, delta, epsilon); // uniquemaze
    } else if (plannerType == PRM_IMPLICIT_RND) {
        planner->setPrmType(ompl::geometric::ImplicitPRM::Random);
        // lattices define this radius in a special function, not here
        // std::cout << "THE RADIUS IS " << radius << std::endl;
        planner->as<og::ImplicitPRM>()->setNNRadius(radius);
    }
    // perform setup steps for the planner
    planner->setup();

    /** log stuff **/

    // print the settings for this space
    si->printSettings(std::cout);
    // print the problem settings
    pdef->print(std::cout);

    /** start the run **/

    // attempt to solve the problem (with some time limit..)
    ob::PlannerStatus solved;
    if (plannerType == PRM_IMPLICIT_LATTICE_NN || plannerType == PRM_IMPLICIT_LATTICE_REGULAR) {
        int runtime = 1000; // 10000
        if (lattice == ompl::geometric::ImplicitPRM::AnStar) runtime = 600;
        solved = planner->ob::Planner::solve(runtime);
    } else if (plannerType == PRM_IMPLICIT_RND) {
        auto plannerPRM = planner->as<og::ImplicitPRM>(); // LazyFullPRM
        plannerPRM->setSampleLimit(maxSamples);
        solved = planner->ob::Planner::solve(runtime); // RRT*
    }
    std::cout << solved << std::endl;

    /** Analyze the results **/

    if (solved) {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        double length = getPathLength(path, robotCount);
        // double clear =pathGeo->clearance(); // can be useful
        og::ImplicitPRM::RunResults res = planner->as<og::ImplicitPRM>()->getResults(); // I keep my result in this structure
        fmt::print("Path length={}, Path separated length={}\n", length, res.lengthSeparated);

        // print the path to screen
        path->print(std::cout);

        // save to file
        if (plannerType == PRM_IMPLICIT_LATTICE_NN || plannerType == PRM_IMPLICIT_LATTICE_REGULAR) {
            std::ofstream outfile;
            outfile.open(testName + ".path");
            auto pathGeo = path->as<ompl::geometric::PathGeometric>();
            pathGeo->interpolate(200);
            pathGeo->printAsMatrix(outfile);
            outfile.close();
            ompl::tools::SelfConfig config = ompl::tools::SelfConfig(si);
            outfile.open("testpath.cfg");
            config.print(outfile);
            outfile.close();

            return {res.samples, length, -1, res.constructionTime,
            res.astarTime, planner->as<og::ImplicitPRM>()->getRadius()};
        }

        // return the results
        return {res.samples, length, -1, res.constructionTime,
            res.astarTime, -1};
    } else
        std::cout << "No solution found" << std::endl;
    return {-1, -1, -1, -1, -1, -1};
}

/// LPRM without NN, 2D maps, N>=1 number of robots
/// @param lattice Zn/Dn*/An*
/// @param env_fname see file for a comprehensive list
/// @param testName this name will also appear in the test results
/// @param robotCount number of robots
/// @param X list of X values for the robots
/// @param Y list of Y values for the robots
/// @param delta the delta from Dayan23
/// @param epsilon the epsilon from Dayan23
/// @param countSamples count all map samples (for RND comparison tests)
/// @return a tuple of {total_samples, path_length, NA, constructionTime, astarTime, NA}
std::tuple<int, double, double, signed long, signed long, double> iLPRM2D(ompl::geometric::ImplicitPRM::LatticeType lattice,
                                                                          const std::string& env_fname, std::string testName,
                                                                          int robotCount, std::vector<double> X, std::vector<double> Y,
                                                                          double delta, double epsilon, bool countSamples = true) {
    return implicit_PRM_2D_multi(lattice, env_fname, testName, PRM_IMPLICIT_LATTICE_REGULAR,
            robotCount, X, Y, delta, epsilon, -1, -1, countSamples, countSamples);
}

/// LPRM with NN, 2D maps, N>=1 number of robots
/// @param lattice Zn/Dn*/An*
/// @param env_fname see file for a comprehensive list
/// @param testName this name will also appear in the test results
/// @param robotCount number of robots
/// @param X list of X values for the robots
/// @param Y list of Y values for the robots
/// @param delta the delta from Dayan23
/// @param epsilon the epsilon from Dayan23
/// @param countSamples count all map samples (for RND comparison tests)
/// @return a tuple of {total_samples, path_length, NA, constructionTime, astarTime, NA}
std::tuple<int, double, double, signed long, signed long, double> iNNLPRM2D(ompl::geometric::ImplicitPRM::LatticeType lattice,
                                                                            const std::string& env_fname, std::string testName,
                                                                            int robotCount, std::vector<double> X, std::vector<double> Y, double delta, double epsilon, bool countSamples = true) {
    return implicit_PRM_2D_multi(lattice, env_fname, testName, PRM_IMPLICIT_LATTICE_NN,
    robotCount, X, Y, delta, epsilon, -1, -1, countSamples, false);
}

/// PRM*/PRM, 2D maps, N>=1 number of robots.
/// @param env_fname see file for a comprehensive list
/// @param testName this name will also appear in the test results
/// @param robotCount number of robots
/// @param X list of X values for the robots
/// @param Y list of Y values for the robots
/// @param maxSamples for RND: limit the max sample count
/// @param radius for RND: use a different radius (usually the PRM* radius)
/// @param runtime for RND: limit runtime
/// @return a tuple of {total_samples, path_length, NA, constructionTime, astarTime}
std::tuple<int, double, double, signed long, signed long> iRndPRM2D(
    const std::string& env_fname, std::string testName,
    int robotCount, std::vector<double> X, std::vector<double> Y, unsigned long int maxSamples, double radius,
    long runtime, bool sameRadius) {
    auto [rndStates, rndLengthLoc, rndClear, rndConstructionTimeLoc, rndAstarRuntimeLoc, tmp]
    = implicit_PRM_2D_multi(ompl::geometric::ImplicitPRM::LatticeType::Unknown, env_fname, testName, PRM_IMPLICIT_RND,
            robotCount, X, Y, -1, -1, maxSamples, radius, runtime, false, sameRadius);
    // we don't use the radius return value in RND-PRM
    return {rndStates, rndLengthLoc, rndClear, rndConstructionTimeLoc, rndAstarRuntimeLoc};
}

// multiple disks, on 2D maps, using RRT/RRT* (random sampling)

/// OMPL version of PRM*
/// @param env_fname see file for a comprehensive list
/// @param testName this name will also appear in the test results
/// @param robotCount number of robots
/// @param x list of X values for the robots
/// @param x list of Y values for the robots
/// @param timeLimit limit runtime
/// @return a tuple of {NA, length, NA, runtime, NA}
std::tuple<int, double, double, signed long, signed long> general_2D_multi(const std::string& env_fname, const std::string& testName,
                                                                           int robotCount, std::vector<double> x, std::vector<double> y,
                                                                           double timeLimit, const std::string& type = "PRM") {
    std::cout << "Testing: " << testName << std::endl;
    /** MP type **/

    ompl::app::SE2MultiRigidBodyPlanning setup(robotCount);

    /** env/robot **/

    // robots: cylinder, cube, cube_big, kenny_env, car1_planar_robot, UniqueSolutionMaze_robot, Barriers_easy_robot
    std::string robot_fname = "/home/itai/ompl3/omplapp-1.6.0-Source/resources/2D/cylinder.dae";

    /** Model setup actions **/
    setup.setEnvironmentMesh(env_fname);
    setup.setRobotMesh(robot_fname);
    for (int i = 0; i < robotCount - 1; ++i) {
        setup.addRobotMesh(robot_fname);
    }
    // infer bounds from the environment we added
    setup.inferEnvironmentBounds();

    /** SI actions **/

    auto si(setup.getSpaceInformation());
    auto svc(setup.allocStateValidityChecker(si, setup.getGeometricStateExtractor(), true));
    si->setStateValidityChecker(svc);
    // set state validity checking for this space
    // si->setStateValidityChecker(isStateValid);
    // set valid sampler
    // si->setValidStateSamplerAllocator(allocValidDeterministicSampler);
    // set edge validator
    si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
    si->setup();

    /** define start/goal states **/

    // currently set to N-swap
    ompl::base::ScopedState<ob::CompoundStateSpace> start(si);
    ompl::base::ScopedState<ob::CompoundStateSpace> goal(start);
    for (int i = 0; i < robotCount; ++i) {
        auto *startRobot = start->as<ob::SE2StateSpace::StateType>(i);
        auto *goalRobot = goal->as<ob::SE2StateSpace::StateType>(i);
        startRobot->setX(x[i]);
        startRobot->setY(y[i]);
        startRobot->setYaw(0);
        goalRobot->setX(x[(i + 1) % robotCount]);
        goalRobot->setY(y[(i + 1) % robotCount]);
        goalRobot->setYaw(0);
    }

    /** Define the problem **/

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);
    // on RRT*, set the optimization objective
    // IMPORTANT: this is also important for PRM. without this, PRM won't stop after finding a solution.
    auto optObj = ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
    optObj->setCostThreshold(ob::Cost(std::numeric_limits<double>::max()));
    pdef->setOptimizationObjective(optObj);
    // pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ClearanceObjective(si)));

    /** Define the planner **/

    ompl::base::PlannerPtr planner(std::make_shared<og::PRMstar>(si));
    if (type == "RRTstar") {
        planner = std::make_shared<og::RRTstar>(si);
    }
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    // perform setup steps for the planner
    planner->setup();


    /** log stuff and start the run **/

    // print the settings for this space
    si->printSettings(std::cout);
    // print the problem settings
    pdef->print(std::cout);

    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout << type << " timelimit: t=" << timeLimit << " sec" << std::endl;
    auto status = planner->ob::Planner::solve(timeLimit); // LazyFullPRM
    auto t2 = std::chrono::high_resolution_clock::now();
    auto dur =
    duration_cast<std::chrono::microseconds>(t2 - t1).count();

    /** Analyze the results **/

    if (status == ob::PlannerStatus::EXACT_SOLUTION) {
        std::cout << "Time for " << type << ":" << dur << std::endl;
        std::cout << "hello" << std::endl;
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        double length = getPathLength(path, robotCount);

        // print the path to screen
        path->print(std::cout);
        std::cout << "hello " << length << std::endl;
        std::cout << "in plan_with_maze_multi" << std::endl;
        return {-1, length, -1, dur, -1};
    }
    std::cout << "No solution found" << std::endl;
    return {-1, -1, -1, -1, -1};
}

/// run the OMPL version of PRM* (which is incremental)
/// @param env_fname see file for a comprehensive list
/// @param testName this name will also appear in the test results
/// @param robotCount number of robots
/// @param X list of X values for the robots
/// @param Y list of Y values for the robots
/// @param runtime limit runtime
/// @return a tuple of {NA, length, NA, runtime, NA}
std::tuple<int, double, double, signed long, signed long> incRndPRM2D(
    const std::string& env_fname, std::string testName,
    int robotCount, std::vector<double> X, std::vector<double> Y, double runtime) {
    auto [rndStates, rndLengthLoc, rndClear, rndTotalTimeLoc, rndAstarRuntimeLoc]
    = general_2D_multi(env_fname, testName, robotCount, X, Y, runtime);
    // we don't use the radius return value in RND-PRM
    return {-1, rndLengthLoc, -1, rndTotalTimeLoc, -1};
}

/// run the OMPL version of PRM* (which is incremental)
/// @param env_fname see file for a comprehensive list
/// @param testName this name will also appear in the test results
/// @param robotCount number of robots
/// @param X list of X values for the robots
/// @param Y list of Y values for the robots
/// @param runtime limit runtime
/// @return a tuple of {NA, length, NA, runtime, NA}
std::tuple<int, double, double, signed long, signed long> RRT2D(
    const std::string& env_fname, std::string testName,
    int robotCount, std::vector<double> X, std::vector<double> Y, double runtime) {
    auto [a_, rndLengthLoc, b_, rndTotalTimeLoc, c_]
    = general_2D_multi(env_fname, testName, robotCount, X, Y, runtime, "RRTstar");
    // we don't use the radius return value in RND-PRM
    return {-1, rndLengthLoc, -1, rndTotalTimeLoc, -1};
}

/// runs a default iPRM planner setup to find a random valid start state
/// @param env_fname see file for a comprehensive list
/// @param robotCount number of robots
/// @return an Eigen::VectorXd vector of {X1, Y1, ... , XN, YN} for N robots
Eigen::VectorXd getRandomStartPoint(const std::string& env_fname, int robotCount) {
    /** MP type **/

    ompl::app::SE2MultiRigidBodyPlanning setup(robotCount);

    /** Model options (env/robot) **/

    // a wider cylinder (for bigger clearance)
    std::string robot_fname = "/home/itai/ompl3/omplapp-1.6.0-Source/resources/2D/cylinder_wider.dae";

    /** Model setup actions **/

    setup.setEnvironmentMesh(env_fname);
    setup.setRobotMesh(robot_fname);
    for (int i = 0; i < robotCount - 1; ++i) {
        setup.addRobotMesh(robot_fname);
    }
    // infer bounds from the environment we added
    setup.inferEnvironmentBounds();

    /** SI actions **/

    auto si(setup.getSpaceInformation());
    auto svc(setup.allocStateValidityChecker(si, setup.getGeometricStateExtractor(), true));
    si->setStateValidityChecker(svc);
    // set state validity checking for this space
    // si->setStateValidityChecker(isStateValid);
    // get the samples
    // si->setValidStateSamplerAllocator(allocValidDeterministicSampler);
    // set edge validator
    si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
    si->setup();

    /** Define the planner **/

    ompl::base::PlannerPtr planner(std::make_shared<og::ImplicitPRM>(si)); // RRT
    planner->as<og::ImplicitPRM>()->setRobotCount(robotCount);
    Eigen::VectorXd validStart = planner->as<og::ImplicitPRM>()->getValidStartState();
    return validStart;
}

#pragma endregion used_planners

#pragma region experiment_funcs

/// return the number of samples in all lattices, with various (dimension, delta, epsilon) values.
/// @return number of samples
int findNumberOfSamples() {
    std::vector<double> dims{2, 3, 4, 5, 6, 7, 8, 9, 10/*, 11, 12*/};
    std::vector<double> deltas{/*0.33, 0.3, 0.25, 0.2*/1};
    std::vector<double> epsilons{2/*, 2, 1, 0.9, 0.75, 0.65, 0.5, 0.4/*, 0.3, 0.2#1#*/};
    std::vector<Lattices::LatticeType> types{/*AnStar, DnStar,*/ Lattices::Zn };
    for (Lattices::LatticeType type: types) {
        if (type == LatticeType::Zn) {
            std::cout << "Working on lattice: Zn" << std::endl;
        } else if (type == LatticeType::DnStar) {
            std::cout << "Working on lattice: Dn*" << std::endl;
        } else if (type == LatticeType::AnStar) {
            std::cout << "Working on lattice: An*" << std::endl;
        }
        for (double epsilon : epsilons) {
            for (double delta : deltas) {
                std::string csvResSamples = "";
                std::string csvResEdges = "";
                for (int dim: dims) {
                    // count samples
                    auto start1 = std::chrono::high_resolution_clock::now();
                    Lattices::Lattice lattice = Lattices::Lattice(dim, delta, epsilon, type);
                    auto [samples, sumOfEdges] = lattice.getSampleCountInBallNew();
                    auto end1 = std::chrono::high_resolution_clock::now();
                    auto duration1 = duration_cast<std::chrono::milliseconds>(end1 - start1);
                    std::cout << "dim = " << dim << ", delta = " << delta << ", epsilon = " << epsilon << std::endl;
                    fmt::print("[{}MS]: samples={}, sum of edges={}\n", duration1.count(), samples, sumOfEdges);
                    csvResSamples += fmt::to_string(samples) + ",";
                    csvResEdges += fmt::to_string(sumOfEdges) + ",";
                    std::cout << "===================" << std::endl;
                }
                std::cout << epsilon << ": samples = [" << csvResSamples << "]" << std::endl;
                std::cout << epsilon << ": edges = [" << csvResEdges << "]" << std::endl;
            }
        }
    }
    return 0;
}

/// compare An* to PRM*/PRM (averaged on N runs), and report statistics
/// @param env_fname see file for a comprehensive list
/// @param testName this name will also appear in the test results
/// @param robotCount number of robots
/// @param xInit list of X values for the robots
/// @param yInit list of Y values for the robots
/// @param delta the delta from Dayan23
/// @param epsilon the epsilon from Dayan23
/// @param sameRadius  unused
/// @param setSamples  unused
void compareAnStarToOthers(const std::string& env_fname, const std::string& folderName, const std::string& setName,
                           const std::string& testName, int robotCount,
                           std::vector<double> xInit, std::vector<double> yInit,
                           double delta, double epsilon, AnstarVsRndTestResult& res,
                           bool sameRadius = false, long setSamples = -1) {
        std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    // 2D tests
    int RUNS = 1;
    if (robotCount == 6) RUNS = 1;
    int i = 0;
    auto micTOsec = 1e6;
    int detTotalAttempts = 0;

    double avgTimeQualityAnVsRRT = 0;
    double avgLenQualityAnVsRRT = 0;
    std::vector<double> medianTimeQualityAnVsRRT;
    std::vector<double> medianLenQualityAnVsRRT;

    int rndTotalSuccess = 0;
    // regular
    double avgTotalTimeQualityAnVsRND = 0;
    double avgAstarTimeQualityAnVsRND = 0;
    double avgLenQualityAnVsRND = 0;
    std::vector<double> medianTotalTimeQualityAnVsRND;
    std::vector<double> medianAstarTimeQualityAnVsRND;
    std::vector<double> medianLenQualityAnVsRND;
    // NN
    double avgTotalTimeQualityAnVsRNDNN = 0;
    double avgAstarTimeQualityAnVsRNDNN = 0;
    double avgLenQualityAnVsRNDNN = 0;
    std::vector<double> medianTotalTimeQualityAnVsRNDNN;
    std::vector<double> medianAstarTimeQualityAnVsRNDNN;
    std::vector<double> medianLenQualityAnVsRNDNN;
    int rndRepeats = 20;
    while (i < RUNS) {
        fmt::print("\nStarting run #{}\n", i);
        // auto validStart = getRandomStartPoint(env_fname, robotCount);
        fmt::print("\nChose a starting point.\n");
        // std::vector<double> X;
        // std::vector<double> Y;
        // for (int j = 0; j < robotCount; ++j) {
            // X.push_back(validStart[2 * j]);
            // Y.push_back(validStart[2 * j + 1]);
        // }
        std::vector<double> X = xInit;
        std::vector<double> Y = yInit;
        detTotalAttempts++;
        // no NN
        auto [sampleCount, detLength, detClear, detConstructionTime, detAstarRuntime, detRadius] =
            iLPRM2D(ompl::geometric::ImplicitPRM::LatticeType::Lc1, env_fname, testName,
                    robotCount, X, Y, delta, epsilon, true);
        if (sampleCount == -1) continue;
        long finalSampleCount = sampleCount;
        // NN
        auto [sampleCountNN, detLengthNN, detClearNN, detConstructionTimeNN, detAstarRuntimeNN, detRadiusNN] =
            iNNLPRM2D(ompl::geometric::ImplicitPRM::LatticeType::AnStar, env_fname, testName,
                      robotCount, X, Y, delta, epsilon, false);
        // update results
        res.AnstarTotalLoc = (detConstructionTime + detAstarRuntime) / micTOsec;
        res.AnstarTotalGlo = (detConstructionTimeNN + detAstarRuntimeNN)  / micTOsec;
        res.AnstarSearchGlo = detAstarRuntimeNN  / micTOsec;
        res.AnstarLength = detLength;

        // print results
        double detTotalRuntime = detConstructionTime + detAstarRuntime;
        double detTotalRuntimeNN = detConstructionTimeNN + detAstarRuntimeNN;
        std::cout << "samples: " << finalSampleCount << std::endl;
        std::cout << "regular: " << detConstructionTime << " || " << detAstarRuntime << std::endl;
        std::cout << "NN: " << detConstructionTimeNN << " || " << detAstarRuntimeNN << std::endl;
        double AvgRndTotalRuntime = 0;
        double AvgRndAstarRuntime = 0;
        double AvgRndLength = 0;
        std::vector<double> medianRndTotalRuntime;
        std::vector<double> medianRndAstarRuntime;
        std::vector<double> medianRndLength;
        std::cout << "samples=" << finalSampleCount << std::endl;
        std::cout << "runtime=" << detTotalRuntime << std::endl;
        std::cout << "fin_length=" << detLength << std::endl;

        // run RND tests
        double limitedRunTime = 1000;
        std::cout << "limiting to T=" << limitedRunTime << std::endl;
        std::vector<long> rndTotalRuntime;
        std::vector<long> rndAstarRuntime;
        std::vector<long> rndLength;
        for (int t = 0; t < rndRepeats; ++t) {
            std::cout << "Starting repeat #" << t << std::endl;
            auto [rndStates, rndLengthLoc, rndClear, rndConstructionTimeLoc, rndAstarRuntimeLoc] =  /// RETURN THIS
            iRndPRM2D(env_fname, testName, robotCount, X, Y, finalSampleCount, detRadius, limitedRunTime, sameRadius);
            if (rndConstructionTimeLoc > 0) {
                rndTotalRuntime.push_back(rndConstructionTimeLoc + rndAstarRuntimeLoc);
                rndAstarRuntime.push_back(rndAstarRuntimeLoc);
                medianRndTotalRuntime.push_back(rndConstructionTimeLoc + rndAstarRuntimeLoc);
                medianRndAstarRuntime.push_back(rndAstarRuntimeLoc);
            }
            if (rndLengthLoc > 0) {
                rndLength.push_back(rndLengthLoc);
                medianRndLength.push_back(rndLengthLoc);
            }
        }
        int rndLocalSuccess = rndTotalRuntime.size();
        rndTotalSuccess += rndLocalSuccess;
        std::cout << "RND success rate: " << (rndLocalSuccess * 100) / rndRepeats << "%" << std::endl;
        double medianRndTotalRuntimePerRun = -1;
        double medianRndAstarRuntimePerRun = -1;
        double medianRndLengtPerRun = -1;
        double AvgRndTotalRuntimePerRun = -1;
        double AvgRndAstarRuntimePerRun = -1;
        double AvgRndLengthPerRun = -1;
        if (rndLocalSuccess > 0) {
            AvgRndTotalRuntimePerRun = GetAverage(rndTotalRuntime);
            AvgRndAstarRuntimePerRun = GetAverage(rndAstarRuntime);
            AvgRndLengthPerRun = GetAverage(rndLength);
            medianRndTotalRuntimePerRun = GetMedian(medianRndTotalRuntime);
            medianRndAstarRuntimePerRun = GetMedian(medianRndAstarRuntime);
            medianRndLengtPerRun = GetMedian(medianRndLength);
        }

        if (rndLocalSuccess > 0) {
            // regular
            avgTotalTimeQualityAnVsRND += AvgRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime);
            avgAstarTimeQualityAnVsRND += AvgRndAstarRuntimePerRun / static_cast<double>(detAstarRuntime);
            avgLenQualityAnVsRND += AvgRndLengthPerRun / detLength;
            medianTotalTimeQualityAnVsRND.push_back(medianRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime));
            medianAstarTimeQualityAnVsRND.push_back(medianRndAstarRuntimePerRun / static_cast<double>(detAstarRuntime));
            medianLenQualityAnVsRND.push_back(medianRndLengtPerRun / detLength);
            // NN
            avgTotalTimeQualityAnVsRNDNN += AvgRndTotalRuntimePerRun / static_cast<double>(detTotalRuntimeNN);
            avgAstarTimeQualityAnVsRNDNN += AvgRndAstarRuntimePerRun / static_cast<double>(detAstarRuntimeNN);
            avgLenQualityAnVsRNDNN += AvgRndLengthPerRun / detLength;
            medianTotalTimeQualityAnVsRNDNN.push_back(medianRndTotalRuntimePerRun / static_cast<double>(detTotalRuntimeNN));
            medianAstarTimeQualityAnVsRNDNN.push_back(medianRndAstarRuntimePerRun / static_cast<double>(detAstarRuntimeNN));
            medianLenQualityAnVsRNDNN.push_back(medianRndLengtPerRun / detLengthNN);
        }
        i++;

        std::ofstream outFile;
        // create dir
        std::string testFullName = setName + testName;
        std::string path = folderName + "/" + setName + "/";
        std::filesystem::create_directories(path);
        outFile.open(path + testFullName + ".txt");
        std::cout << "Printing " << testName << std::endl;
        if (outFile.is_open()) {
            std::cout << "Printing started " << testFullName << std::endl;
            outFile << "Map: " << env_fname << ", de = (" << delta << "," << epsilon << ")" << std::endl
            << "Testing in RobotCount=" << robotCount << " with Scenarios=" << RUNS << " and RRT,PRM averaged each on N=" << rndRepeats << "runs." << std::endl;

            outFile << "An*: Regular";
            outFile << " Time=" << detTotalRuntime / micTOsec;
            outFile << " A*-Time=" << detAstarRuntime / micTOsec;
            outFile << " Length=" << detLength;
            outFile << std::endl;

            outFile << "An*: NN";
            outFile << " Time=" << detTotalRuntimeNN / micTOsec;
            outFile << " A*-Time=" << detAstarRuntimeNN / micTOsec;
            outFile << " Length=" << detLength;
            outFile << std::endl;

            outFile << "[RND Median]";
            outFile << " total_time="  << medianRndTotalRuntimePerRun / micTOsec;
            outFile << " Q_total_time="  << medianRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime);
            outFile << " Q_total_time_NN="  << medianRndTotalRuntimePerRun / static_cast<double>(detTotalRuntimeNN);
            outFile << " astar_time="  << medianRndAstarRuntimePerRun / micTOsec;
            outFile << " Q_astar_time="  << medianRndAstarRuntimePerRun / static_cast<double>(detAstarRuntime);
            outFile << " Q_astar_time_NN="  << medianRndAstarRuntimePerRun / static_cast<double>(detAstarRuntimeNN);
            outFile << " length="  << medianRndLengtPerRun;
            outFile << " Q_length="  << medianRndLengtPerRun / detLength;
            outFile << " Q_length_NN="  << medianRndLengtPerRun / detLengthNN;
            outFile << std::endl;

            outFile << "[RND Avg]";
            outFile << " total_time="  << AvgRndTotalRuntimePerRun / micTOsec;
            outFile << " Q_total_time="  << AvgRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime);
            outFile << " Q_total_time_NN="  << AvgRndTotalRuntimePerRun / static_cast<double>(detTotalRuntimeNN);
            outFile << " astar_time="  << AvgRndAstarRuntimePerRun / micTOsec;
            outFile << " Q_astar_time="  << AvgRndAstarRuntimePerRun / static_cast<double>(detAstarRuntime);
            outFile << " Q_astar_time_NN="  << AvgRndAstarRuntimePerRun / static_cast<double>(detAstarRuntimeNN);
            outFile << " length="  << AvgRndLengthPerRun;
            outFile << " Q_length="  << AvgRndLengthPerRun / detLength;
            outFile << " Q_length_NN="  << AvgRndLengthPerRun / detLengthNN;
            outFile << std::endl;

            outFile << "RND success rate: " << (rndLocalSuccess * 100) / rndRepeats << "%" << std::endl;

            outFile.close();
        }
        if (sameRadius) {
            res.RndMinusTotalGlo = AvgRndTotalRuntimePerRun / micTOsec;
            res.RndMinusSearchGlo = AvgRndAstarRuntimePerRun / micTOsec;
            res.RndMinusLength = AvgRndLengthPerRun;
            res.RndMinusSuccess = (rndLocalSuccess * 100) / rndRepeats;
        } else {
            res.RndTotalGlo = AvgRndTotalRuntimePerRun / micTOsec;
            res.RndSearchGlo = AvgRndAstarRuntimePerRun / micTOsec;
            res.RndLength = AvgRndLengthPerRun;
            res.RndSuccess = (rndLocalSuccess * 100) / rndRepeats;
        }
    }
    if (RUNS > 1) {
            double TimeQualityAnVsRRT = GetMedian(medianTimeQualityAnVsRRT);
    double LenQualityAnVsRRT = GetMedian(medianLenQualityAnVsRRT);
    double TotalTimeQualityAnVsRND = GetMedian(medianTotalTimeQualityAnVsRND);
    double AstarTimeQualityAnVsRND = GetMedian(medianAstarTimeQualityAnVsRND);
    double LenQualityAnVsRND = GetMedian(medianLenQualityAnVsRND);
    avgTimeQualityAnVsRRT /= RUNS;
    avgLenQualityAnVsRRT /= RUNS;
    avgTotalTimeQualityAnVsRND /= RUNS;
    avgAstarTimeQualityAnVsRND /= RUNS;
    avgLenQualityAnVsRND /= RUNS;
    std::cout << "Map: " << env_fname << ", de = (" << delta << "," << epsilon << ")" << std::endl
              << "Testing in RobotCount=" << robotCount << " with Scenarios=" << RUNS << " and RRT,PRM averaged each on N=" << rndRepeats << "runs." << std::endl
              << "DET: it took " << detTotalAttempts << " attempts to get to " << RUNS << " successful runs (" <<
                  (RUNS * 100.0) / detTotalAttempts << "% rate)" << std::endl
              << "[Median]: RND median total-time quality=" << TotalTimeQualityAnVsRND << std::endl
              << "[Median] RND median astar-time quality=" << AstarTimeQualityAnVsRND << std::endl
              << "[Median] RND median length quality=" << LenQualityAnVsRND << std::endl
              << "[AVG]: RND Avg total-time quality=" << avgTotalTimeQualityAnVsRND << std::endl
              << "[AVG] RND Avg astar-time quality=" << avgAstarTimeQualityAnVsRND << std::endl
              << "[AVG] RND Avg length quality=" << avgLenQualityAnVsRND << std::endl
              << "RND success rate over all tests=" <<  (rndTotalSuccess * 100.0) / (rndRepeats * RUNS) << std::endl;
    }
}

///  compare An* to OMPL-PRM* (averaged on N runs), and report statistics
/// @param env_fname see file for a comprehensive list
/// @param testName this name will also appear in the test results
/// @param robotCount number of robots
/// @param xInit list of X values for the robots
/// @param yInit list of Y values for the robots
/// @param delta the delta from Dayan23
/// @param epsilon the epsilon from Dayan23
void compareAnstarToIncPRM(const std::string& env_fname, const std::string& testName, const std::string& folderBase,
                           int robotCount, std::vector<double> xInit, std::vector<double> yInit,
                           double delta, double epsilon) {
        std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    // 2D tests
    int RUNS = 1;
    if (robotCount == 6) RUNS = 1;
    int i = 0;
    auto micTOsec = 1e6;
    int detTotalAttempts = 0;

    double avgTimeQualityAnVsRRT = 0;
    double avgLenQualityAnVsRRT = 0;
    std::vector<double> medianTimeQualityAnVsRRT;
    std::vector<double> medianLenQualityAnVsRRT;

    int rndTotalSuccess = 0;
    // regular
    double avgTotalTimeQualityAnVsRND = 0;
    double avgAstarTimeQualityAnVsRND = 0;
    double avgLenQualityAnVsRND = 0;
    std::vector<double> medianTotalTimeQualityAnVsRND;
    std::vector<double> medianAstarTimeQualityAnVsRND;
    std::vector<double> medianLenQualityAnVsRND;
    int rndRepeats = 20;
    long detSampleCount = 0;
    while (i < RUNS) {
        fmt::print("\nStarting run #{}\n", i);
        // auto validStart = getRandomStartPoint(env_fname, robotCount);
        fmt::print("\nChose a starting point.\n");
        // std::vector<double> X;
        // std::vector<double> Y;
        // for (int j = 0; j < robotCount; ++j) {
            // X.push_back(validStart[2 * j]);
            // Y.push_back(validStart[2 * j + 1]);
        // }
        std::vector<double> X = xInit;
        std::vector<double> Y = yInit;
        detTotalAttempts++;
        // no NN
        auto [sampleCount, detLength, detClear, detConstructionTime, detAstarRuntime, detRadius] =
            iLPRM2D(ompl::geometric::ImplicitPRM::LatticeType::AnStar, env_fname, testName,
                    robotCount, X, Y, delta, epsilon, false);
        double detTotalRuntime = detConstructionTime + detAstarRuntime;
        double AvgRndTotalRuntime = 0;
        double AvgRndLength = 0;
        std::vector<double> medianRndTotalRuntime;
        std::vector<double> medianRndLength;
        int rndLocalSuccess = 0;
        std::cout << "runtime=" << detTotalRuntime << std::endl;
        std::cout << "fin_length=" << detLength << std::endl;

        double limitedRunTime = (static_cast<double>(detTotalRuntime) / static_cast<double>(micTOsec)) * 10;
        std::cout << "limiting to T=" << limitedRunTime << std::endl;
        for (int t = 0; t < rndRepeats; ++t) {
            std::cout << "Starting repeat #" << t << std::endl;
            auto [rndStates, rndLengthLoc, rndClear, rndConstructionTimeLoc, rndAstarRuntimeLoc] =
            incRndPRM2D(env_fname, testName, robotCount, X, Y, limitedRunTime);
            if (rndConstructionTimeLoc > 0) {
                AvgRndTotalRuntime += rndConstructionTimeLoc;
                medianRndTotalRuntime.push_back(rndConstructionTimeLoc);
            }
            if (rndLengthLoc > 0) {
                AvgRndLength += rndLengthLoc;
                medianRndLength.push_back(rndLengthLoc);
                rndLocalSuccess++;
            }
        }
        rndTotalSuccess += rndLocalSuccess;
        std::cout << "RND success rate: " << (rndLocalSuccess * 100) / rndRepeats << "%" << std::endl;
        double medianRndTotalRuntimePerRun = -1;
        double medianRndAstarRuntimePerRun = -1;
        double medianRndLengtPerRun = -1;
        double AvgRndTotalRuntimePerRun = -1;
        double AvgRndAstarRuntimePerRun = -1;
        double AvgRndLengthPerRun = -1;
        if (rndLocalSuccess > 0) {
            AvgRndTotalRuntimePerRun = ( AvgRndTotalRuntime * 1.0) / rndLocalSuccess;
            AvgRndLengthPerRun = (AvgRndLength * 1.0) / rndLocalSuccess;
            medianRndTotalRuntimePerRun = GetMedian(medianRndTotalRuntime);
            medianRndLengtPerRun = GetMedian(medianRndLength);

            avgTotalTimeQualityAnVsRND += AvgRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime);
            avgAstarTimeQualityAnVsRND += AvgRndAstarRuntimePerRun / static_cast<double>(detAstarRuntime);
            avgLenQualityAnVsRND += AvgRndLengthPerRun / detLength;
            medianTotalTimeQualityAnVsRND.push_back(medianRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime));
            medianAstarTimeQualityAnVsRND.push_back(medianRndAstarRuntimePerRun / static_cast<double>(detAstarRuntime));
            medianLenQualityAnVsRND.push_back(medianRndLengtPerRun / detLength);
        }
        i++;

        std::ofstream outFile;
        outFile.open(folderBase + testName + "compareAnstarToIncPRM.txt");
        std::cout << "Printing " << testName << std::endl;
        if (outFile.is_open()) {
            std::cout << "Printing started " << testName << std::endl;
            outFile << "Map: " << env_fname << ", de = (" << delta << "," << epsilon << ")" << std::endl
      << "Testing in RobotCount=" << robotCount << " with Scenarios=" << RUNS << " and RRT,PRM averaged each on N=" << rndRepeats << "runs." << std::endl;

            outFile << "An*: Regular";
            outFile << " Time=" << detTotalRuntime / micTOsec;
            outFile << " Length=" << detLength;
            outFile << std::endl;

            outFile << "[RND Median]";
            outFile << " total_time="  << medianRndTotalRuntimePerRun / micTOsec;
            outFile << " Q_total_time="  << medianRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime);
            outFile << " length="  << medianRndLengtPerRun;
            outFile << " Q_length="  << medianRndLengtPerRun / detLength;
            outFile << std::endl;

            outFile << "[RND Avg]";
            outFile << " total_time="  << AvgRndTotalRuntimePerRun / micTOsec;
            outFile << " Q_total_time="  << AvgRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime);
            outFile << " length="  << AvgRndLengthPerRun;
            outFile << " Q_length="  << AvgRndLengthPerRun / detLength;
            outFile << std::endl;

            outFile << "RND success rate: " << (rndLocalSuccess * 100) / rndRepeats << "%" << std::endl;

            outFile.close();
        }
    }
    if (RUNS > 1) {
        double TotalTimeQualityAnVsRND = GetMedian(medianTotalTimeQualityAnVsRND);
        double AstarTimeQualityAnVsRND = GetMedian(medianAstarTimeQualityAnVsRND);
        double LenQualityAnVsRND = GetMedian(medianLenQualityAnVsRND);
        avgTimeQualityAnVsRRT /= RUNS;
        avgLenQualityAnVsRRT /= RUNS;
        avgTotalTimeQualityAnVsRND /= RUNS;
        avgAstarTimeQualityAnVsRND /= RUNS;
        avgLenQualityAnVsRND /= RUNS;
        std::cout << "Map: " << env_fname << ", de = (" << delta << "," << epsilon << ")" << std::endl
                  << "Testing in RobotCount=" << robotCount << " with Scenarios=" << RUNS << " and PRM averaged each on N=" << rndRepeats << "runs." << std::endl
                  << "DET: it took " << detTotalAttempts << " attempts to get to " << RUNS << " successful runs (" <<
                      (RUNS * 100.0) / detTotalAttempts << "% rate)" << std::endl
                  << "[Median]: RND Avg total-time quality=" << TotalTimeQualityAnVsRND << std::endl
                  << "[Median] RND Avg length quality=" << LenQualityAnVsRND << std::endl
                  << "[AVG]: RND Avg total-time quality=" << avgTotalTimeQualityAnVsRND << std::endl
                  << "[AVG] RND Avg length quality=" << avgLenQualityAnVsRND << std::endl
                  << "RND success rate over all tests=" <<  (rndTotalSuccess * 100.0) / (rndRepeats * RUNS) << std::endl;
    }
}

///  compare An* to OMPL-PRM* (averaged on N runs), and report statistics
/// @param env_fname see file for a comprehensive list
/// @param testName this name will also appear in the test results
/// @param robotCount number of robots
/// @param xInit list of X values for the robots
/// @param yInit list of Y values for the robots
/// @param delta the delta from Dayan23
/// @param epsilon the epsilon from Dayan23
void compareAnstarToRRTstar(const std::string& env_fname, const std::string& testName, const std::string& folderBase,
                           int robotCount, std::vector<double> xInit, std::vector<double> yInit,
                           double delta, double epsilon) {
        std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    // 2D tests
    int RUNS = 1;
    if (robotCount == 6) RUNS = 1;
    int i = 0;
    auto micTOsec = 1e6;
    int detTotalAttempts = 0;

    double avgTimeQualityAnVsRRT = 0;
    double avgLenQualityAnVsRRT = 0;
    std::vector<double> medianTimeQualityAnVsRRT;
    std::vector<double> medianLenQualityAnVsRRT;

    int rndTotalSuccess = 0;
    // regular
    double avgTotalTimeQualityAnVsRND = 0;
    double avgAstarTimeQualityAnVsRND = 0;
    double avgLenQualityAnVsRND = 0;
    std::vector<double> medianTotalTimeQualityAnVsRND;
    std::vector<double> medianAstarTimeQualityAnVsRND;
    std::vector<double> medianLenQualityAnVsRND;
    int rndRepeats = 20;
    long detSampleCount = 0;
    while (i < RUNS) {
        fmt::print("\nStarting run #{}\n", i);
        // auto validStart = getRandomStartPoint(env_fname, robotCount);
        fmt::print("\nChose a starting point.\n");
        // std::vector<double> X;
        // std::vector<double> Y;
        // for (int j = 0; j < robotCount; ++j) {
            // X.push_back(validStart[2 * j]);
            // Y.push_back(validStart[2 * j + 1]);
        // }
        std::vector<double> X = xInit;
        std::vector<double> Y = yInit;
        detTotalAttempts++;
        // no NN
        auto [sampleCount, detLength, detClear, detConstructionTime, detAstarRuntime, detRadius] =
            // iLPRM2D(ompl::geometric::ImplicitPRM::LatticeType::AnStar, env_fname, testName,
            iNNLPRM2D(ompl::geometric::ImplicitPRM::LatticeType::AnStar, env_fname, testName,
                    robotCount, X, Y, delta, epsilon, false);
        double detTotalRuntime = detConstructionTime + detAstarRuntime;
        double AvgRndTotalRuntime = 0;
        double AvgRndLength = 0;
        std::vector<double> medianRndTotalRuntime;
        std::vector<double> medianRndLength;
        int rndLocalSuccess = 0;
        std::cout << "runtime=" << detTotalRuntime << std::endl;
        std::cout << "fin_length=" << detLength << std::endl;

        double limitedRunTime = (static_cast<double>(detTotalRuntime) / static_cast<double>(micTOsec));
        std::cout << "limiting to T=" << limitedRunTime << std::endl;
        for (int t = 0; t < rndRepeats; ++t) {
            std::cout << "Starting repeat #" << t << std::endl;
            auto [rndStates, rndLengthLoc, rndClear, rndConstructionTimeLoc, rndAstarRuntimeLoc] =
            RRT2D(env_fname, testName, robotCount, X, Y, limitedRunTime);
            if (rndConstructionTimeLoc > 0) {
                AvgRndTotalRuntime += rndConstructionTimeLoc;
                medianRndTotalRuntime.push_back(rndConstructionTimeLoc);
            }
            if (rndLengthLoc > 0) {
                AvgRndLength += rndLengthLoc;
                medianRndLength.push_back(rndLengthLoc);
                rndLocalSuccess++;
            }
        }
        rndTotalSuccess += rndLocalSuccess;
        std::cout << "RND success rate: " << (rndLocalSuccess * 100) / rndRepeats << "%" << std::endl;
        double medianRndTotalRuntimePerRun = -1;
        double medianRndAstarRuntimePerRun = -1;
        double medianRndLengtPerRun = -1;
        double AvgRndTotalRuntimePerRun = -1;
        double AvgRndAstarRuntimePerRun = -1;
        double AvgRndLengthPerRun = -1;
        if (rndLocalSuccess > 0) {
            AvgRndTotalRuntimePerRun = ( AvgRndTotalRuntime * 1.0) / rndLocalSuccess;
            AvgRndLengthPerRun = (AvgRndLength * 1.0) / rndLocalSuccess;
            medianRndTotalRuntimePerRun = GetMedian(medianRndTotalRuntime);
            medianRndLengtPerRun = GetMedian(medianRndLength);

            avgTotalTimeQualityAnVsRND += AvgRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime);
            avgAstarTimeQualityAnVsRND += AvgRndAstarRuntimePerRun / static_cast<double>(detAstarRuntime);
            avgLenQualityAnVsRND += AvgRndLengthPerRun / detLength;
            medianTotalTimeQualityAnVsRND.push_back(medianRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime));
            medianAstarTimeQualityAnVsRND.push_back(medianRndAstarRuntimePerRun / static_cast<double>(detAstarRuntime));
            medianLenQualityAnVsRND.push_back(medianRndLengtPerRun / detLength);
        }
        i++;

        std::ofstream outFile;
        outFile.open(folderBase + testName + "_compareAnstarToRRTstar.txt");
        std::cout << "Printing " << testName << std::endl;
        if (outFile.is_open()) {
            std::cout << "Printing started " << testName << std::endl;
            outFile << "Map: " << env_fname << ", de = (" << delta << "," << epsilon << ")" << std::endl
      << "Testing in RobotCount=" << robotCount << " with Scenarios=" << RUNS << " and RRT,PRM averaged each on N=" << rndRepeats << "runs." << std::endl;

            outFile << "An*: Regular";
            outFile << " Time=" << detTotalRuntime / micTOsec;
            outFile << " Length=" << detLength;
            outFile << std::endl;

            outFile << "[RND Median]";
            outFile << " total_time="  << medianRndTotalRuntimePerRun / micTOsec;
            outFile << " Q_total_time="  << medianRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime);
            outFile << " length="  << medianRndLengtPerRun;
            outFile << " Q_length="  << medianRndLengtPerRun / detLength;
            outFile << std::endl;

            outFile << "[RND Avg]";
            outFile << " total_time="  << AvgRndTotalRuntimePerRun / micTOsec;
            outFile << " Q_total_time="  << AvgRndTotalRuntimePerRun / static_cast<double>(detTotalRuntime);
            outFile << " length="  << AvgRndLengthPerRun;
            outFile << " Q_length="  << AvgRndLengthPerRun / detLength;
            outFile << std::endl;

            outFile << "RND success rate: " << (rndLocalSuccess * 100) / rndRepeats << "%" << std::endl;

            outFile.close();
        }
    }
    if (RUNS > 1) {
        double TotalTimeQualityAnVsRND = GetMedian(medianTotalTimeQualityAnVsRND);
        double AstarTimeQualityAnVsRND = GetMedian(medianAstarTimeQualityAnVsRND);
        double LenQualityAnVsRND = GetMedian(medianLenQualityAnVsRND);
        avgTimeQualityAnVsRRT /= RUNS;
        avgLenQualityAnVsRRT /= RUNS;
        avgTotalTimeQualityAnVsRND /= RUNS;
        avgAstarTimeQualityAnVsRND /= RUNS;
        avgLenQualityAnVsRND /= RUNS;
        std::cout << "Map: " << env_fname << ", de = (" << delta << "," << epsilon << ")" << std::endl
                  << "Testing in RobotCount=" << robotCount << " with Scenarios=" << RUNS << " and PRM averaged each on N=" << rndRepeats << "runs." << std::endl
                  << "DET: it took " << detTotalAttempts << " attempts to get to " << RUNS << " successful runs (" <<
                      (RUNS * 100.0) / detTotalAttempts << "% rate)" << std::endl
                  << "[Median]: RND Avg total-time quality=" << TotalTimeQualityAnVsRND << std::endl
                  << "[Median] RND Avg length quality=" << LenQualityAnVsRND << std::endl
                  << "[AVG]: RND Avg total-time quality=" << avgTotalTimeQualityAnVsRND << std::endl
                  << "[AVG] RND Avg length quality=" << avgLenQualityAnVsRND << std::endl
                  << "RND success rate over all tests=" <<  (rndTotalSuccess * 100.0) / (rndRepeats * RUNS) << std::endl;
    }
}

// int runSpecificTestOnInputSet(const std::string& env_fname, const std::string& testName, int robotCount,
//     std::vector<double> xInit, std::vector<double> yInit,
//     double delta, double epsilon, bool diffRadius = false) {
//     std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
//     // 2D tests
//     double highestLen, avgLen = 0;
//     double lowestLen = DBL_MAX;
//     int fails = 0;
//     double highestClear, avgClear = 0;
//     double lowestClear = DBL_MAX;
//     int betterLengthThanMine = 0;
//     int betterClearanceThanMine = 0;
//     // float avgTotalRuntime = 0, avgAstarRuntime = 0;
//     // auto RUNS = getHMapTestSet(x1Init, y1Init, x2Init, y2Init, x3Init, y3Init);
//     // int i = 0;
//     double avgLenQuality = 0;
//     double avgTotalTimeQuality = 0;
//     double avgAstarTimeQuality = 0;
//     double avg = 0;
//     // for (auto run: RUNS) {
//     int RUNS = 1;
//     int i = 0;
//     auto micTOsec = 1e6;
//     // for (int i = 0; i < RUNS; ++i) {
//     int AnFail = 0;
//     int RNDFail = 0;
//     while (i < RUNS) {
//         fmt::print("\nStarting run #{}\n", i);
//         auto validStart = getRandomStartPoint(env_fname, robotCount);
//         fmt::print("\nChose a starting point.\n");
//         // double x1 = run[0];
//         // double y1 = run[1];
//         // double x2 = run[2];
//         // double y2 = run[3];
//         // double x3 = run[4];
//         // double y3 =
//         std::vector<double> X = {validStart[0], validStart[2], validStart[4], validStart[6], validStart[8], validStart[10]};
//         std::vector<double> Y = {validStart[1], validStart[3], validStart[5], validStart[7], validStart[9], validStart[11]};
//         // double x1 = 20.8317;
//         // double y1 = 35.6572;
//         // double x2 = 7.18125;
//         // double y2 = -23.4666;
//         // double x3 = 37.3669;
//         // double y3 = 34.1791;
//         // double x1 = -3.034532903215009;
//         // double y1 = 33.98781599313389;
//         // double x2 = -37.56675603760405;
//         // double y2 = 35.23887985863031;
//         // double x3 = 11.797583997471293;
//         // double y3 = 31.303733233765726;
//         // double x1 = -3.7065935314402907;
//         // double y1 = -41.92242309732091;
//         // double x2 = -40.653390674725216;
//         // double y2 = 33.62042349169958;
//         // double x3 = 0;
//         // double y3 = 0;
//         // double x1 = 1;
//         // double y1 = -2;
//         // double x2 = 1;
//         // double y2 = 2;
//         // double x3 = -2;
//         // double y3 = 0;
//         // double x1 = 0;
//         // double y1 = 1.7;
//         // double x2 = 0;
//         // double y2 = -1.7;
//         // double x3 = 0;
//         // double y3 = 0;
//         // std::vector<double> X = xInit;
//         // std::vector<double> Y = yInit;
//         // int robotCount = 2;
//         // auto [states, rndLength, rndClear, rndTotalRuntime, rndAstarRuntime] =
//         // plan_with_maze_multi(i + 1, env_fname, testName, X, Y, 0, 0); // RRT, for RRT* use sampleCount=runtime
//         // return 0;
//
//         auto [sampleCount, detLength, detClear, detTotalRuntime, detAstarRuntime, detRadius] =
//             LPRM_2D_multi(ompl::geometric::ImplicitPRM::LatticeType::AnStar, env_fname, testName,
//             robotCount, X, Y, delta, epsilon, og::ImplicitPRM::Lattice, false, false, false);
//         if (sampleCount == -1) continue;
//         auto [sampleCount2, detLength2, detClear2, detTotalRuntime2, detAstarRuntime2, detRadius2] =
//         LPRM_2D_multi(ompl::geometric::ImplicitPRM::LatticeType::DnStar, env_fname, testName,
//         robotCount, X, Y, delta, epsilon, og::ImplicitPRM::Lattice, false, false, false);
//         if (sampleCount2 == -1) continue;
//         auto [sampleCount3, detLength3, detClear3, detTotalRuntime3, detAstarRuntime3, detRadius3] =
//         LPRM_2D_multi(ompl::geometric::ImplicitPRM::LatticeType::Zn, env_fname, testName,
//         robotCount, X, Y, delta, epsilon, og::ImplicitPRM::Lattice, false, false, false);
//         if (sampleCount3 == -1) continue;
//         std::cout << "samples: " << sampleCount << std::endl;
//         std::cout << detTotalRuntime << " || " << detAstarRuntime << std::endl;
//         // return 0;
//         // if (sampleCount == -1) return 0;
//         i++;
//         // int sampleCount = 20000;
//         std::cout << "construction:";
//         std::cout << " detAn*=" << detTotalRuntime / micTOsec;
//         std::cout << " detDn*=" << detTotalRuntime2 / micTOsec;
//         std::cout << " detZn="  << detTotalRuntime3 / micTOsec;
//         std::cout << std::endl;
//         std::cout << "A*:";
//         std::cout << " detAn*=" << detAstarRuntime / micTOsec;
//         std::cout << " detDn*=" << detAstarRuntime2 / micTOsec;
//         std::cout << " detZn="  << detAstarRuntime3 / micTOsec;
//         std::cout << std::endl;
//         return 0;
//         try {
//             double radius = detRadius;
//             int rndFails = 0;
//             double constructionAvg = 0;
//             double astarAvg = 0;
//             double rndLengthAvg = 0;
//             if (diffRadius)
//                 radius = 2 * std::pow((1.0 + (1.0/6.0)) * (std::log(sampleCount) / sampleCount), 1.0 / 6.0) * 110;
//             // double radius = detRadius;
//             int rndRepeats = 1;
//             // int rndRepeats = 20;
//             for (int t = 0; t < rndRepeats; ++t) {
//                 auto [states, rndLength, rndClear, rndTotalRuntime, rndAstarRuntime] =
//                     PRM_implicit_2D_multi(i + 1, env_fname, testName,
//                     robotCount, X, Y, sampleCount, radius, detTotalRuntime / micTOsec); // RRT, for RRT* use sampleCount=runtime
//                     // robotCount, x1, y1, x2, y2, x3, y3, 11000000, radius); // RRT, for RRT* use sampleCount=runtime
//                 if (rndLength < 0) {
//                     rndFails++;
//                     // std::cout << "construction:";
//                     // std::cout << " rnd=FAIL" << std::endl;
//                     // std::cout << "A*:";
//                     // std::cout << " rnd=FAIL" << std::endl;
//                     continue;
//                     // return 0;
//                 }
//                 constructionAvg += rndTotalRuntime / micTOsec;
//                 astarAvg += rndAstarRuntime / micTOsec;
//                 rndLengthAvg += rndLength;
//                 double ratio = (static_cast<double>(rndAstarRuntime) / static_cast<double>(detAstarRuntime));
//                 if (ratio > 3 ) {
//                     std::cout << "GOT A GOOD DIFF OF " << ratio << std::endl;
//                 } else {
//                     std::cout << "got a ratio of: " << ratio << std::endl;
//                 }
//                 avgAstarTimeQuality += ratio;
//
//                 // return 0;
//                 // std::cout << "det=" << detLength << ", rnd=" << rndLength << std::endl;
//                 // std::cout << "RND run #:" << t << std::endl;
//                 // std::cout << "construction:";
//                 // std::cout << " rnd=" << rndTotalRuntime / micTOsec << std::endl;
//                 // std::cout << "A*:";
//                 // std::cout << " rnd=" << rndAstarRuntime / micTOsec << std::endl;
//             }
//             double rndSuccessRate = (1 - ((rndFails * 1.0) / rndRepeats)) * 100;
//             // std::cout << rndSuccessRate << "% success";
//             if (rndFails < rndRepeats) {
//                 constructionAvg /= rndRepeats;
//                 astarAvg /= rndRepeats;
//                 rndLengthAvg /= rndRepeats;
//             } else {
//                 constructionAvg = -1;
//                 astarAvg = -1;
//                 rndLengthAvg = -1;
//             }
//             fmt::print("Map: {}, Problem: {}-swap, de=({},{})\n", testName, robotCount, delta, epsilon);
//             // if (robotCount == 2) {
//             //     fmt::print("r1:({},{})->({},{}), r2:({},{})->({},{})\n",
//             //     x1, y1, x2, y2, x2, y2, x1, y1);
//             // } else if (robotCount == 3) {
//             //     fmt::print("r1:({},{})->({},{}), r2:({},{})->({},{}), r3:({},{})->({},{})\n",
//             //     x1, y1, x2, y2, x2, y2, x3, y3, x3, y3, x1, y1);
//             // }
//             // fmt::print("{},{},{},{},", detTotalRuntime / micTOsec, detTotalRuntime2 / micTOsec,
//             // detTotalRuntime3 / micTOsec, constructionAvg);
//             // fmt::print("{},{},{},{},", detAstarRuntime / micTOsec, detAstarRuntime2 / micTOsec,
//             // detAstarRuntime3 / micTOsec, astarAvg);
//             fmt::print("{},{},", detAstarRuntime / micTOsec, astarAvg); // compare only An* to RND
//             // fmt::print("{},{},{},{},", detLength, detLength2,
//             //     detLength3, rndLengthAvg);
//             fmt::print("{},", rndSuccessRate);
//             // return 0;
//             // if (rndLength < detLength) betterLengthThanMine++;
//             // if (rndClear > detClear) betterClearanceThanMine++;
//             // lowestLen = std::min(lowestLen, rndLength);
//             // highestLen = std::max(highestLen, rndLength);
//             // avgLen += rndLength;
//             // lowestClear = std::min(lowestClear, rndClear);
//             // highestClear = std::max(highestClear, rndClear);
//             // avgClear += rndClear;
//             // avgLenQuality += (detLength / rndLength);
//             // avgTotalTimeQuality += (static_cast<double>(detTotalRuntime + detAstarRuntime) / static_cast<double>(rndTotalRuntime + rndAstarRuntime));
//             // avgAstarTimeQuality += (static_cast<double>(detAstarRuntime) / static_cast<double>(rndAstarRuntime));
//         } catch (...) {
//             fails++;
//         }
//     }
//     std::cout << "Avg ratio of: " << avgAstarTimeQuality / RUNS << std::endl;
//     return 0;
//     // auto successful = static_cast<double>(RUNS.size() - fails);
//     auto successful = static_cast<double>(RUNS - fails);
//     if (successful == 0) {
//         // fmt::print("All {} runs failed", RUNS.size());
//         fmt::print("All {} runs failed", RUNS);
//     } else {
//         avgLen /= (successful * 1.0);
//         avgLenQuality /= (successful * 1.0);
//         avgClear /= (successful * 1.0);
//         avgTotalTimeQuality /= (successful * 1.0);
//         avgAstarTimeQuality /= (successful * 1.0);
//         // double successfulPer = (successful * 100.0) / static_cast<double>(RUNS.size());
//         double successfulPer = (successful * 100.0) / static_cast<double>(RUNS);
//         double betterLengthThanMinePer = (betterLengthThanMine * 100.0) / successful;
//         double betterClearanceThanMinePer = (betterClearanceThanMine * 100.0) / successful;
//         fmt::print("/******** RESULTS of {} *******/\n", testName);
//         // fmt::print("Ran {} runs of PRM, comparing det to rnf sample sets, with det's radius, with the following results.\n", RUNS.size());
//         fmt::print("Ran {} runs of PRM, comparing det to rnf sample sets, with det's radius, with the following results.\n", RUNS);
//         fmt::print("{}% of the runs were successful.\n", successfulPer);
//         fmt::print("time quality: total={}, astar={}\n", avgTotalTimeQuality, avgAstarTimeQuality);
//         fmt::print("length quality: {}\n", avgLenQuality);
//         fmt::print("{}% of the runs found a shorter path than mine \n", betterLengthThanMinePer);
//         fmt::print("{}% of the runs found a path with a higher clearance than mine \n", betterClearanceThanMinePer);
//         fmt::print("/******** END RESULTS *******/\n", testName);
//     }
//
//     return 0;
// }

/// Compare selected lattices to eachother, across various test scenarios
void compareLatticesToEachother(const std::string& folderBase) {
    try {
        std::vector<LatticeType> types{LatticeType::Zn, LatticeType::AnStar, LatticeType::Lc1, LatticeType::Lc1b, LatticeType::Lc2};
        std::vector<std::string> results;
        for (const auto& test: getTests()) {
            std::string testFullName = test.setName + test.testName;
            std::cout << "Started test=" << testFullName << std::endl;
            results.push_back(fmt::format("Test results for [{}]:", testFullName));
            auto X = test.X;
            auto Y = test.Y;
            bool found = false;
            if (test.random) {
                // random tests
                auto validStart = getRandomStartPoint(test.map, test.robotCount);
                X.clear();
                Y.clear();
                for (int j = 0; j < test.robotCount; ++j) {
                    X.push_back(validStart[2 * j]);
                    Y.push_back(validStart[2 * j + 1]);
                }
            }
            for (const auto& type: types) {
                auto [sampleCount, detLength, detClear, detConstruction,
                    detAstarRuntime, detRadius] = iLPRM2D(type, test.map,
                        testFullName, test.robotCount, X, Y, test.delta, 10, false);
                double tosec = 1000000.0;
                std::string resStr = fmt::format("Lattice={}, Total Time={}, A* Time={}, totalL={}",
                    ompl::geometric::ImplicitPRM::latticeString(type), (detConstruction+detAstarRuntime)/tosec,
                    detAstarRuntime/tosec, detLength);
                results.push_back(resStr);
            }
            std::cout << "Ended test=" << testFullName << std::endl;
            results.push_back(fmt::format("End of results for [{}].\n###############################", testFullName));

        }
        std::ofstream outFile;

        // create dir
        // std::string path = folderBase + "/compareLatticesToEachother/";
        std::filesystem::create_directories(folderBase);

        // open file and write
        outFile.open(folderBase + "comparing_lattices.txt");
        if (outFile.is_open()) {
            for (const auto& line: results) {
                outFile << line << std::endl;
            }
            outFile.close();
        }
    } catch (...) {
        std::cout << "ERROR! BEEP BOOP" << std::endl;
    }
}

/// Get the best delta value (in terms of planning runtime) for a specific lattice
std::vector<std::string> calibrateLatticeInner(ompl::geometric::ImplicitPRM::LatticeType type) {
    std::vector<std::string> res;
    double tosec = 1000000.0;

    try {
        for (const auto& test: getTests()) {
            std::string testFullName = test.testName + test.setName;
            std::cout << "Started test=" << testFullName << std::endl;
            auto X = test.X;
            auto Y = test.Y;
            // tests can request to be random
            if (test.random) {
                // random tests
                auto validStart = getRandomStartPoint(test.map, test.robotCount);
                X.clear();
                Y.clear();
                for (int j = 0; j < test.robotCount; ++j) {
                    X.push_back(validStart[2 * j]);
                    Y.push_back(validStart[2 * j + 1]);
                }
            }
            double bestT = 100000.0;
            double bestD = 0.0;
            double currD = test.delta;
            fmt::print("Calibration run for test={}, lattice={}\n", test.setName, "An*");
            int deltaI = 0;
            // calibration runs: increasing delta. easier, runs get faster or reach a deadend quicker.
            while (deltaI < 30) {
                fmt::print("########## Delta={} ##########\n", currD);
                currD = test.delta + test.resolution * deltaI;
                deltaI++;
                auto [sampleCount, detLength, detClear, detConstruction, detAstarRuntime, detRadius] =
                iLPRM2D(type, test.map, testFullName,
                test.robotCount, X, Y, currD, 10, false);
                if (detAstarRuntime < 0) continue;
                double totalRuntimeSec = (detConstruction+detAstarRuntime)/tosec;
                if (totalRuntimeSec > 0 && totalRuntimeSec < bestT) {
                    bestT = totalRuntimeSec;
                    bestD = currD;
                }
            }
            deltaI = 0;
            // calibration runs: decreasing delta. runs take longer and longer, in general (so we don't test 30, too long).
            while (deltaI > -15) {
                fmt::print("########## Delta={} ##########\n", currD);
                currD = test.delta + test.resolution * deltaI;
                auto [sampleCount, detLength, detClear, detConstruction, detAstarRuntime, detRadius] =
                iLPRM2D(type, test.map, testFullName,
                test.robotCount, X, Y, currD, 10, false);
                double totalRuntimeSec = (detConstruction+detAstarRuntime)/tosec;
                if (totalRuntimeSec > 0 && totalRuntimeSec < bestT) {
                    bestT = totalRuntimeSec;
                    bestD = currD;
                }
                deltaI--;
            }
            res.push_back(fmt::format("[{}]: Calibration  run for test={}{} ENDED. Best delta={} with T={}",
                type, test.setName, test.testName, bestD, bestT));
        }
    } catch (...) {
        std::cout << "ERROR! BEEP BOOP" << std::endl;
    }

    return res;
}

/// Get the best delta value (in terms of planning runtime) for a set of chosen lattices
void calibrateLattices(const std::string& folderBase) {
    std::vector<std::string> res;

    for (const auto& type: {ompl::geometric::ImplicitPRM::AnStar, ompl::geometric::ImplicitPRM::Lc2 }) {
        std::vector<std::string> resTmp = calibrateLatticeInner(type);
        for (const auto& elem: resTmp) {
            res.push_back(elem);
        }
    }

    // print results
    // create dir
    std::ofstream outFile;
    std::filesystem::create_directories(folderBase);
    outFile.open(folderBase + "calibrateLattices.txt");
    std::cout << "Printing lattice results" << std::endl;
    if (outFile.is_open()) {
        for (const auto& elem: res) {
            std::cout << elem << std::endl;
            outFile << elem << std::endl;
        }
    }
}

/// An* vs PRM*/PRM tests
void AnstarVSImplicitPRM(const std::string& folderBase) {
    try {
        auto anstarVsRND_tests = getTests();

        std::string resultCsv = folderBase + "results.csv";
        std::filesystem::create_directories(folderBase);
        std::ofstream outCSVFile(resultCsv, std::ios::app);
        outCSVFile << fmt::format("{},{},{},{},{},{},{},{},{},{},{},{},\n",
            "", "Total (s)", "Total (s)", "Total (s)", "Total (s)",
            "Search(S)","Search(S)","Search(S)",
            "Len", "Len", "Success (%)", "Success (%)");
        outCSVFile << fmt::format("{},{},{},{},{},{},{},{},{},{},{},{},\n",
            "Scenario (Robot #)", "An* LOC", "An* GLO", "RND GLO", "RND- GLO",
            "An* GLO", "RND GLO", "RND- GLO",
            "RND GLO", "RND- GLO", "RND GLO", "RND- GLO");

        for (const auto& test: anstarVsRND_tests) {
            AnstarVsRndTestResult res{};
            std::cout << "Started test=" << test.setName + test.testName <<  std::endl;

            compareAnStarToOthers(test.map, folderBase, test.setName, test.testName + "_SAME_R",
                test.robotCount, test.X, test.Y, test.delta, 10, res, true);
            compareAnStarToOthers(test.map, folderBase, test.setName, test.testName,
                test.robotCount, test.X, test.Y, test.delta, 10, res, false);
            // add to the results excel
            // std::string resultCsv = "TestResult/results.csv";
            // std::ofstream outCSVFile(resultCsv, std::ios::app);
            res.RndLength /= res.AnstarLength;
            res.RndMinusLength /= res.AnstarLength;
            outCSVFile << fmt::format("{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{},{},\n",
                fmt::format("{} ({})", test.setName + test.testName, test.robotCount),
                res.AnstarTotalLoc, res.AnstarTotalGlo, res.RndTotalGlo, res.RndMinusTotalGlo,
                res.AnstarSearchGlo, res.RndSearchGlo, res.RndMinusSearchGlo,
                res.RndLength, res.RndMinusLength,
                res.RndSuccess, res.RndMinusSuccess);
            std::cout << "Ended test=" << test.setName + test.testName << std::endl;
        }
    } catch (...) {
        std::cout << "ERROR! BEEP BOOP" << std::endl;
    }
}

/// An* vs OMPL-PRM* tests
void AnstarVSprmstar(const std::string& folderBase) {
    try {


        for (const auto& test: getTests()) {
            std::string testFullName = test.setName + test.testName;

            std::cout << "Started test=" << testFullName << std::endl;
            // compareAnstarToIncPRM(test.map, testFullName + "_PRMStar", folderBase,
            // test.robotCount, test.X, test.Y, test.delta, 10);
            compareAnstarToRRTstar(test.map, testFullName + "_RRTStar", folderBase,
                test.robotCount, test.X, test.Y, test.delta, 10);
            std::cout << "Ended test=" << testFullName << std::endl;
        }
    } catch (...) {
        std::cout << "ERROR! BEEP BOOP" << std::endl;
    }
}

#pragma endregion experiment_funcs

// handling general, uncaught, exceptions
void handler(int sig) {
    void *array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

// run any test you'd like here. It creates a test folder (named by current date/time data), and saves the results there.
int main(int argc, char **) {
    signal(SIGSEGV, handler);   // install our handler

    // Available maps: H_planar_env, UniqueSolutionMaze_env, RandomPolygons_planar_env, kenny_env, BugTrap_planar_env, BoundingBox_planar_env,
    // BoundingBox_for_four, BugTrap_planar_env, room1

    std::string folderBase = getUniqueTestFolder();

    // AnstarVSprmstar(folderBase); // old, no for use
    /// Comparative test sets
    compareLatticesToEachother(folderBase);
    AnstarVSImplicitPRM(folderBase);
    calibrateLattices(folderBase);

    /// Vamp tests
    VAMPTests::runVampTests(argc, folderBase);
    /// counting samples
    findNumberOfSamples();
}
