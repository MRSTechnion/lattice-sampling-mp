//#include "pch.h"
#include "../include/NonOptimalPlanner.h"
#include "../Samplers.h"

//void DrawPath2();

//ob::ValidStateSamplerPtr NonOptimalPlanner::allocMyValidStateSampler(const ob::SpaceInformation* si)
//{
//    return std::make_shared<MyValidStateSampler>(si);
//}

/*
void NonOptimalPlanner::plan(int workspaceIndex, int samplesetIndex, int plannerIndex, int timeout)
{
    logger("Setting up the state space.");

    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    // set the bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker(MyValidStateSampler::isStateValid);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = 0, start[1] = 0.51;

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = 0, goal[1] = 0.24;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // set sampler (optional; the default is uniform sampling)
    if (samplesetIndex == 1)
    {
    }
    switch (samplesetIndex)
    {
    case 1:
        logger("Using PRM with random sampling.");
        // use my-sampler
        ss.getSpaceInformation()->setValidStateSamplerAllocator(NonOptimalPlanner::allocMyValidStateSampler);
        break;
    default:
        break;
    }

    // create a planner for the defined space
    auto planner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // attempt to solve the problem within ten seconds of planning time
    logger(std::format("Attempting to plan within {0} seconds", timeout).c_str());
    std::ostringstream strCout;
    std::ostringstream strCerr;
    std::streambuf* oldCoutStreamBuf = std::cout.rdbuf();
    std::streambuf* oldCerrStreamBuf = std::cerr.rdbuf();
    std::cout.rdbuf(strCout.rdbuf());
    std::cerr.rdbuf(strCerr.rdbuf());
    // solve
    ompl::base::PlannerStatus status = ss.solve(timeout);
    // return output to normal
    std::cout.rdbuf(oldCoutStreamBuf);
    std::cout.rdbuf(oldCerrStreamBuf);

    loggerOMPL(strCout, strCerr);
    if (status)
    {
        logger("Found solution:");
        // print the path to screen 
        std::ostringstream ossSolution;
        ss.getSolutionPath().printAsMatrix(ossSolution);
        logger(std::format("Found solution (in X Y pairs):\n{0}", ossSolution.str()).c_str());

        // save it to file for python to draw a path
        std::ofstream ofsSolution("solution.txt", std::ofstream::out);
        ofsSolution << ossSolution.str();
        ofsSolution.close();

        logger("Exporting solution PNG with Python.");
        //DrawPath();
        logger("Finished exporting solution PNG.");
    }
    else
        logger("No solution found.");
}
*/

void NonOptimalPlanner::plan(PlanData pData) 
{
    // plan in SE2
    //logger("Non-Optimal Planner start.");

    ompl::app::SE2RigidBodyPlanning setup;

    logger("Loading DAE files");
    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/robot1.dae";
    std::string map_fname = getMapName(pData.mapIndex);
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + std::format("/2D/{0}", map_fname);
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);
    // infer bounds from the environment we added
    setup.inferEnvironmentBounds();

    auto si(setup.getSpaceInformation());
    auto svc(setup.allocStateValidityChecker(si, setup.getGeometricStateExtractor(), false));
    si->setStateValidityChecker(svc);
    si->setup();

    logger("Setting states");

    // define starting state
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(setup.getSpaceInformation());
    start->setX(pData.s_x);
    start->setY(pData.s_y);

    // define goal state
    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(start);
    goal->setX(pData.e_x);
    goal->setY(pData.e_y);

    // set the start & goal states
    //setup.setStartAndGoalStates(start, goal);
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);

    // Create a planner
    ob::PlannerPtr nonOptimizingPlanner = this->allocatePlanner(si, pData.plannerIndex);
    ob::ValidStateSamplerPtr samplerAllocator = allocateSampler(si.get());
    setup.getSpaceInformation()->setValidStateSamplerAllocator(NonOptimalPlanner::allocateSampler);
    //auto planner(std::make_shared<og::PRM>(setup.getSpaceInformation()));
    //setup.setPlanner(nonOptimizingPlanner);
    nonOptimizingPlanner->setProblemDefinition(pdef);
    nonOptimizingPlanner->setup();

    logger("starting SE2 simulation");
    // attempt to solve the problem, and print it to screen if a solution is found
    std::ostringstream strCout;
    std::ostringstream strCerr;
    std::streambuf* oldCoutStreamBuf = std::cout.rdbuf();
    std::streambuf* oldCerrStreamBuf = std::cerr.rdbuf();
    std::cout.rdbuf(strCout.rdbuf());
    std::cerr.rdbuf(strCerr.rdbuf());
    // solve
    //ompl::base::PlannerStatus status = setup.solve();
    ob::PlannerStatus status = nonOptimizingPlanner->solve(pData.runTime);
    // return output to normal
    std::cout.rdbuf(oldCoutStreamBuf);
    std::cout.rdbuf(oldCerrStreamBuf);

    loggerOMPL(strCout, strCerr);
    if (status)
    {
        logger("Found solution:");
        // print the path to screen 
        std::ostringstream ossSolution;
        setup.getSolutionPath().printAsMatrix(ossSolution);
        logger(std::format("Found solution (in X Y pairs):\n{0}", ossSolution.str()).c_str());

        // save it to file for python to draw a path
        std::filesystem::create_directory(pData.OutputID);
        std::string outputFile = std::format("{0}/solution.txt", pData.OutputID);
        std::ofstream ofsSolution(outputFile, std::ofstream::out);
        ofsSolution << ossSolution.str();
        ofsSolution.close();

        logger("Exporting solution images with Python.");
        DrawPath(map_fname.data(), pData.OutputID);
        logger("Finished exporting solution images.");
    }

    //setup.getSolutionPath().print(std::cout);
    logger("finished SE2 simulation");
}

ob::PlannerPtr NonOptimalPlanner::allocatePlanner(const ob::SpaceInformationPtr& si, int plannerType)
{
    std::string chosenMsg;
    switch (plannerType)
    {
    case 0:
        logger("Chosen planner: PRM.");
        return std::make_shared<og::PRM>(si);
        break;
    case 2:
        logger("Chosen planner: RRT.");
        return std::make_shared<og::RRT>(si);
        break;
    case 4:
        logger("Chosen planner: BKPIECE1.");
        return std::make_shared<og::BKPIECE1>(si);
        break;
    default:
    {
        logger("Planner-type enum is not implemented in allocation function.");
        return ob::PlannerPtr(); // Address compiler warning re: no return value.
        break;
    }
    }
}

ob::ValidStateSamplerPtr  NonOptimalPlanner::allocateSampler(const ob::SpaceInformation* si)
{
    switch (2)
    {
    //case 1:
    //    //logger("Chosen sampler: SG.");
    //    return std::make_shared<StaggeredGrid>(si);// , 3, 0.1);
    //    break;
    default:
    {
        //logger("Planner-type enum is not implemented in allocation function.");
        return ob::ValidStateSamplerPtr(); // Address compiler warning re: no return value.
        break;
    }
    }
}

/*void DrawPath()
{
    Py_Initialize();
    PyRun_SimpleString("import sys");
    //PyRun_SimpleString(R"(sys.path = ['C:\\Users\\Itai\\PycharmProjects\\OMPL_MRMP_Draw\\venv\\lib\\site-packages', 'C:\\Users\\Itai\\AppData\\Local\\Programs\\Python\\Python310\\lib'])");
    PyRun_SimpleString(R"(sys.path =['.\libs'])");
    FILE* fp = fopen("Draw_2D_Solution.py", "r");
    PyRun_AnyFile(fp, "Draw_2D_Solution.py");
    //Py_Finalize();
}*/
