/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Ryan Luna, Henning Kayser */

// #include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/tools/config/SelfConfig.h"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <boost/foreach.hpp>
#include <queue>

#include "GoalVisitor.hpp"

#include "../include/ImplicitRandomPRM.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/tools/config/MagicConstants.h>

namespace ob = ompl::base;

#define foreach BOOST_FOREACH

namespace ompl
{
    namespace magic
    {
        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS_LAZY = 5;

        /** \brief When optimizing solutions with lazy planners, this is the minimum
            number of path segments to add before attempting a new optimized solution
            extraction */
        static const unsigned int MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION = 5;
    }
}

ompl::geometric::ImplicitRandomPRM::ImplicitRandomPRM(const base::SpaceInformationPtr &si, bool starStrategy)
  : base::Planner(si, "LazyPRM")
  , starStrategy_(starStrategy)
  , indexProperty_(boost::get(boost::vertex_index_t(), g_))
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , weightProperty_(boost::get(boost::edge_weight, g_))
  , vertexComponentProperty_(boost::get(vertex_component_t(), g_))
  , vertexValidityProperty_(boost::get(vertex_flags_t(), g_))
  , edgeValidityProperty_(boost::get(edge_flags_t(), g_))
  , robotCount_(1)
  , maxEdge_(0.0)
  , mapExtent_(this->si_->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(0) // first SE(2)
                                      ->as<ob::CompoundStateSpace>()->getSubspace(0) // R^d component
                                      ->as<ob::RealVectorStateSpace>()->getBounds()) // get bounds
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;

    dimRd_ = mapExtent_.low.size();
    angleFix_ = 1;
    workState_ = si_->allocState();


    Planner::declareParam<double>("range", this, &ImplicitRandomPRM::setRange, &ImplicitRandomPRM::getRange, "0.:1.:10000.");
    if (!starStrategy_)
        Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &ImplicitRandomPRM::setMaxNearestNeighbors,
                                            std::string("8:1000"));

    addPlannerProgressProperty("iterations INTEGER", [this]
                               {
                                   return getIterationCount();
                               });
    addPlannerProgressProperty("best cost REAL", [this]
                               {
                                   return getBestCost();
                               });
    addPlannerProgressProperty("milestone count INTEGER", [this]
                               {
                                   return getMilestoneCountString();
                               });
    addPlannerProgressProperty("edge count INTEGER", [this]
                               {
                                   return getEdgeCountString();
                               });
}

ompl::geometric::ImplicitRandomPRM::ImplicitRandomPRM(const base::PlannerData &data, bool starStrategy)
  : ImplicitRandomPRM(data.getSpaceInformation(), starStrategy)
{
    if (data.numVertices() > 0)
    {
        // mapping between vertex id from PlannerData and Vertex in Boost.Graph
        std::map<unsigned int, Vertex> vertices;
        // helper function to create vertices as needed and update the vertices mapping
        const auto &getOrCreateVertex = [&](unsigned int vertex_index) {
            if (!vertices.count(vertex_index))
            {
                const auto &data_vertex = data.getVertex(vertex_index);
                Vertex graph_vertex = boost::add_vertex(g_);
                stateProperty_[graph_vertex] = si_->cloneState(data_vertex.getState());
                vertexValidityProperty_[graph_vertex] = VALIDITY_UNKNOWN;
                unsigned long int newComponent = componentCount_++;
                vertexComponentProperty_[graph_vertex] = newComponent;
                vertices[vertex_index] = graph_vertex;
            }
            return vertices.at(vertex_index);
        };

        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });

        for (size_t vertex_index = 0; vertex_index < data.numVertices(); ++vertex_index)
        {
            Vertex m = getOrCreateVertex(vertex_index);
            std::vector<unsigned int> neighbor_indices;
            data.getEdges(vertex_index, neighbor_indices);
            for (const unsigned int neighbor_index : neighbor_indices)
            {
                Vertex n = getOrCreateVertex(neighbor_index);
                base::Cost weight;
                data.getEdgeWeight(vertex_index, neighbor_index, &weight);
                const Graph::edge_property_type properties(weight);
                const Edge &edge = boost::add_edge(m, n, properties, g_).first;
                edgeValidityProperty_[edge] = VALIDITY_UNKNOWN;
                uniteComponents(m, n);
            }
            nn_->add(m);
        }
    }
}

ompl::geometric::ImplicitRandomPRM::~ImplicitRandomPRM()
{
    si_->freeState(workState_);
    clear();
};

static std::string EigenToString(const Eigen::VectorXd& v){
    std::string res = "";
    for (int i = 0; i < v.size(); ++i) {
        int valint = std::round(v(i) * pow(10, 6));
        double val = valint / pow(10, 6);
        res += std::to_string(val) + ",";
    }
    return res;
}

static std::string EigenToString(const Eigen::VectorXi& v){
    std::string res = "";
    for (int i = 0; i < v.size(); ++i) {
        int valint = std::round(v(i) * pow(10, 6));
        double val = valint / pow(10, 6);
        res += std::to_string(val) + ",";
    }
    return res;
}

void ompl::geometric::ImplicitRandomPRM::changeAngleToQuaternion(double u1, double u2, double u3, ob::SE3StateSpace::StateType* state) {
    // first create the quaternion (similar to the nasa one, but different order. from ompl_app.py).
    state->rotation().w = std::cos(0.5*u1) * std::cos(0.5*u2) * std::cos(0.5*u3) -
        std::sin(0.5*u1) * std::sin(0.5*u2) * std::sin(0.5*u3);
    state->rotation().x = std::sin(0.5*u1) * std::cos(0.5*u2) * std::cos(0.5*u3) +
        std::cos(0.5*u1) * std::sin(0.5*u2) * std::sin(0.5*u3);
    state->rotation().y = std::cos(0.5*u1) * std::sin(0.5*u2) * std::cos(0.5*u3) -
        std::sin(0.5*u1) * std::cos(0.5*u2) * std::sin(0.5*u3);
    state->rotation().z = std::cos(0.5*u1) * std::cos(0.5*u2) * std::sin(0.5*u3) +
        std::sin(0.5*u1) * std::sin(0.5*u2) * std::cos(0.5*u3);
}

bool ompl::geometric::ImplicitRandomPRM::LazyAstar() {
    return true;
}

void ompl::geometric::ImplicitRandomPRM::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    }
    if (!connectionFilter_)
        connectionFilter_ = [](const Vertex &, const Vertex &)
        {
            return true;
        };

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            if (!starStrategy_)
                opt_->setCostThreshold(opt_->infiniteCost());
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    sampler_ = si_->allocStateSampler();
}

void ompl::geometric::ImplicitRandomPRM::setRange(double distance)
{
    maxDistance_ = distance;
    if (!userSetConnectionStrategy_)
        setDefaultConnectionStrategy();
    if (isSetup())
        setup();
}

void ompl::geometric::ImplicitRandomPRM::setMaxNearestNeighbors(unsigned int k)
{
    if (starStrategy_)
        throw Exception("Cannot set the maximum nearest neighbors for " + getName());
    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    }
    if (!userSetConnectionStrategy_)
        connectionStrategy_ = KBoundedStrategy<Vertex>(k, maxDistance_, nn_);
    if (isSetup())
        setup();
}

void ompl::geometric::ImplicitRandomPRM::setDefaultConnectionStrategy()
{
    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    }

    if (starStrategy_)
        connectionStrategy_ = KStarStrategy<Vertex>([this] { return milestoneCount(); }, nn_, si_->getStateDimension());
    else
        connectionStrategy_ = KBoundedStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS_LAZY, maxDistance_, nn_);
}

void ompl::geometric::ImplicitRandomPRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::ImplicitRandomPRM::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::ImplicitRandomPRM::clearValidity()
{
    foreach (const Vertex v, boost::vertices(g_))
        vertexValidityProperty_[v] = VALIDITY_UNKNOWN;
    foreach (const Edge e, boost::edges(g_))
        edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
}

void ompl::geometric::ImplicitRandomPRM::clear()
{
    Planner::clear();
    freeMemory();
    if (nn_)
        nn_->clear();
    clearQuery();

    componentCount_ = 0;
    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::geometric::ImplicitRandomPRM::freeMemory()
{
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();
}

ompl::geometric::ImplicitRandomPRM::Vertex ompl::geometric::ImplicitRandomPRM::addMilestone(base::State *state, Eigen::VectorXd& newV)
{
    if (state == nullptr) {
        // create the state
        if (dimRd_ == 2) {
            for (int i = 0; i < robotCount_; ++i) {
                workState_->as<ob::CompoundStateSpace::StateType>()->components[i]
                          ->as<ob::SE2StateSpace::StateType>()->setXY(newV[2*i], newV[2*i+1]);
                workState_->as<ob::CompoundStateSpace::StateType>()->components[i]
                          ->as<ob::SE2StateSpace::StateType>()->setYaw(0);
            }
        } else if (dimRd_ == 3) {
            for (int i = 0; i < robotCount_; ++i) {
                workState_->as<ob::SE3StateSpace::StateType>()->setXYZ(newV[3*i], newV[3*i+1], newV[3*i+2]);
                changeAngleToQuaternion(0, 0, 0, workState_->as<ob::SE3StateSpace::StateType>());
            }
        }
        // allocate it
        state = si_->cloneState(workState_);
    }
    // is it valid?
    Vertex m = nullptr;
    EigenVecToVertex_[newV] = nullptr; // just to add it, so it is known
    if (si_->isValid(state)) {
        m = boost::add_vertex(g_);
        stateProperty_[m] = state;
        vertexValidityProperty_[m] = VALIDITY_UNKNOWN;
        unsigned long int newComponent = componentCount_++;
        vertexComponentProperty_[m] = newComponent;
        componentSize_[newComponent] = 1;
        // my maps
        // std::string newVfixedStr = EigenToString(angleShifted);
        // EigenVecToVertex_[EigenToString(newV)] = m;
        EigenVecToVertex_[newV] = m;
        bool a = EigenVecToVertex_.contains(newV);
        EigenVecToVertexOld_[EigenToString(newV)] = m;
        // EigenVecToVertex_2[newV] = m;
        strToEigen_[EigenToString(newV)] = newV;
        // std::cout << EigenToString(newV) << std::endl;
        if (m==nullptr)
            int x = 1;
        nn_->add(m);
    }

    return m;
}

void ompl::geometric::ImplicitRandomPRM::setStartAndGoalEigen(const Eigen::VectorXd &startEigen, const Eigen::VectorXd &goalEigen) {
    startEigen_ = startEigen;
    goalEigen_ = goalEigen;
}

void ompl::geometric::ImplicitRandomPRM::AddAllVerticesWithNN() {
     // goOverSamples(1, startEigen_);
    long sampleCount = 0;
    while (sampleCount++ < sampleLimit_) {
        sampler_->sampleUniform(workState_);
        auto newV = StateToEigen(workState_);
        addMilestone(si_->cloneState(workState_), newV);
    }
}

ompl::base::PlannerStatus ompl::geometric::ImplicitRandomPRM::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart()) {
        startEigen_ = StateToEigen(st);
        startM_.push_back(addMilestone(si_->cloneState(st), startEigen_));
        // startM_.push_back(addMilestone2(si_->cloneState(st), startEigen_, true, false));
    }

    if (startM_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr) {
            goalEigen_ = StateToEigen(st);
            goalM_.push_back(addMilestone(si_->cloneState(st), goalEigen_));
            // goalM_.push_back(addMilestone2(si_->cloneState(st), goalEigen_, true, false));
        }

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    bestCost_ = opt_->infiniteCost();
    // base::State *workState = si_->allocState();
    std::pair<std::size_t, std::size_t> startGoalPair;
    base::PathPtr bestSolution;
    bool fullyOptimized = false;
    bool someSolutionFound = false;
    unsigned int optimizingComponentSegments = 0;
    // add all valid vertices and ALL edges
    OMPL_INFORM("Adding vertices and building NN structure.");
    auto tConstructionStart = std::chrono::high_resolution_clock::now();
    AddAllVerticesWithNN();
    auto tConstructionEnd = std::chrono::high_resolution_clock::now();
    results_.constructionTime =
        duration_cast<std::chrono::microseconds>(tConstructionEnd - tConstructionStart).count();
    std::cout << "added " << boost::num_vertices(g_) << " vertices." << std::endl;
    std::cout << "It took " << results_.constructionTime << "MS." << std::endl;
    // construct edges
    const long int solComponent = solutionComponent(&startGoalPair);
    Vertex startV = startM_[startGoalPair.first];
    Vertex goalV = goalM_[startGoalPair.second];
    OMPL_INFORM("Running A* on the PRM to find a solution");
    base::PathPtr solution;
    solution = constructSolution(startV, goalV);
    if (solution)
    {
        someSolutionFound = true;
        base::Cost c = solution->cost(opt_);
        if (opt_->isSatisfied(c))
        {
            fullyOptimized = true;
            bestSolution = solution;
            bestCost_ = c;
        }
        if (opt_->isCostBetterThan(c, bestCost_))
        {
            bestSolution = solution;
            bestCost_ = c;
        }
    }

    pdef_->addSolutionPath(bestSolution);
    // }
    auto vertices = 0;
    auto edges = 0;
    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);
    std::cout << "Edge count=" << boost::num_edges(g_) << std::endl;

    return bestSolution ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::ImplicitRandomPRM::uniteComponents(Vertex a, Vertex b)
{
    unsigned long int componentA = vertexComponentProperty_[a];
    unsigned long int componentB = vertexComponentProperty_[b];
    if (componentA == componentB)
        return;
    if (componentSize_[componentA] > componentSize_[componentB])
    {
        std::swap(componentA, componentB);
        std::swap(a, b);
    }
    markComponent(a, componentB);
}

void ompl::geometric::ImplicitRandomPRM::markComponent(Vertex v, unsigned long int newComponent)
{
    std::queue<Vertex> q;
    q.push(v);
    while (!q.empty())
    {
        Vertex n = q.front();
        q.pop();
        unsigned long int &component = vertexComponentProperty_[n];
        if (component == newComponent)
            continue;
        if (componentSize_[component] == 1)
            componentSize_.erase(component);
        else
            componentSize_[component]--;
        component = newComponent;
        componentSize_[newComponent]++;
        boost::graph_traits<Graph>::adjacency_iterator nbh, last;
        for (boost::tie(nbh, last) = boost::adjacent_vertices(n, g_); nbh != last; ++nbh)
            q.push(*nbh);
    }
}

long int ompl::geometric::ImplicitRandomPRM::solutionComponent(std::pair<std::size_t, std::size_t> *startGoalPair) const
{
    for (std::size_t startIndex = 0; startIndex < startM_.size(); ++startIndex)
    {
        long int startComponent = vertexComponentProperty_[startM_[startIndex]];
        for (std::size_t goalIndex = 0; goalIndex < goalM_.size(); ++goalIndex)
        {
            if (startComponent == (long int)vertexComponentProperty_[goalM_[goalIndex]])
            {
                startGoalPair->first = startIndex;
                startGoalPair->second = goalIndex;
                return startComponent;
            }
        }
    }
    return -1;
}

enum checkState {
    OPEN,
    CLOSE,
};

struct EigenAstar {
    // Eigen::VectorXd v;
    // checkState color;
    double g;
    double h;
    ob::State* state;
    int visited;
};

struct EigenGT {
    bool operator()(const EigenAstar* v1, const EigenAstar* v2) {
        return v1->g + v1->h > v2->g + v2->h;
    }
};

Eigen::VectorXd ompl::geometric::ImplicitRandomPRM::StateToEigen(const ob::State* state) const {
    int d_ = dimRd_ * robotCount_;
    std::vector<double> vals;
    for (int i = 0; i < robotCount_; ++i) {
        vals.push_back(state->as<ob::CompoundStateSpace::StateType>()->components[i]
                            ->as<ob::SE2StateSpace::StateType>()->getX());
        vals.push_back(state->as<ob::CompoundStateSpace::StateType>()->components[i]
                              ->as<ob::SE2StateSpace::StateType>()->getY());
    }
    Eigen::VectorXd res = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vals.data(), vals.size());
    return res;
}

ompl::base::Cost ompl::geometric::ImplicitRandomPRM::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}

void ompl::geometric::ImplicitRandomPRM::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states. Tag all states known to be valid as 1.
    // Unchecked states are tagged as 0.
    for (auto i : startM_)
        data.addStartVertex(base::PlannerDataVertex(stateProperty_[i], 1));

    for (auto i : goalM_)
        data.addGoalVertex(base::PlannerDataVertex(stateProperty_[i], 1));

    // Adding edges and all other vertices simultaneously
    foreach (const Edge e, boost::edges(g_))
    {
        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);
        data.addEdge(base::PlannerDataVertex(stateProperty_[v1]), base::PlannerDataVertex(stateProperty_[v2]));

        // Add the reverse edge, since we're constructing an undirected roadmap
        data.addEdge(base::PlannerDataVertex(stateProperty_[v2]), base::PlannerDataVertex(stateProperty_[v1]));

        // Add tags for the newly added vertices
        data.tagState(stateProperty_[v1], (vertexValidityProperty_[v1] & VALIDITY_TRUE) == 0 ? 0 : 1);
        data.tagState(stateProperty_[v2], (vertexValidityProperty_[v2] & VALIDITY_TRUE) == 0 ? 0 : 1);
    }
}

#pragma region unused_funcs
// test function
ob::State* ompl::geometric::ImplicitRandomPRM::createNewState(Eigen::VectorXd& newV) {
    if (dimRd_ == 2) {
        for (int i = 0; i < robotCount_; ++i) {
            workState_->as<ob::CompoundStateSpace::StateType>()->components[i]
            ->as<ob::SE2StateSpace::StateType>()->setXY(newV[2*i], newV[2*i+1]);
            workState_->as<ob::CompoundStateSpace::StateType>()->components[i]
            ->as<ob::SE2StateSpace::StateType>()->setYaw(0);
        }
    } else if (dimRd_ == 3) {
        for (int i = 0; i < robotCount_; ++i) {
            workState_->as<ob::SE3StateSpace::StateType>()->setXYZ(newV[3*i], newV[3*i+1], newV[3*i+2]);
            changeAngleToQuaternion(0, 0, 0, workState_->as<ob::SE3StateSpace::StateType>());
        }
    }
    return si_->cloneState(workState_);
}

ompl::base::PathPtr ompl::geometric::ImplicitRandomPRM::constructSolution(const Vertex &start, const Vertex &goal) {
    std::unordered_map<Eigen::VectorXd, EigenAstar*, matrix_hash<Eigen::VectorXd>, TVectorEquals> VISITED;
    std::unordered_map<Eigen::VectorXd, bool, matrix_hash<Eigen::VectorXd>, TVectorEquals> INVALID;
    std::unordered_map<EigenAstar*, EigenAstar*> PREV;
    std::vector<EigenAstar*> OPEN;
    int d_ = dimRd_ * robotCount_;
    auto compareEigens = TVectorEquals();
    // init the root node
    startEigen_ = StateToEigen(stateProperty_[start]);
    goalEigen_ = StateToEigen(stateProperty_[goal]);
    auto* startV = new EigenAstar{
        0, distEquclid(startEigen_, goalEigen_,d_), this->createNewState(startEigen_), 1
    };
    VISITED[startEigen_] = startV;
    PREV[startV] = nullptr;
    OPEN.push_back(startV);
    // heapify OPEN
    std::ranges::make_heap(OPEN, EigenGT());
    // A* loop
    auto tAstarStart = std::chrono::high_resolution_clock::now();
    auto tLast = std::chrono::high_resolution_clock::now();
    auto tLatest = std::chrono::high_resolution_clock::now();
    while (!OPEN.empty()) {
        // get the min f-value vertex
        std::ranges::pop_heap(OPEN, EigenGT());
        EigenAstar* minV = OPEN.back();
        OPEN.pop_back();
        Eigen::VectorXd minVeigen = StateToEigen(minV->state);
        // have we reached the goal?
        if (compareEigens(minVeigen, goalEigen_)) {
            std::vector<const base::State*> resVec;
            while (PREV[minV] != nullptr) {
                resVec.push_back(minV->state);
                minV = PREV[minV];
            }
            resVec.push_back(minV->state);
            auto p(std::make_shared<PathGeometric>(si_));
            for (std::vector<const base::State *>::const_reverse_iterator st = resVec.rbegin(); st != resVec.rend(); ++st) {
                p->append(*st);
            }
            std::cout << "size of visited=" << VISITED.size() << std::endl;
            auto tAstarEnd = std::chrono::high_resolution_clock::now();
            results_.astarTime =
                duration_cast<std::chrono::microseconds>(tAstarEnd - tAstarStart).count();
            return p;
        }
        // go over neighbors in a ball
        std::vector<Vertex> neighbors;
        nn_->nearestR(EigenVecToVertex_[StateToEigen(minV->state)], r_, neighbors);
        std::vector<Eigen::VectorXd> neighborsEigen;
        for (const auto& neighborM: neighbors) {
            neighborsEigen.push_back(StateToEigen(stateProperty_[neighborM]));
        }
        bool hasGoal = false;
        if (!compareEigens(minVeigen, goalEigen_) && distEquclid(minVeigen, goalEigen_, d_) < r_) {
            neighborsEigen.push_back(goalEigen_);
            hasGoal = true;
        }
        for (auto neighbor: neighborsEigen) {
            // tLatest = std::chrono::high_resolution_clock::now();
            // std::cout << "dur5=" << duration_cast<std::chrono::microseconds>(tLatest - tLast).count() << std::endl;
            // tLast = tLatest;

            // neighbor += minVeigen; // shift the neighbor to its true place
            // vertex/edge validity checking
            auto neigbhorState = this->createNewState(neighbor);
            if (!si_->isValid(neigbhorState)) {
                si_->freeState(neigbhorState);
                continue; // vertex in collision
            }
            bool isEdgeValid = si_->checkMotion(minV->state, neigbhorState);
            if (!isEdgeValid) continue; // edge in collision TODO RETURN THIS

            double newG = minV->g + distEquclid(neighbor, minVeigen,d_);
            double currG = VISITED.contains(neighbor) ? VISITED[neighbor]->g : DBL_MAX;
            if (newG < currG) {
                EigenAstar* neighborV = nullptr;
                if (VISITED.contains(neighbor)) {
                    neighborV = VISITED[neighbor];
                } else {
                    neighborV = new EigenAstar{
                        newG, distEquclid(neighbor, goalEigen_,d_),
                        this->createNewState(neighbor), 0
                    };
                    VISITED[neighbor] = neighborV; // eigen to struct
                }
                PREV[neighborV] = minV; // new parent
                // is it in OPEN?
                bool inOpen = false;
                for (auto elem: OPEN) {
                    if (elem == neighborV) {
                        inOpen = true;
                        break;
                    }
                }
                // if not, insert it
                if (!inOpen) {
                    // insert to heap and heapify
                    OPEN.push_back(neighborV); // insert to heap
                    std::push_heap(OPEN.begin(), OPEN.end(), EigenGT()); // bubble element to its place
                }
            }
        }
        if (hasGoal) neighborsEigen.pop_back(); // remove the goal.
    }
    std::cout << "size of visited=" << VISITED.size() << std::endl;
    auto tAstarEnd = std::chrono::high_resolution_clock::now();
    results_.astarTime =
        duration_cast<std::chrono::microseconds>(tAstarEnd - tAstarStart).count();
    return nullptr;
}

Eigen::VectorXd ompl::geometric::ImplicitRandomPRM::getValidStartState() {
    sampler_ = si_->allocStateSampler();
    sampler_->sampleUniform(workState_);
    std::string a = EigenToString(StateToEigen(workState_));
    while (!si_->isValid(workState_)) {
        sampler_->sampleUniform(workState_);
        a = EigenToString(StateToEigen(workState_));
    }
    return StateToEigen(workState_);
}

#pragma endregion unused_funcs
