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

#include "../include/LatticePRM.h"

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

ompl::geometric::LatticePRM::LatticePRM(const base::SpaceInformationPtr &si, bool starStrategy)
  : base::Planner(si, "LatticePRM")
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


    Planner::declareParam<double>("range", this, &LatticePRM::setRange, &LatticePRM::getRange, "0.:1.:10000.");
    if (!starStrategy_)
        Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &LatticePRM::setMaxNearestNeighbors,
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

ompl::geometric::LatticePRM::LatticePRM(const base::PlannerData &data, bool starStrategy)
  : LatticePRM(data.getSpaceInformation(), starStrategy)
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

ompl::geometric::LatticePRM::~LatticePRM()
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

// std::tuple<double, double, double, double> angleToQuaternion(double u1, double u2, double u3) {
//     // first create the quaternion (from a NASA document!)
//     double q1 = -std::sin(0.5*u1) * std::sin(0.5*u2) * std::sin(0.5*u3) +
//         std::cos(0.5*u1) * std::cos(0.5*u2) * std::cos(0.5*u3);
//     double q2 = std::sin(0.5*u1) * std::cos(0.5*u2) * std::cos(0.5*u3) +
//         std::cos(0.5*u1) * std::sin(0.5*u2) * std::sin(0.5*u3);
//     double q3 = -std::sin(0.5*u1) * std::cos(0.5*u2) * std::sin(0.5*u3) +
//         std::cos(0.5*u1) * std::sin(0.5*u2) * std::cos(0.5*u3);
//     double q4 = std::sin(0.5*u1) * std::sin(0.5*u2) * std::cos(0.5*u3) +
//         std::cos(0.5*u1) * std::cos(0.5*u2) * std::sin(0.5*u3);
//     // then convert it to the axis-angle representation
//     double acos = std::acos(q4);
//     double denom = std::sin(acos);
//     return {q1/denom, q2/denom, q3/denom, 2*acos};
// }

void ompl::geometric::LatticePRM::changeAngleToQuaternion(double u1, double u2, double u3, ob::SE3StateSpace::StateType* state) {
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

bool ompl::geometric::LatticePRM::LazyAstar() {
    return true;
}

bool compareDoubles(double a, double b) {
    return fabs(a - b) < DBL_EPSILON;
}

void ompl::geometric::LatticePRM::setup()
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
    // if (!sampler_)
    //     sampler_ = si_->allocValidStateSampler();
}

void ompl::geometric::LatticePRM::setRange(double distance)
{
    maxDistance_ = distance;
    if (!userSetConnectionStrategy_)
        setDefaultConnectionStrategy();
    if (isSetup())
        setup();
}

void ompl::geometric::LatticePRM::setMaxNearestNeighbors(unsigned int k)
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

void ompl::geometric::LatticePRM::setDefaultConnectionStrategy()
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

void ompl::geometric::LatticePRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::LatticePRM::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::LatticePRM::clearValidity()
{
    foreach (const Vertex v, boost::vertices(g_))
        vertexValidityProperty_[v] = VALIDITY_UNKNOWN;
    foreach (const Edge e, boost::edges(g_))
        edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
}

void ompl::geometric::LatticePRM::clear()
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

void ompl::geometric::LatticePRM::freeMemory()
{
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();
}

ompl::geometric::LatticePRM::Vertex ompl::geometric::LatticePRM::addMilestone(base::State *state, Eigen::VectorXd& newV)
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
    }

    return m;
}

double roundDbl(double d) {
    return (int)(d * 10000) / 10000.0;
}

void ompl::geometric::LatticePRM::setLatticeType(LatticeType type, double delta, double epsilon) {
    LatticeType_ = type;
    // this radius is defined in Dayan et. al (23')
    r_ = (2*(epsilon + 1)* delta) / std::sqrt(1+epsilon * epsilon);
    double beta = (delta * epsilon) / std::sqrt(1 + pow(epsilon, 2));
    int dim = dimRd_ * robotCount_;
    double rescale_ = 0;

    double theta = r_ / beta;
    // double theoretical = (1.0 / dim) * pow(1.46 * theta, dim) + pow(theta * sqrt((2 * dim - 1) / 16.0), dim - 2);
    // std::cout << "theoretical = " << theoretical << std::endl;
    // this transformation is from the paper
    switch (type) {
        case File:
            break;
        case Zn:
            rescale_ = (2 * beta) / sqrt(dim);
            // Zn rescaled generator
            T_ = Eigen::MatrixXd(dim, dim);
            for (int j = 0; j < dim; ++j) {
                for (int w = 0; w < dim; ++w) {
                    if (j == w) {
                        T_(j, w) = roundDbl(rescale_);
                    } else {
                        T_(j, w) = 0;
                    }
                }
            }
            break;
        case DnStar:
            if (dim % 2 == 0) {
                rescale_ = sqrt(8.0 / dim) * beta;
            } else {
                rescale_ =  (4*beta) / sqrt(2.0 * dim - 1);
            }
            // DnStar rescaled generator
            T_ = Eigen::MatrixXd(dim, dim);
            for (int j = 0; j < dim; ++j) {
                for (int w = 0; w < dim; ++w) {
                    if (j < dim - 1) {
                        if (j == w) {
                            T_(j, w) = roundDbl(rescale_);
                        } else {
                            T_(j, w) = 0;
                        }
                    } else {
                        T_(j, w) = roundDbl(rescale_ * 0.5);
                    }
                }
            }
            T_ = T_.transpose().eval();
            break;
        case AnStar:
            rescale_ = sqrt((12.0 * (dim + 1)) / (dim * (dim + 2))) * beta;
            double anstar_x = 1.0 / (dim + 1 - sqrt(dim + 1));
            // DnStar rescaled generator
            T_ = Eigen::MatrixXd(dim, dim);
            for (int j = 0; j < dim; ++j) {
                for (int w = 0; w < dim; ++w) {
                    if (j == 0) {
                        if (w < dim - 1) {
                            T_(j, w) = roundDbl(rescale_);
                        } else {
                            T_(j, w) = roundDbl((rescale_ * (anstar_x - 1)));
                        }
                    } else {
                        if (w == j - 1) {
                            T_(j, w) = roundDbl(-rescale_);
                        } else if (w < dim - 1) {
                            T_(j, w) = 0;
                        } else {
                            T_(j, w) = roundDbl(rescale_ * anstar_x);
                        }
                    }
                }
            }
            break;
    }
    // set the max base vector
    for (int i = 0; i < dim; ++i) {
        maxEdge_ = std::max(maxEdge_, T_.col(i).norm());
    }
}

void ompl::geometric::LatticePRM::setStartAndGoalEigen(const Eigen::VectorXd &startEigen, const Eigen::VectorXd &goalEigen) {
    startEigen_ = startEigen;
    goalEigen_ = goalEigen;
}

double distEquclid(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2, int d_) {
    double res = 0;
    for (int w = 0; w < d_; ++w) {
        res += std::pow(v1[w] - v2[w], 2);
    }
    return std::sqrt(res);
}

bool ompl::geometric::LatticePRM::connectVertices(const Eigen::VectorXd& root, const Eigen::VectorXd& neighbor) {
    double rootToNode = (neighbor - root).norm();
    Vertex rootVertex = EigenVecToVertex_[root];
    Vertex neighborV = EigenVecToVertex_[neighbor];
    if (edges_.contains(neighborV) && edges_[neighborV] == rootVertex) return false; // already covered
    if (rootToNode < r_ || compareDoubles(rootToNode, r_)) {
        if (si_->checkMotion(stateProperty_[rootVertex], stateProperty_[neighborV])) {
            const base::Cost weight = opt_->motionCost(stateProperty_[rootVertex], stateProperty_[neighborV]);
            // const base::Cost weight(rootToNode);//opt_->motionCost(stateProperty_[rootVertex], stateProperty_[neighborV]);
            const Graph::edge_property_type properties(weight);
            const Edge &e = boost::add_edge(rootVertex, neighborV, properties, g_).first;
            edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
            edges_[rootVertex] = neighborV;
            uniteComponents(rootVertex, neighborV);
            // std::cout << "NEW: e1=" << EigenToString(root) << "-->" << EigenToString(neighbor) << std::endl;
            return true;
        }
    }
    return false;
}

long ompl::geometric::LatticePRM::AddAllEdges() {
    // connect the lattice vertices
    int d_ = dimRd_ * robotCount_;
    long edges = 0;
    long edgesTried= 0;
    std::vector<Eigen::VectorXd> ballRNN = this->returnSamplesInBallNoBounds(startEigen_);
    for (auto& mapPair: EigenVecToVertex_) {
        Eigen::VectorXd root = mapPair.first;
        if (EigenVecToVertex_[root] == nullptr) continue; // invalid root
        if (root == startEigen_ || root == goalEigen_) continue;
        // goOverSamples(0, root);
        // try also to connect the node to the start/goal
        // TODO change this to direct connection, this is wasteful time-wise
        if (root != startEigen_ && distEquclid(startEigen_, root, d_) < r_) {
            if (connectVertices(root, startEigen_)) edges++;
        }
        if (root != goalEigen_ && distEquclid(goalEigen_, root, d_) < r_) {
            if (connectVertices(root, goalEigen_)) edges++;
        }
        // go over neighbors in a ballreturnSamplesInBallNoBounds
        Eigen::VectorXd ballShiftVector = root - startEigen_;
        edgesTried += 2;
        // for (auto neighbor: ballRNN) {
        for (auto & neighbor : ballRNN) {
            auto neighborShifted = neighbor + ballShiftVector; // shift the neighbor to its true place
            if (!EigenVecToVertex_.contains(neighborShifted)) continue;
            if (EigenVecToVertex_[neighborShifted] == nullptr) continue;
            if (connectVertices(root, neighborShifted)) edges++;
        }
    }
    return edges;
    // TODO I assume 1 goal and 1 start for the time being
}

long ompl::geometric::LatticePRM::AddAllEdgesOld() {
    // connect the lattice vertices
    long edges = 0;
    for (auto& it: EigenVecToVertexOld_) {
        Eigen::VectorXd root = strToEigen_[it.first];
        if (root == startEigen_ || root == goalEigen_) continue;
        edges += goOverSamples(0, root);
        // try also to connect the node to the start/goal
        // TODO change this to direct connection, this is wasteful time-wise
        for (auto neighbor: {startEigen_, goalEigen_}) {
            double rootToNode = (neighbor - root).norm();
            if (rootToNode < r_ || compareDoubles(rootToNode, r_)) {
                Vertex rootVertex = EigenVecToVertexOld_[EigenToString(root)];
                Vertex neighborV = EigenVecToVertexOld_[EigenToString(neighbor)];
                if (si_->checkMotion(stateProperty_[rootVertex], stateProperty_[neighborV])) {
                    const base::Cost weight = opt_->motionCost(stateProperty_[rootVertex], stateProperty_[neighborV]);
                    // const base::Cost weight(rootToNode);//opt_->motionCost(stateProperty_[rootVertex], stateProperty_[neighborV]);
                    const Graph::edge_property_type properties(weight);
                    const Edge &e = boost::add_edge(rootVertex, neighborV, properties, g_).first;
                    edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
                    uniteComponents(rootVertex, neighborV);
                    // std::cout << "NEW: e1=" << EigenToString(root) << "-->" << EigenToString(neighbor) << std::endl;
                    edges++;
                    // std::cout << "connected=" << EigenToString(root) << "-->" << EigenToString(neighbor) << std::endl;
                }
            }
        }
    }
    // TODO I assume 1 goal and 1 start for the time being
    return edges;
}

void ompl::geometric::LatticePRM::AddAllVerticesOld() {
     goOverSamples(1, startEigen_);
}

// type: 0=ball (connecting edges), 1=box (sampling vertices)
// ball is used in connecting edge, box is used in the start to create the vertices.
long ompl::geometric::LatticePRM::countSamplesInMap(Eigen::VectorXd& root) {
    int d_ = dimRd_ * robotCount_;
    long samples = 0;
    std::vector<Eigen::VectorXd> open;
    std::vector<Eigen::VectorXd> openTemp;
    std::unordered_map<std::string, int> visited;
    // long samples = 1;
    // add root
    visited[EigenToString(root)] = 0;
    open.push_back(root);
    // get the samples in the cube
    while (!open.empty() || !openTemp.empty()) {
        if (open.empty()) {
            // we have the next wave of neighbors
            open = openTemp;
            openTemp.clear();
        }
        Eigen::VectorXd v = open[open.size() - 1];
        open.pop_back();
        std::string vstr = EigenToString(v);
        int sign = -1;
        for (int j = 0; j < 2; ++j) {
            sign = -sign;
            for (int i = 0; i < d_; ++i) {
                Eigen::VectorXd newNeighbor = v + sign * T_.col(i);
                std::string newNeighStr = EigenToString(newNeighbor);
                if (visited.contains(newNeighStr)) {
                    visited[newNeighStr]++;
                    // nodes can only have up to 2d visits
                    if (visited[newNeighStr] == 2 * d_) visited.erase(newNeighStr);
                } else {
                    visited[newNeighStr] = 1;
                    double newvNorm = 0;
                    bool inAreaBigger = true;
                    // is it in the rectangle
                    for (int w = 0; w < d_; ++w) {
                        inAreaBigger = inAreaBigger && (
                            (newNeighbor[w] > mapExtent_.low[w % dimRd_] - maxEdge_
                                || compareDoubles(newNeighbor[w], mapExtent_.low[w  % dimRd_] - maxEdge_)) &&
                            (newNeighbor[w] < mapExtent_.high[w % dimRd_] + maxEdge_
                                || compareDoubles(newNeighbor[w], mapExtent_.high[w  % dimRd_] + maxEdge_)));
                    }
                    bool inArea = true;
                    // is it in the rectangle
                    for (int w = 0; w < d_; ++w) {
                        inArea = inArea && (
                            (newNeighbor[w] > mapExtent_.low[w % dimRd_]
                                || compareDoubles(newNeighbor[w], mapExtent_.low[w  % dimRd_])) &&
                            (newNeighbor[w] < mapExtent_.high[w % dimRd_]
                                || compareDoubles(newNeighbor[w], mapExtent_.high[w  % dimRd_])));
                    }
                    if (inArea) samples++;
                    // we need to explore all space, regardless of collisions.
                    // some samples may be "isolated" in terms of direct connection to another
                    // sample, but in a PRM r-ball they may end up connecting.
                    if (inAreaBigger) openTemp.push_back(newNeighbor);
                }
            }
        }
    }
    return samples;
}

// type: 0=ball (connecting edges), 1=box (sampling vertices)
// ball is used in connecting edge, box is used in the start to create the vertices.
void ompl::geometric::LatticePRM::addSamplesInMapRegion(const Eigen::VectorXd& root) {
    int d_ = dimRd_ * robotCount_;
    std::vector<Eigen::VectorXd> open;
    std::vector<Eigen::VectorXd> openTemp;
    // std::unordered_map<std::string, int> visited;
    std::unordered_map<Eigen::VectorXd, int, matrix_hash<Eigen::VectorXd>, TVectorEquals> visited;
    // add root
    visited[root] = 0;
    open.push_back(root);
    Vertex rootVertex = EigenVecToVertex_[root];
    // get the samples in the cube
    while (!open.empty() || !openTemp.empty()) {
        if (open.empty()) {
            // we have the next wave of neighbors
            open = openTemp;
            openTemp.clear();
        }
        Eigen::VectorXd v = open[open.size() - 1];
        open.pop_back();
        // std::string vstr = EigenToString(v);
        int sign = -1;
        for (int j = 0; j < 2; ++j) {
            sign = -sign;
            for (int i = 0; i < d_; ++i) {
                Eigen::VectorXd newNeighbor = v + sign * T_.col(i);
                std::string nstr = EigenToString(newNeighbor);
                if (visited.contains(newNeighbor)) {
                    visited[newNeighbor]++;
                    // nodes can only have up to 2d visits
                    if (visited[newNeighbor] == 2 * d_) visited.erase(newNeighbor);
                } else {
                    visited[newNeighbor] = 1;
                    bool inArea = true;
                    // is it in the rectangle
                    for (int w = 0; w < d_; ++w) {
                        inArea = inArea && (
                            (newNeighbor[w] > mapExtent_.low[w % dimRd_] - maxEdge_
                                || compareDoubles(newNeighbor[w], mapExtent_.low[w  % dimRd_] - maxEdge_)) &&
                            (newNeighbor[w] < mapExtent_.high[w % dimRd_] + maxEdge_
                                || compareDoubles(newNeighbor[w], mapExtent_.high[w  % dimRd_] - maxEdge_)));
                    }
                    if (inArea) {
                        if (addMilestone(nullptr, newNeighbor) != nullptr);
                            // std::cout << "added " << EigenToString(newNeighbor) << std::endl;

                        // if (EigenToString(newNeighbor)[0]  == '-' && EigenToString(newNeighbor)[1]  == '2' && EigenToString(newNeighbor)[2]  == '.') {
                        //     for (auto& it: EigenVecToVertex_) {
                        //         std::cout << "TESTING: " << EigenToString(it.first) << std::endl;
                        //     }
                        // }

                        // we need to explore all space, regardless of collisions.
                        // some samples may be "isolated" in terms of direct connection to another
                        // sample, but in a PRM r-ball they may end up connecting.
                        openTemp.push_back(newNeighbor);
                    }
                }
            }
        }
    }
}

// type: 0=ball (connecting edges), 1=box (sampling vertices)
// ball is used in connecting edge, box is used in the start to create the vertices.
long ompl::geometric::LatticePRM::goOverSamples(int type, Eigen::VectorXd& root) {
    bool testMode = false;
    int d_ = dimRd_ * robotCount_;
    long edges = 0;
    std::vector<Eigen::VectorXd> open;
    std::vector<Eigen::VectorXd> openTemp;
    std::unordered_map<std::string, int> visited;
    // long samples = 1;
    // add root
    visited[EigenToString(root)] = 0;
    open.push_back(root);
    Vertex rootVertex = EigenVecToVertexOld_[EigenToString(root)];
    // get the samples in the cube
    while (!open.empty() || !openTemp.empty()) {
        if (open.empty()) {
            // we have the next wave of neighbors
            open = openTemp;
            openTemp.clear();
        }
        Eigen::VectorXd v = open[open.size() - 1];
        open.pop_back();
        std::string vstr = EigenToString(v);
        int sign = -1;
        for (int j = 0; j < 2; ++j) {
            sign = -sign;
            for (int i = 0; i < d_; ++i) {
                Eigen::VectorXd newNeighbor = v + sign * T_.col(i);
                std::string newNeighStr = EigenToString(newNeighbor);
                // if (EigenToString(root) == "37.695700,40.695700,-30.935100,40.695700,-30.935100,1.972600,")
                    // std::cout << "Special:" << newNeighStr << std::endl;
                // if (EigenToString(root) == "47.930600,18.022900,-20.700200,18.022900,-20.700200,-20.700200,")
                //     int x = 1;
                // if (newNeighStr == "47.930600,18.022900,-20.700200,18.022900,-20.700200,-20.700200,")
                //     int x = 1;
                // if (EigenToString(root) == "47.930600,18.022900,-20.700200,18.022900,-20.700200,-20.700200,")
                //     if (newNeighStr == "37.695700,40.695700,-30.935100,40.695700,-30.935100,1.972600,")
                //         int x = 1; //
                if (visited.contains(newNeighStr)) {
                    visited[newNeighStr]++;
                    // nodes can only have up to 2d visits
                    if (visited[newNeighStr] == 2 * d_) visited.erase(newNeighStr);
                } else {
                    visited[newNeighStr] = 1;
                    double newvNorm = 0;
                    bool inArea = true;
                    // is it in the rectangle
                    for (int w = 0; w < d_; ++w) {
                        inArea = inArea && (
                            (newNeighbor[w] > mapExtent_.low[w % dimRd_] - maxEdge_
                                || compareDoubles(newNeighbor[w], mapExtent_.low[w  % dimRd_] - maxEdge_)) &&
                            (newNeighbor[w] < mapExtent_.high[w % dimRd_] + maxEdge_
                                || compareDoubles(newNeighbor[w], mapExtent_.high[w  % dimRd_] + maxEdge_)));
                    }

                    if (type == 0) {
                        newvNorm = (newNeighbor - root).norm();
                        // make sure we're also in a ball
                        inArea = inArea &&  (newvNorm < r_ || compareDoubles(newvNorm, r_));
                    }
                    if (inArea) {
                        if (type == 0) {
                            // only connect to vertices that were valid
                            if (EigenVecToVertexOld_.contains(newNeighStr)) {
                                // connect edges
                                Vertex neighborV = EigenVecToVertexOld_[newNeighStr];
                                // Vertex neighborV = EigenVecToVertexOld_[newNeighbor];
                                // check the edges already
                                if (si_->checkMotion(stateProperty_[rootVertex], stateProperty_[neighborV])) {
                                    const base::Cost weight = opt_->motionCost(stateProperty_[rootVertex], stateProperty_[neighborV]);
                                    // const base::Cost weight(newvNorm);//opt_->motionCost(stateProperty_[rootVertex], stateProperty_[neighborV]);
                                    const Graph::edge_property_type properties(weight);
                                    const Edge &e = boost::add_edge(rootVertex, neighborV, properties, g_).first;
                                    // edges++;
                                    edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
                                    uniteComponents(rootVertex, neighborV);
                                    // std::cout << "NEW: e1=" << EigenToString(root) << "-->" << EigenToString(newNeighbor) << std::endl;
                                    edges++;
                                }
                            }
                        } if (type == 1) {
                            addMilestone(nullptr, newNeighbor);
                            // if (newNeighborVertex != nullptr) samples++;
                        }
                        // we need to explore all space, regardless of collisions.
                        // some samples may be "isolated" in terms of direct connection to another
                        // sample, but in a PRM r-ball they may end up connecting.
                        openTemp.push_back(newNeighbor);
                    }
                }
            }
        }
    }
    return edges;
}

double ompl::geometric::LatticePRM::distEquclid2(const ob::State* s1, const ob::State* s2, int d_) {
    double res = 0;
    for (int i = 0; i < robotCount_; ++i) {
        res += std::pow(s1->as<ob::CompoundStateSpace::StateType>()->components[i] //i-th SE(n)
                        ->as<ob::SE2StateSpace::StateType>()->getX() -
                        s2->as<ob::CompoundStateSpace::StateType>()->components[i] //i-th SE(n)
                        ->as<ob::SE2StateSpace::StateType>()->getX(), 2);
        res += std::pow(s1->as<ob::CompoundStateSpace::StateType>()->components[i] //i-th SE(n)
                        ->as<ob::SE2StateSpace::StateType>()->getY() -
                        s2->as<ob::CompoundStateSpace::StateType>()->components[i] //i-th SE(n)
                        ->as<ob::SE2StateSpace::StateType>()->getY(), 2);
    }
    return std::sqrt(res);
}

std::vector<Eigen::VectorXd> ompl::geometric::LatticePRM::returnSamplesInBall(Eigen::VectorXd &root) {
    int d_ = dimRd_ * robotCount_;
    std::vector<Eigen::VectorXd> open;
    std::vector<Eigen::VectorXd> openTemp;
    std::unordered_map<Eigen::VectorXd, int, matrix_hash<Eigen::VectorXd>, TVectorEquals> visited;
    long samples = 1;
    // add root
    // visited[EigenToString(root)] = 0;
    visited[root] = 0;
    open.push_back(root);
    // Vertex rootVertex = EigenVecToVertex_[EigenToString(root)];
    std::vector<Eigen::VectorXd> result;
    if (root != startEigen_ && distEquclid(startEigen_, root, d_) < r_) result.push_back(startEigen_);
    if (root != goalEigen_ && distEquclid(goalEigen_, root, d_) < r_) result.push_back(goalEigen_);
    Eigen::VectorXd bla;
    while (!open.empty() || !openTemp.empty()) {
        if (open.empty()) {
            // we have the next wave of neighbors
            open = openTemp;
            openTemp.clear();
        }
        Eigen::VectorXd v = open[open.size() - 1];
        // std::string vstr = EigenToString(v);
        open.pop_back();
        // for (auto v: open) {
        //     std::cout << v << std::endl;
        // }
        // std::cout << "vstr=" << vstr << std::endl;
        int sign = -1;
        for (int j = 0; j < 2; ++j) {
            sign = -sign;
            for (int i = 0; i < d_; ++i) {
                Eigen::VectorXd newNeighbor = v + sign * T_.col(i);
                // if (EigenToString(newNeighbor) == "-55.673000,-32.927900,27.072100,-32.927900,27.072100,27.072100,") {
                //     int y = 1;
                // }
                // std::string newNeighStr = EigenToString(newNeighbor);
                // std::cout << open.size() << "," << openTemp.size() << std::endl;
                // if (visited.contains(newNeighStr)) {
                bool haa = visited.contains(newNeighbor);
                if (visited.contains(newNeighbor) && visited[newNeighbor] < 2*d_) {
                    // visited[newNeighStr]++;
                    visited[newNeighbor]++;
                    // if (newNeighStr == "-33.000000,-30.000000,30.000000,-30.000000,30.000000,30.000000,") {
                    //     std::cout << visited[newNeighbor] << std::endl;
                    //     std::cout << newNeighStr << std::endl;
                    // }
                    // if (newNeighStr == "-22.859900,-33.023000,26.976900,-33.023000,26.976900,26.976900,") {
                    // if (newNeighStr == "-22.860000,-33.023000,26.977000,-33.023000,26.977000,26.977000,") {
                    //     std::cout << visited[newNeighbor] << std::endl;
                    //     std::cout << newNeighStr << std::endl;
                    // }
                    // nodes can only have up to 2d visits
                    // if (visited[newNeighStr] == 2 * d_) visited.erase(newNeighStr);
                    if (visited[newNeighbor] == 2 * d_)
                        visited.erase(newNeighbor);
                } else {
                    // visited[newNeighStr] = 1;
                    // if (newNeighStr == "-33.000000,-30.000000,30.000000,-30.000000,30.000000,30.000000,") {
                    // bool ha = visited.contains(newNeighbor);
                    // auto ha2 = TVectorEquals();
                    // bool ha3 = ha2(newNeighbor, startEigen_);
                    // bool ha4 = visited.contains(startEigen_);
                    // std::cout << vstr << std::endl;
                    // }
                    // if (newNeighStr == "-22.859944,-33.023048,26.976952,-33.023048,26.976952,26.976952,") {
                    // if (newNeighStr == "-22.860000,-33.023000,26.977000,-33.023000,26.977000,26.977000,") {
                    //     if (visited.contains(newNeighbor))
                    //         std::cout << "?A?!?!" << visited[newNeighbor] << std::endl;
                    //     auto a = visited.contains(bla);
                    //     std::cout << newNeighStr << std::endl;
                    //     int as = 1;
                    //     bla = Eigen::VectorXd(newNeighbor);
                    // }
                    // auto sad = TVectorEquals();
                    // auto sad2 = sad(newNeighbor, startEigen_);
                    // std::cout << "======================================" << std::endl;
                    // for (auto v: visited) {
                    //     std::cout << v.first << std::endl << "count=" << v.second << std::endl;
                    // }
                    visited[newNeighbor] = 1;
                    double newvNorm = 0;
                    bool inArea = true;
                    // is it in the rectangle
                    for (int w = 0; w < d_; ++w) {
                        inArea = inArea && (
                        (newNeighbor[w] > mapExtent_.low[w % dimRd_] || compareDoubles(newNeighbor[w], mapExtent_.low[w  % dimRd_])) &&
                        (newNeighbor[w] < mapExtent_.high[w % dimRd_] || compareDoubles(newNeighbor[w], mapExtent_.high[w  % dimRd_])));
                    }
                    // if (EigenToString(newNeighbor) == "-25.253000,-41.996900,18.003100,-41.996900,18.003100,18.003100,") {
                    //     int x = 1;
                    // }
                    // newvNorm = (newNeighbor - root).norm();
                    newvNorm = distEquclid(newNeighbor, root, d_);
                    inArea = inArea &&  (newvNorm < r_ || compareDoubles(newvNorm, r_));
                    if (inArea) {
                        result.push_back(newNeighbor);
                        openTemp.push_back(newNeighbor);
                    }
                }
            }
        }
    }
    return result;
}

std::vector<Eigen::VectorXd> ompl::geometric::LatticePRM::returnSamplesInBallNoBounds(Eigen::VectorXd &root) {
    int d_ = dimRd_ * robotCount_;
    std::vector<Eigen::VectorXd> open;
    std::vector<Eigen::VectorXd> openTemp;
    std::unordered_map<Eigen::VectorXd, int, matrix_hash<Eigen::VectorXd>, TVectorEquals> visited;
    long samples = 1;
    // add root
    // visited[EigenToString(root)] = 0;
    visited[root] = 0;
    open.push_back(root);
    // Vertex rootVertex = EigenVecToVertex_[EigenToString(root)];
    std::vector<Eigen::VectorXd> result;
    // if (root != startEigen_ && distEquclid(startEigen_, root, d_) < r_) result.push_back(startEigen_);
    // if (root != goalEigen_ && distEquclid(goalEigen_, root, d_) < r_) result.push_back(goalEigen_);
    Eigen::VectorXd bla;
    while (!open.empty() || !openTemp.empty()) {
        if (open.empty()) {
            // we have the next wave of neighbors
            open = openTemp;
            openTemp.clear();
        }
        Eigen::VectorXd v = open[open.size() - 1];
        // std::string vstr = EigenToString(v);
        open.pop_back();
        // for (auto v: open) {
        //     std::cout << v << std::endl;
        // }
        // std::cout << "vstr=" << vstr << std::endl;
        // Eigen::VectorXd t1(6);
        // t1 << -1.0, -1.0, -1.0, -1.0, -1.0, -7.0;
        // Eigen::VectorXd t2 = root + T_ * t1;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        // Eigen::VectorXd t3(6);
        // t3 << -1.0, -1.0, -1.0, -1.0, -1.0, -6.0;
        // t2 = root + T_ * t3;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        // Eigen::VectorXd t4(6);
        // t4 << -1.0, -1.0, -1.0, -1.0, -1.0, -5.0;
        // t2 = root + T_ * t4;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        // Eigen::VectorXd t5(6);
        // t5 << -1.0, -1.0, -1.0, -1.0, 0.0, -5.0;
        // t2 = root + T_ * t5;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        // Eigen::VectorXd t6(6);
        // t6 << -1.0, -1.0, -1.0, -1.0, 0.0, -4.0;
        // t2 = root + T_ * t6;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        // Eigen::VectorXd t7(6);
        // t7 << -1.0, -1.0, -1.0, 0.0, 0.0, -4.0;
        // t2 = root + T_ * t7;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        // Eigen::VectorXd t8(6);
        // t8 << -1.0, -1.0, -1.0, 0.0, 0.0, -3.0;
        // t2 = root + T_ * t8;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        // Eigen::VectorXd t9(6);
        // t9 << -1.0, -1.0, 0.0, 0.0, 0.0, -3.0;
        // t2 = root + T_ * t9;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        // Eigen::VectorXd t10(6);
        // t10 << -1.0, -1.0, 0.0, 0.0, 0.0, -2.0;
        // t2 = root + T_ * t10;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        // Eigen::VectorXd t11(6);
        // t11 << -1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
        // t2 = root + T_ * t11;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        // Eigen::VectorXd t12(6);
        // t12 << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        // t2 = root + T_ * t12;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        // Eigen::VectorXd t13(6);
        // t13 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        // t2 = root + T_ * t13;
        // std::cout << EigenToString(t2) << std::endl;
        // std::cout << distEquclid(t2, root, d_) << std::endl;
        int sign = -1;
        for (int j = 0; j < 2; ++j) {
            sign = -sign;
            for (int i = 0; i < d_; ++i) {
                Eigen::VectorXd newNeighbor = v + sign * T_.col(i);
                std::string newNeighStr = EigenToString(newNeighbor);
                // std::cout << open.size() << "," << openTemp.size() << std::endl;
                // if (visited.contains(newNeighStr)) {
                // bool haa = visited.contains(newNeighbor);
                // if (EigenToString(newNeighbor) == "-25.253000,-41.996900,18.003100,-41.996900,18.003100,18.003100,") {
                //     int x = 1;
                // }
                // if (EigenToString(newNeighbor) == "-40.463000,-37.462400,22.537600,-37.462400,22.537600,22.537600,") {
                //     int x = 1;
                // }
                // if (EigenToString(newNeighbor) == "-55.673000,-32.927900,27.072100,-32.927900,27.072100,27.072100,") {
                //     int x = 1;
                // }
                // if (EigenToString(newNeighbor) == "-35.928400,-32.927900,27.072100,-32.927900,27.072100,7.327500,") {
                //     int x = 1;
                // }
                // if (EigenToString(newNeighbor) == "-51.138400,-28.393400,31.606600,-28.393400,31.606600,11.862000,") {
                //     int x = 1;
                // }
                // if (EigenToString(newNeighbor) == "-31.393800,-28.393400,31.606600,-28.393400,11.862000,11.862000,") {
                //     int x = 1;
                // }
                // if (EigenToString(newNeighbor) == "-46.603800,-23.858900,36.141100,-23.858900,16.396500,16.396500,") {
                //     int x = 1;
                // }
                // if (EigenToString(newNeighbor) == "-26.859200,-23.858900,36.141100,-43.603500,16.396500,16.396500,") {
                //     int x = 1;
                // }
                // if (EigenToString(newNeighbor) == "-42.069200,-19.324400,40.675600,-39.069000,20.931000,20.931000,") {
                //     int x = 1;
                // }
                // if (EigenToString(newNeighbor) == "-22.324600,-19.324400,20.931000,-39.069000,20.931000,20.931000,") {
                //     int x = 1;
                // }
                // if (EigenToString(newNeighbor) == "-37.534600,-14.789900,25.465500,-34.534500,25.465500,25.465500,") {
                //     int x = 1;
                // }
                // if (EigenToString(newNeighbor) == "-17.790000,-34.534500,25.465500,-34.534500,25.465500,25.465500,") {
                //     int x = 1;
                // }


                if (visited.contains(newNeighbor) && visited[newNeighbor] < 2*d_) {
                    // visited[newNeighStr]++;
                    visited[newNeighbor]++;
                    // if (newNeighStr == "-33.000000,-30.000000,30.000000,-30.000000,30.000000,30.000000,") {
                    //     std::cout << visited[newNeighbor] << std::endl;
                    //     std::cout << newNeighStr << std::endl;
                    // }
                    // if (newNeighStr == "-22.859900,-33.023000,26.976900,-33.023000,26.976900,26.976900,") {
                    // if (newNeighStr == "-22.860000,-33.023000,26.977000,-33.023000,26.977000,26.977000,") {
                    //     std::cout << visited[newNeighbor] << std::endl;
                    //     std::cout << newNeighStr << std::endl;
                    // }
                    // nodes can only have up to 2d visits
                    // if (visited[newNeighStr] == 2 * d_) visited.erase(newNeighStr);
                    if (visited[newNeighbor] == 2 * d_)
                        visited.erase(newNeighbor);
                } else {
                    // visited[newNeighStr] = 1;
                    // if (newNeighStr == "-33.000000,-30.000000,30.000000,-30.000000,30.000000,30.000000,") {
                    // bool ha = visited.contains(newNeighbor);
                    // auto ha2 = TVectorEquals();
                    // bool ha3 = ha2(newNeighbor, startEigen_);
                    // bool ha4 = visited.contains(startEigen_);
                    // std::cout << vstr << std::endl;
                    // }
                    // if (newNeighStr == "-22.859944,-33.023048,26.976952,-33.023048,26.976952,26.976952,") {
                    // if (newNeighStr == "-22.860000,-33.023000,26.977000,-33.023000,26.977000,26.977000,") {
                    //     if (visited.contains(newNeighbor))
                    //         std::cout << "?A?!?!" << visited[newNeighbor] << std::endl;
                    //     auto a = visited.contains(bla);
                    //     std::cout << newNeighStr << std::endl;
                    //     int as = 1;
                    //     bla = Eigen::VectorXd(newNeighbor);
                    // }
                    // auto sad = TVectorEquals();
                    // auto sad2 = sad(newNeighbor, startEigen_);
                    // std::cout << "======================================" << std::endl;
                    // for (auto v: visited) {
                    //     std::cout << v.first << std::endl << "count=" << v.second << std::endl;
                    // }
                    visited[newNeighbor] = 1;
                    double newvNorm = 0;
                    bool inArea = true;

                    // // is it in the rectangle
                    // for (int w = 0; w < d_; ++w) {
                    //     inArea = inArea && (
                    //         (newNeighbor[w] > mapExtent_.low[w % dimRd_] || compareDoubles(newNeighbor[w], mapExtent_.low[w  % dimRd_])) &&
                    //         (newNeighbor[w] < mapExtent_.high[w % dimRd_] || compareDoubles(newNeighbor[w], mapExtent_.high[w  % dimRd_])));
                    // }

                    // newvNorm = (newNeighbor - root).norm();
                    newvNorm = distEquclid(newNeighbor, root, d_);
                    inArea = inArea &&  (newvNorm < r_ || compareDoubles(newvNorm, r_));
                    if (inArea) {
                        result.push_back(newNeighbor);
                        openTemp.push_back(newNeighbor);
                    }
                }
            }
        }
    }
    return result;
}

ompl::base::PlannerStatus ompl::geometric::LatticePRM::solve(const base::PlannerTerminationCondition &ptc)
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
        if (st != nullptr)
            goalM_.push_back(addMilestone(si_->cloneState(st), goalEigen_));
            // goalM_.push_back(addMilestone2(si_->cloneState(st), goalEigen_, true, false));

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
    std::cout << T_ << std::endl;
    auto t_start_vertices = std::chrono::high_resolution_clock::now();
    // AddAllVertices();
    if (buildAllGraph_) {
        OMPL_INFORM("Adding vertices.");
        // addSamplesInMapRegion(startEigen_);
        AddAllVerticesOld();
        std::cout << "added " << boost::num_vertices(g_) << " vertices." << std::endl;
    }
    auto t_start_edges = std::chrono::high_resolution_clock::now();
    if (buildAllGraph_) {
        OMPL_INFORM("Adding edges.");
        // AddAllEdges();
        AddAllEdgesOld();
        std::cout << "added " << boost::num_edges(g_) << " edges." << std::endl;
    }
    // AddAllEdges();
    // construct edges
    const long int solComponent = solutionComponent(&startGoalPair);
    Vertex startV = startM_[startGoalPair.first];
    Vertex goalV = goalM_[startGoalPair.second];
    OMPL_INFORM("Running A* on the PRM to find a solution");
    base::PathPtr solution;
    int solutionsChecked = 0;
    auto t_start_astar = std::chrono::high_resolution_clock::now();
    do
    {
        if (buildAllGraph_) {
            solution = constructSolution(startV, goalV);
        } else {
            solution = constructSolutionTest2(startV, goalV);
        }
        solutionsChecked++;
    } while (!solution && vertexComponentProperty_[startV] == vertexComponentProperty_[goalV]);
    std::cout << "Solutions checked overall: " << solutionsChecked << std::endl;
    auto t_end_astar = std::chrono::high_resolution_clock::now();
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
    if (buildAllGraph_) {
        vertices = duration_cast<std::chrono::microseconds>(t_start_edges - t_start_vertices).count();
        edges = duration_cast<std::chrono::microseconds>(t_start_astar - t_start_edges).count();
    }
    auto astar = duration_cast<std::chrono::microseconds>(t_end_astar - t_start_astar).count();
    OMPL_INFORM("Vertex creation: %lu S, edge creation: %lu S, astar runs: %lu S", vertices, edges, astar);
    runtime_ = vertices + edges + astar;
    runtimeAstar_ = astar;
    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);
    std::cout << "Edge count=" << boost::num_edges(g_) << std::endl;

    return bestSolution ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::LatticePRM::uniteComponents(Vertex a, Vertex b)
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

void ompl::geometric::LatticePRM::markComponent(Vertex v, unsigned long int newComponent)
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

long int ompl::geometric::LatticePRM::solutionComponent(std::pair<std::size_t, std::size_t> *startGoalPair) const
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

ompl::base::PathPtr ompl::geometric::LatticePRM::constructSolution(const Vertex &start, const Vertex &goal)
{
    // Need to update the index map here, becuse nodes may have been removed and
    // the numbering will not be 0 .. N-1 otherwise.
    unsigned long int index = 0;
    boost::graph_traits<Graph>::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(g_); vi != vend; ++vi, ++index)
        indexProperty_[*vi] = index;

    boost::property_map<Graph, boost::vertex_predecessor_t>::type prev;
    try
    {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(g_, start,
                            [this, goal](Vertex v)
                            {
                                return costHeuristic(v, goal);
                            },
                            boost::predecessor_map(prev)
                                .distance_compare([this](base::Cost c1, base::Cost c2)
                                                  {
                                                      return opt_->isCostBetterThan(c1, c2);
                                                  })
                                .distance_combine([this](base::Cost c1, base::Cost c2)
                                                  {
                                                      return opt_->combineCosts(c1, c2);
                                                  })
                                .distance_inf(opt_->infiniteCost())
                                .distance_zero(opt_->identityCost())
                                .visitor(AStarGoalVisitor<Vertex>(goal)));
    }
    catch (AStarFoundGoal &)
    {
    }
    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");

    // First, get the solution states without copying them, and check them for validity.
    // We do all the node validity checks for the vertices, as this may remove a larger
    // part of the graph (compared to removing an edge).
    std::vector<const base::State *> states(1, stateProperty_[goal]);
    std::set<Vertex> milestonesToRemove;
    for (Vertex pos = prev[goal]; prev[pos] != pos; pos = prev[pos])
    {
        states.push_back(stateProperty_[pos]);
    }

    // start is checked for validity already
    states.push_back(stateProperty_[start]);

    // Check the edges too, if the vertices were valid. Remove the first invalid edge only.
    std::vector<const base::State *>::const_iterator prevState = states.begin(), state = prevState + 1;
    Vertex prevVertex = goal, pos = prev[goal];
    do
    {
        prevState = state;
        ++state;
        prevVertex = pos;
        pos = prev[pos];
    } while (prevVertex != pos);

    auto p(std::make_shared<PathGeometric>(si_));
    for (std::vector<const base::State *>::const_reverse_iterator st = states.rbegin(); st != states.rend(); ++st)
        p->append(*st);
    return p;
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

Eigen::VectorXd ompl::geometric::LatticePRM::StateToEigen(const ob::State* state) const {
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

ompl::base::Cost ompl::geometric::LatticePRM::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}

void ompl::geometric::LatticePRM::getPlannerData(base::PlannerData &data) const
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
ob::State* ompl::geometric::LatticePRM::createNewState(Eigen::VectorXd& newV) {
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

    // auto vertices = boost::vertices(g_);
    // // std::unordered_map<std::vector<int>, Vertex> integerVectors_;
    // for (auto it = vertices.first; it != vertices.second; ++it) {
    //     Vertex v = *it;
    //     auto vState = stateProperty_[v];
    //     // all vertices should be here
    //     if (stateSpaceType_ == 3) {
    //         auto vStateSE2 = vState->as<base::SE2StateSpace::StateType>();
    //         std::vector<double> vInt = {vStateSE2->getX(), vStateSE2->getY(), vStateSE2->getYaw()};
    //         for (auto baseV: baseVecs) {
    //             std::vector<double> vIntNeighbor = {vInt[0] + baseV[0], vInt[0] + baseV[0], vInt[0] + baseV[0]};
    //             if (IntVecToVertex_.contains(vIntNeighbor)) {
    //                 Vertex neighborV = IntVecToVertex_[vIntNeighbor];
    //                 if (connectionFilter_(v, neighborV)) {
    //                     const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[neighborV]);
    //                     const Graph::edge_property_type properties(weight);
    //                     const Edge &e = boost::add_edge(v, neighborV, properties, g_).first;
    //                     edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
    //                     uniteComponents(v, neighborV);
    //                 }
    //             }
    //         }
    //     } else if (stateSpaceType_ == 6) {
    //         std::vector<int> vInt = VertexToIntVec_[v];
    //         // auto vStateSE2 = vState->as<base::SE3StateSpace::StateType>();
    //         // auto* vSE3 = new base::SE3StateSpace::StateType();
    //         // vSE3->setXYZ(vStateSE2->getX(), vStateSE2->getY(), vStateSE2->getZ());
    //         // vSE3->rotation().
    //         // std::vector<double> vInt = {vStateSE2->getX(), vStateSE2->getY(), vStateSE2->getZ(),
    //         //                             vStateSE2->rotation() vStateSE2->getYaw()};
    //         // for (auto baseV: baseVecs) {
    //         for (int i = 0; i < 6; ++i) {
    //             int sign = -1;
    //             for (int j = 0; j < 2; ++j) {
    //                 sign *= -1;
    //                 std::vector<double> vIntNeighbor;
    //                 vIntNeighbor[i] = vInt[i] + sign * ;
    //
    //                 for (int j = 0; j < 6; ++j) {
    //                     vIntNeighbor[i] = vInt[i] + baseV[i];
    //                 }
    //                 // std::vector<double> vIntNeighbor = {vInt[0] + baseV[0], vInt[0] + baseV[0], vInt[0] + baseV[0]};
    //                 if (IntVecToVertex_.contains(vIntNeighbor)) {
    //                     Vertex neighborV = IntVecToVertex_[vIntNeighbor];
    //                     const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[neighborV]);
    //                     const Graph::edge_property_type properties(weight);
    //                     const Edge &e = boost::add_edge(v, neighborV, properties, g_).first;
    //                     edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
    //                     uniteComponents(v, neighborV);
    //                 }
    //             }
    //         }
    //     }
    // }

// ompl::geometric::LatticePRM::Vertex ompl::geometric::LatticePRM::addMilestone2(base::State *state, Eigen::VectorXd& newV, bool start, bool goal)
// {
//     if (state == nullptr) {
//         // create the state
//         if (dimRd_ == 2) {
//             for (int i = 0; i < robotCount_; ++i) {
//                 workState_->as<ob::CompoundStateSpace::StateType>()->components[i]
//                           ->as<ob::SE2StateSpace::StateType>()->setXY(newV[2*i], newV[2*i+1]);
//                 workState_->as<ob::CompoundStateSpace::StateType>()->components[i]
//                           ->as<ob::SE2StateSpace::StateType>()->setYaw(0);
//             }
//         } else if (dimRd_ == 3) {
//             for (int i = 0; i < robotCount_; ++i) {
//                 workState_->as<ob::SE3StateSpace::StateType>()->setXYZ(newV[3*i], newV[3*i+1], newV[3*i+2]);
//                 changeAngleToQuaternion(0, 0, 0, workState_->as<ob::SE3StateSpace::StateType>());
//             }
//         }
//         // allocate it
//         state = si_->cloneState(workState_);
//     }
//     // TODO I'm trying to only add vertices that are valid. so semi-lazy.
//     Vertex m = nullptr;
//     if (si_->isValid(state)) {
//         m = boost::add_vertex(g_);
//         stateProperty_[m] = state;
//         vertexValidityProperty_[m] = VALIDITY_UNKNOWN;
//         unsigned long int newComponent = componentCount_++;
//         vertexComponentProperty_[m] = newComponent;
//         componentSize_[newComponent] = 1;
//         // my maps
//         // std::string newVfixedStr = EigenToString(angleShifted);
//         // EigenVecToVertex_[EigenToString(newV)] = m;
//         // if (start) {
//         //     startV_ = m;
//         // } else if (goal) {
//         //     goalV_ = m;
//         // } else {
//         // EigenVecToVertex_2[newV] = m;
//         // }
//         // strToEigen_[EigenToString(newV)] = newV;
//         // std::cout << EigenToString(newV) << std::endl;
//     }
//
//     // // TODO this is were the neighbors are picked. I need to update "connectionStrategy" to return all lattice neighbors.
//     // // Which milestones will we attempt to connect to?
//     // const std::vector<Vertex> &neighbors = connectionStrategy_(m);
//     // foreach (Vertex n, neighbors)
//     //     if (connectionFilter_(m, n))
//     //     {
//     //         const base::Cost weight = opt_->motionCost(stateProperty_[m], stateProperty_[n]);
//     //         const Graph::edge_property_type properties(weight);
//     //         const Edge &e = boost::add_edge(m, n, properties, g_).first;
//     //         edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
//     //         uniteComponents(m, n);
//     //     }
//     //
//     // nn_->add(m);
//
//     return m;
// }
//
// void ompl::geometric::LatticePRM::AddAllEdges2() {
//     // connect the lattice vertices
//     std::cout << EigenVecToVertex_.size() << "," << EigenVecToVertex_2.size() << std::endl;
//     for (auto& it: EigenVecToVertex_2) {
//         Eigen::VectorXi root = it.first;
//         if (!EigenVecToVertex_2.contains(root) || EigenVecToVertex_2[root] == nullptr) continue;
//         // if (root == startEigen_ || root == goalEigen_) continue;
//         goOverSamples2(0, root);
//         Eigen::VectorXd rootD = Eigen::VectorXd::Zero(dimRd_ * robotCount_);
//         for (int i = 0; i < dimRd_ * robotCount_; ++i) {
//             rootD += root[i] * T_.col(i);
//         }
//         // try also to connect the node to the start/goal
//         // TODO change this to direct connection, this is wasteful time-wise
//         std::vector<Eigen::VectorXd> startGoal = {startEigen_, goalEigen_};
//         int both = 0;
//         for (int i = 0; i < 2; ++i) {
//             auto neighbor = startGoal[i];
//         // }
//         // for (const auto& neighbor: {startEigen_, goalEigen_}) {
//             // auto t1 = std::chrono::high_resolution_clock::now();
//             double rootToNode = (neighbor - rootD).norm();
//             if (rootToNode < r_ || compareDoubles(rootToNode, r_)) {
//                 both++;
//                 std::cout << "i=" << i << "||" << EigenToString(neighbor) << "||" << EigenToString(rootD) << std::endl;
//                 Vertex rootVertex = EigenVecToVertex_2[root];
//                 Vertex neighborV = startM_[0];
//                 if (i == 1) {
//                     neighborV = goalM_[0];
//                 }
//                 // Vertex neighborV = EigenVecToVertex_2[neighbor];
//                 const base::Cost weight = opt_->motionCost(stateProperty_[rootVertex], stateProperty_[neighborV]);
//                 const Graph::edge_property_type properties(weight);
//                 const Edge &e = boost::add_edge(rootVertex, neighborV, properties, g_).first;
//                 edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
//                 uniteComponents(rootVertex, neighborV);
//             }
//         }
//         if (both == 2)
//             int x = 1;
//     }
//     // TODO I assume 1 goal and 1 start for the time being
// }
//
// void ompl::geometric::LatticePRM::AddAllVertices2() {
//     auto t1 = std::chrono::high_resolution_clock::now();
//     // goOverSamples(1, startEigen_);
//     auto t2 = std::chrono::high_resolution_clock::now();
//     // Eigen::VectorXd z = Eigen::VectorXd::Zero(dimRd_ * robotCount_);
//     Eigen::VectorXi z2 = Eigen::VectorXi::Zero(dimRd_ * robotCount_);
//     // goOverSamples(1, z);
//     auto t3 = std::chrono::high_resolution_clock::now();
//     goOverSamples2(1, z2);
//     auto t4 = std::chrono::high_resolution_clock::now();
//     auto dur1 = duration_cast<std::chrono::microseconds>(t2 - t1).count();
//     auto dur2 = duration_cast<std::chrono::microseconds>(t3 - t2).count();
//     auto dur3 = duration_cast<std::chrono::microseconds>(t4 - t3).count();
//     std::cout << "dur1=" << dur1 << ", dur2=" << dur2 << ", dur3=" << dur3 << std::endl;
// }

// // type: 0=ball (connecting edges), 1=box (sampling vertices)
// // ball is used in connecting edge, box is used in the start to create the vertices.
// void ompl::geometric::LatticePRM::goOverSamples2(int type, Eigen::VectorXi& root) {
//     bool testMode = false;
//     int d_ = dimRd_ * robotCount_;
//     std::vector<Eigen::VectorXi> open;
//     std::vector<Eigen::VectorXi> openTemp;
//     // std::unordered_map<Eigen::VectorXi, int, matrix_hash<Eigen::VectorXi>> visited;
//     std::unordered_map<Eigen::VectorXi, int, intHash> visited;
//     Eigen::VectorXd rootD = Eigen::VectorXd::Zero(d_);
//     for (int i = 0; i < d_; ++i) {
//         rootD[i] = root[i];
//     }
//     if (type == 1) {
//         Vertex rootV = addMilestone2(nullptr, rootD);
//         if (rootV != nullptr) {
//             EigenVecToVertex_2[root] = rootV;
//         }
//     }
//
//     // std::unordered_map<std::string, int> visited;
//     // long samples = 1;
//     // add root
//     // visited[EigenToString(root)] = 0;
//     visited[root] = 0;
//     open.push_back(root);
//     Vertex rootVertex = EigenVecToVertex_2[root];
//     // std::cout << "root=" << EigenToString(root) << std::endl;
//     // get the samples in the cube
//     while (!open.empty() || !openTemp.empty()) {
//         if (open.empty()) {
//             // we have the next wave of neighbors
//             open = openTemp;
//             openTemp.clear();
//         }
//         Eigen::VectorXi v = open[open.size() - 1];
//         open.pop_back();
//         std::string vstr = EigenToString(v);
//         // std::cout << vstr << std::endl;
//         int sign = -1;
//         for (int j = 0; j < 2; ++j) {
//             sign = -sign;
//             for (int i = 0; i < d_; ++i) {
//                 Eigen::VectorXi newNeighbor(v);
//                 newNeighbor[i] += sign;
//                 // std::cout << EigenToString(newNeighbor) << std::endl;
//                 // Eigen::VectorXd newNeighbor = v + sign * T_.col(i);
//                 // std::string newNeighStr = EigenToString(newNeighbor);
//                 // if (visited.contains(newNeighStr)) {
//                 if (visited.contains(newNeighbor)) {
//                     visited[newNeighbor]++;
//                     // visited[newNeighStr]++;
//                     // nodes can only have up to 2d visits
//                     // if (visited[newNeighStr] == 2 * d_) visited.erase(newNeighStr);
//                     if (visited[newNeighbor] == 2 * d_) visited.erase(newNeighbor);
//                 } else {
//                     // visited[newNeighStr] = 1;
//                     visited[newNeighbor] = 1;
//                     double newvNorm = 0;
//                     bool inArea = true;
//                     Eigen::VectorXd newNeighborActual = Eigen::VectorXd::Zero(d_);
//                     for (int t = 0; t < d_; ++t) {
//                         // std::cout << EigenToString(newNeighborActual) << std::endl;
//                         newNeighborActual += T_.col(t) * newNeighbor[t];
//                     }
//                     // std::cout << EigenToString(newNeighborActual) << std::endl;
//                     // is it in the rectangle
//                     for (int w = 0; w < d_; ++w) {
//                         inArea = inArea && (
//                             (newNeighborActual[w] > mapExtent_.low[w % dimRd_] || compareDoubles(newNeighborActual[w], mapExtent_.low[w  % dimRd_])) &&
//                             (newNeighborActual[w] < mapExtent_.high[w % dimRd_] || compareDoubles(newNeighborActual[w], mapExtent_.high[w  % dimRd_])));
//                     }
//
//                     if (type == 0) {
//                         newvNorm = (newNeighborActual - rootD).norm();
//                         // make sure we're also in a ball
//                         inArea = inArea &&  (newvNorm < r_ || compareDoubles(newvNorm, r_));
//                     }
//                     if (inArea) {
//                         if (type == 0) {
//                             // only connect to vertices that were valid
//                             // if (EigenVecToVertex_.contains(newNeighStr)) {
//                             if (EigenVecToVertex_2.contains(newNeighbor) && EigenVecToVertex_2[newNeighbor] != nullptr) {
//                                 // connect edges
//                                 // Vertex neighborV = EigenVecToVertex_[newNeighStr];
//                                 Vertex neighborV = EigenVecToVertex_2[newNeighbor];
//                                 // Vertex neighborV = EigenVecToVertex_2[newNeighbor];
//                                 // std::cout << EigenToString(newNeighbor) << std::endl;
//                                 const base::Cost weight = opt_->motionCost(stateProperty_[rootVertex], stateProperty_[neighborV]);
//                                 const Graph::edge_property_type properties(weight);
//                                 const Edge &e = boost::add_edge(rootVertex, neighborV, properties, g_).first;
//                                 // edges++;
//                                 edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
//                                 uniteComponents(rootVertex, neighborV);
//                             }
//                         } if (type == 1) {
//                             // Vertex newNeighborVertex = addMilestone(nullptr, newNeighbor);
//                             Vertex newNeighborVertex = addMilestone2(nullptr, newNeighborActual);
//                             if (newNeighborVertex != nullptr) {
//                                 EigenVecToVertex_2[newNeighbor] = newNeighborVertex;
//                             }
//                             // if (newNeighborVertex != nullptr) samples++;
//                         }
//                         if (type == 1 || !testMode) openTemp.push_back(newNeighbor);
//                     }
//                 }
//             }
//         }
//     }
// }

ompl::base::PathPtr ompl::geometric::LatticePRM::constructSolutionTest(const Vertex &start, const Vertex &goal) {
    // std::unordered_map<Eigen::VectorXd, int, matrix_hash<Eigen::VectorXd>> COLOR;
    std::unordered_map<Eigen::VectorXd, EigenAstar*, matrix_hash<Eigen::VectorXd>, TVectorEquals> VISITED;
    std::unordered_map<Eigen::VectorXd, bool, matrix_hash<Eigen::VectorXd>, TVectorEquals> INVALID;
    // std::unordered_map<std::string, EigenAstar*> VISITED;
    std::unordered_map<EigenAstar*, EigenAstar*> PREV;
    std::vector<EigenAstar*> OPEN;
    int d_ = dimRd_ * robotCount_;
    auto compareEigens = TVectorEquals();
    // init the root node
    auto* startV = new EigenAstar{
        // startEigen_, 0, (startEigen_ - goalEigen_).norm(), this->createNewState(startEigen_)
        /*startEigen_,*/ 0, distEquclid(startEigen_, goalEigen_,d_), this->createNewState(startEigen_), 1
    };
    VISITED[startEigen_] = startV;
    PREV[startV] = nullptr;
    OPEN.push_back(startV);
    // heapify OPEN
    std::ranges::make_heap(OPEN, EigenGT());
    // lazyA* loop
    // double minDist = std::numeric_limits<double>::infinity();
    // std::vector<Eigen::VectorXd> ballRNN = this->returnSamplesInBallNoBounds(startEigen_);
    // std::cout << "ballsize = " << ballRNN.size() << std::endl;
    int ib = 0;
    while (!OPEN.empty()) {
        std::cout << "size of visited=" << VISITED.size() << std::endl;
        // if (VISITED.size() == 5571) {
        // if (VISITED.size() == 4469) {
        // if (VISITED.size() > 40000) {
            // ib++;
            // if (ib > 0) {
                // for (auto elem: VISITED) {
                    // double distF = elem.second->g + elem.second->h;
                    // std::cout << "[" << distF << "]" << ": Visited elem: (" << EigenToString(elem.first) << ")." << std::endl;
                // }
            // }
        // }
        // get the min f-value vertex
        std::ranges::pop_heap(OPEN, EigenGT());
        EigenAstar* minV = OPEN.back();
        OPEN.pop_back();
        // Lazy part: only at this point we valudate the vertex/edge-to-parent
        if (!si_->isValid(minV->state)) {
            si_->freeState(minV->state);
            // INVALID[neighbor] = true;
            continue; // vertex in collision
        }
        if (PREV[minV] != nullptr) {
            bool isEdgeValid = si_->checkMotion(minV->state, PREV[minV]->state);
            if (!isEdgeValid) continue; // edge in collision TODO RETURN THIS
        }

        // std::cout << "minDist = " << minDist << ", curr_g = " << minV->g << ", curr_h = " << minV->h <<  ", with vec =" << EigenToString(StateToEigen(minV->state)) << std::endl;
        // if (minV->h < minDist) {
            // minDist = minV->h;
            // if (minDist == 0)
                // int x = 1;
        // }
        // std::cout << "?!?!1" << std::endl;
        Eigen::VectorXd minVeigen = StateToEigen(minV->state);
        std::cout << EigenToString(minVeigen) << std::endl;
        // have we reach the goal?
        if (compareEigens(minVeigen, goalEigen_)) {
            std::vector<const base::State*> resVec;
            // resVec.push_back(goalEigen_);
            // auto currV = VISITED[EigenToString(neighbor)];
            while (PREV[minV] != nullptr) {
                resVec.push_back(minV->state);
                minV = PREV[minV];
            }
            resVec.push_back(minV->state);
            // std::reverse(resVec.begin(), resVec.end());
            // for (const auto& v: resVec) {
            // std::cout << "--> " << EigenToString(v) << std::endl;
            // }
            auto p(std::make_shared<PathGeometric>(si_));
            for (std::vector<const base::State *>::const_reverse_iterator st = resVec.rbegin(); st != resVec.rend(); ++st) {
                p->append(*st);
                // auto a = (*st)->as<ob::SE2StateSpace::StateType>();
                // std::cout << a->getX() << "," << a->getY() << std::endl;
            }
            std::cout << "size of visited=" << VISITED.size() << std::endl;
            return p;
        }
        // std::cout << "?!?!2" << std::endl;
        // std::cout << "chosen v: " << EigenToString(minVeigen) << std::endl;
        std::vector<Eigen::VectorXd> neighbors = this->returnSamplesInBallNoBounds(minVeigen);
        // std::cout << "ballsize = " << neighbors.size() << std::endl;
        // std::cout << "?!?!3" << std::endl;
        // go over neighbors in a ball
        for (auto neighbor: neighbors) {
            // if (INVALID.contains(neighbor)) continue;
            // if (!si_->isValid(newNeighborState)) {
            //     si_->freeState(newNeighborState);
            //     INVALID[neighbor] = true;
            //     continue; // vertex in collision
            // }
            // double newG = minV->g + this->distEquclid2(newNeighborState, minV->state, d_);
            double newG = minV->g + distEquclid(neighbor, minVeigen,d_);
            double currG = VISITED.contains(neighbor) ? VISITED[neighbor]->g : DBL_MAX;
            if (newG < currG) {
                EigenAstar* neighborV = nullptr;
                if (VISITED.contains(neighbor)) {
                    neighborV = VISITED[neighbor];
                } else {
                    neighborV = new EigenAstar{
                        // neighbor, newG, (neighbor - goalEigen_).norm(), this->createNewState(neighbor)
                        /*neighbor, */newG, distEquclid(neighbor, goalEigen_,d_),
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
    }
    std::cout << "size of visited=" << VISITED.size() << std::endl;
    return nullptr;
}

ompl::base::PathPtr ompl::geometric::LatticePRM::constructSolutionTest2(const Vertex &start, const Vertex &goal) {
    // std::unordered_map<Eigen::VectorXd, int, matrix_hash<Eigen::VectorXd>> COLOR;
    std::unordered_map<Eigen::VectorXd, EigenAstar*, matrix_hash<Eigen::VectorXd>, TVectorEquals> VISITED;
    std::unordered_map<Eigen::VectorXd, bool, matrix_hash<Eigen::VectorXd>, TVectorEquals> INVALID;
    // std::unordered_map<std::string, EigenAstar*> VISITED;
    std::unordered_map<EigenAstar*, EigenAstar*> PREV;
    std::vector<EigenAstar*> OPEN;
    int d_ = dimRd_ * robotCount_;
    auto compareEigens = TVectorEquals();
    // init the root node
    auto* startV = new EigenAstar{
        // startEigen_, 0, (startEigen_ - goalEigen_).norm(), this->createNewState(startEigen_)
        /*startEigen_,*/ 0, distEquclid(startEigen_, goalEigen_,d_), this->createNewState(startEigen_), 1
    };
    VISITED[startEigen_] = startV;
    PREV[startV] = nullptr;
    OPEN.push_back(startV);
    // heapify OPEN
    std::ranges::make_heap(OPEN, EigenGT());
    // lazyA* loop
    // double minDist = std::numeric_limits<double>::infinity();
    Eigen::VectorXd zeroV = Eigen::VectorXd::Zero(d_);
    // std::vector<Eigen::VectorXd> ballRNN = this->returnSamplesInBallNoBounds(startEigen_);
    results_.samples = countSamplesInMap(startEigen_);
    auto tConstructionStart = std::chrono::high_resolution_clock::now();
    std::vector<Eigen::VectorXd> ballRNN = this->returnSamplesInBallNoBounds(zeroV);
    auto tConstructionEnd = std::chrono::high_resolution_clock::now();
    results_.constructionTime =
        duration_cast<std::chrono::microseconds>(tConstructionEnd - tConstructionStart).count();
    // int h = 1;
    // for (auto neighbor: ballRNN) {
    //     std::cout << h++ << ": " << EigenToString(neighbor) << std::endl;
    // }

    std::cout << "ball size=" << ballRNN.size() << std::endl;
    // int ib = 0;
    // auto tLast = std::chrono::high_resolution_clock::now();
    // auto tLatest = std::chrono::high_resolution_clock::now();
    while (!OPEN.empty()) {
        // std::cout << "size of visited=" << VISITED.size() << std::endl;
        // if (VISITED.size() == 5571) {
        // if (VISITED.size() >40000){// 4469) {
        //     ib++;
        //     if (ib > 0) {
        //         for (auto elem: VISITED) {
        //             double distF = elem.second->g + elem.second->h;
        //             std::cout << "[" << distF << "]" << ": Visited elem: (" << EigenToString(elem.first) << ")." << std::endl;
        //         }
        //     }
        // }
        // get the min f-value vertex
        std::ranges::pop_heap(OPEN, EigenGT());
        EigenAstar* minV = OPEN.back();
        OPEN.pop_back();
        Eigen::VectorXd minVeigen = StateToEigen(minV->state);
        // if (EigenToString(minVeigen) == "-48.115200,-45.115200,47.792500,-12.207500,14.884800,14.884800,")
        //     int x = 1;
        // if (EigenToString(minVeigen) == "-33.000000,-30.000000,30.000000,2.907700,30.000000,-2.907700,")
        //     int x = 1;
        // if (EigenToString(minVeigen) == "-35.677300,0.230400,27.322700,33.138100,27.322700, -5.585000")
        //     int x = 1;
        // if (EigenToString(minVeigen) == "-2.769600,0.230400,27.322700,33.138100,27.322700,-38.492700,")
        //     int x = 1;
        // if (EigenToString(minVeigen) == "27.460800,-2.446900,24.645400,30.460800,24.645400,-8.262300")
        //     int x = 1;
        // // Lazy part: only at this point we valudate the vertex/edge-to-parent
        // if (!si_->isValid(minV->state)) {
        //     si_->freeState(minV->state);
        //     // INVALID[neighbor] = true;
        //     continue; // vertex in collision
        // }
        // if (PREV[minV] != nullptr) {
        //     bool isEdgeValid = si_->checkMotion(minV->state, PREV[minV]->state);
        //     if (!isEdgeValid) continue; // edge in collision TODO RETURN THIS
        // }

        // std::cout << "minDist = " << minDist << ", curr_g = " << minV->g << ", curr_h = " << minV->h <<  ", with vec =" << EigenToString(StateToEigen(minV->state)) << std::endl;
        // if (minV->h < minDist) {
            // minDist = minV->h;
            // if (minDist == 0)
                // int x = 1;
        // }
        // std::cout << "?!?!1" << std::endl;
        // Eigen::VectorXd minVeigen = StateToEigen(minV->state);
        // std::cout << EigenToString(minVeigen) << std::endl;
        // if (EigenToString(minVeigen) == "29.162200,-27.072100,32.927900,32.161700,-26.305900,-6.561300,") {
        //     int qw = 1;
        //     if (EigenToString(minVeigen) == "29.162200,-27.072100,32.927900,32.161700,-26.305900,-6.561300,") {
        //         ib++;
        //         if (ib > 0) {
        //             for (auto elem: VISITED) {
        //                 double distF = elem.second->g + elem.second->h;
        //                 std::cout << "[" << distF << "]" << ": Visited elem: (" << EigenToString(elem.first) << ")." << std::endl;
        //             }
        //         }
        //     }
        // }
        // have we reach the goal?
        if (compareEigens(minVeigen, goalEigen_)) {
            std::vector<const base::State*> resVec;
            // resVec.push_back(goalEigen_);
            // auto currV = VISITED[EigenToString(neighbor)];
            while (PREV[minV] != nullptr) {
                resVec.push_back(minV->state);
                minV = PREV[minV];
            }
            resVec.push_back(minV->state);
            // std::reverse(resVec.begin(), resVec.end());
            // for (const auto& v: resVec) {
            // std::cout << "--> " << EigenToString(v) << std::endl;
            // }
            auto p(std::make_shared<PathGeometric>(si_));
            for (std::vector<const base::State *>::const_reverse_iterator st = resVec.rbegin(); st != resVec.rend(); ++st) {
                p->append(*st);
                // auto a = (*st)->as<ob::SE2StateSpace::StateType>();
                // std::cout << a->getX() << "," << a->getY() << std::endl;
            }
            std::cout << "size of visited=" << VISITED.size() << std::endl;
            auto tAstarEnd = std::chrono::high_resolution_clock::now();
            results_.astarTime =
                duration_cast<std::chrono::microseconds>(tAstarEnd - tConstructionEnd).count();
            return p;
        }
        // std::cout << "?!?!2" << std::endl;
        // std::cout << "chosen v: " << EigenToString(minVeigen) << std::endl;
        // std::vector<Eigen::VectorXd> neighbors = this->returnSamplesInBall(minVeigen);
        // std::cout << "?!?!3" << std::endl;
        // go over neighbors in a ball
        // Eigen::VectorXd ballShiftVector = minVeigen - startEigen_;
        bool hasGoal = false;
        if (!compareEigens(minVeigen, goalEigen_) && distEquclid(minVeigen, goalEigen_, d_) < r_) {
            ballRNN.push_back(goalEigen_ - minVeigen);
            hasGoal = true;
        }
        int t = 1;
        for (auto neighbor: ballRNN) {
            // tLatest = std::chrono::high_resolution_clock::now();
            // std::cout << "dur5=" << duration_cast<std::chrono::microseconds>(tLatest - tLast).count() << std::endl;
            // tLast = tLatest;
            // std::cout << t++ << ": " << EigenToString(neighbor) << std::endl;

            // neighbor += ballShiftVector; // shift the neighbor to its true place
            neighbor += minVeigen; // shift the neighbor to its true place
            // if (INVALID.contains(neighbor)) continue;
            // if (!si_->isValid(newNeighborState)) {
            //     si_->freeState(newNeighborState);
            //     INVALID[neighbor] = true;
            //     continue; // vertex in collision
            // }
            // double newG = minV->g + this->distEquclid2(newNeighborState, minV->state, d_);
            // std::cout << i++ << ": " << EigenToString(neighbor) << std::endl;
            // if (EigenToString(neighbor) == "-48.115200,-45.115200,47.792500,-12.207500,14.884800,14.884800,")
            //     int x = 1;
            // if (EigenToString(neighbor) == "-33.000000,-30.000000,30.000000,2.907700,30.000000,-2.907700,")
            //     int x = 1;
            // if (EigenToString(neighbor) == "37.695700,-25.119700,34.880300,40.695700,-30.935100,1.972600,")
            //     int x = 1;
            // if (EigenToString(minVeigen) == "27.460800,-2.446900,24.645400,30.460800,24.645400,-8.262300")
            //     int x = 1;
            // if (EigenToString(minVeigen) == "-35.677300,0.230400,27.322700,33.138100,27.322700, -5.585000")
            //     int x = 1;
            // vertex/edge validity checking
            if (INVALID.contains(neighbor) && INVALID[neighbor] == true) continue;
            auto neigbhorState = this->createNewState(neighbor);
            if (!si_->isValid(neigbhorState)) {
                    si_->freeState(neigbhorState);
                    INVALID[neighbor] = true;
                    continue; // vertex in collision
            }
            bool isEdgeValid = si_->checkMotion(minV->state, neigbhorState);
            if (!isEdgeValid) continue; // edge in collision TODO RETURN THIS

            double newG = minV->g + distEquclid(neighbor, minVeigen,d_);
            double currG = VISITED.contains(neighbor) ? VISITED[neighbor]->g : DBL_MAX;
            // std::cout << newG << ": " << EigenToString(neighbor) << std::endl;
            if (newG < currG) {
                EigenAstar* neighborV = nullptr;
                if (VISITED.contains(neighbor)) {
                    neighborV = VISITED[neighbor];
                } else {
                    neighborV = new EigenAstar{
                        // neighbor, newG, (neighbor - goalEigen_).norm(), this->createNewState(neighbor)
                        /*neighbor, */newG, distEquclid(neighbor, goalEigen_,d_),
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
        // tLatest = std::chrono::high_resolution_clock::now();
        // std::cout << "dur6=" << duration_cast<std::chrono::microseconds>(tLatest - tLast).count() << std::endl;
        // tLast = tLatest;
        if (hasGoal) ballRNN.pop_back(); // remove the goal.
    }
    std::cout << "size of visited=" << VISITED.size() << std::endl;
    auto tAstarEnd = std::chrono::high_resolution_clock::now();
    results_.astarTime =
        duration_cast<std::chrono::microseconds>(tAstarEnd - tConstructionEnd).count();
    return nullptr;
}

#pragma endregion unused_funcs
