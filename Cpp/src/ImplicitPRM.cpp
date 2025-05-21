//
// Created by itai on 11/5/24.
//

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

#include "../include/ImplicitPRM.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/tools/config/MagicConstants.h>

#include <Eigen/Dense>
#include "NTL/matrix.h"
#include "NTL/LLL.h"
#include "NTL/RR.h"
#include "NTL/mat_ZZ.h"
#include "NTL/mat_RR.h"

// #include "flint/flint.h"

namespace ob = ompl::base;
using namespace NTL;
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

#pragma region setup methods

ompl::geometric::ImplicitPRM::ImplicitPRM(const base::SpaceInformationPtr &si, bool starStrategy)
  : base::Planner(si, "ImplicitPRM")
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
  , sameRadius_(false)
  , countSamplesForRND_(false)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;


    dimRd_ = mapExtent_.low.size();
    angleFix_ = 1;
    workState_ = si_->allocState();


    Planner::declareParam<double>("range", this, &ImplicitPRM::setRange, &ImplicitPRM::getRange, "0.:1.:10000.");
    if (!starStrategy_)
        Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &ImplicitPRM::setMaxNearestNeighbors,
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

ompl::geometric::ImplicitPRM::ImplicitPRM(const base::PlannerData &data, bool starStrategy)
  : ImplicitPRM(data.getSpaceInformation(), starStrategy)
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
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) {
            // return distanceFunction(a, b);
                return euclidDistanceFunction(a, b);
        });

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

ompl::geometric::ImplicitPRM::~ImplicitPRM()
{
    si_->freeState(workState_);
    clear();
};

void ompl::geometric::ImplicitPRM::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    OMPL_INFORM("Chose the following PRM type: %s", EnumToString(prmType_));

    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        // nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                 // {
                                     // return distanceFunction(a, b);
                                 // });
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) {
            // return distanceFunction(a, b);
            return euclidDistanceFunction(a, b);
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
    if (!sampler_ && prmType_ == Random)
        sampler_ = si_->allocStateSampler();
}

void ompl::geometric::ImplicitPRM::setRange(double distance)
{
    maxDistance_ = distance;
    if (!userSetConnectionStrategy_)
        setDefaultConnectionStrategy();
    if (isSetup())
        setup();
}

void ompl::geometric::ImplicitPRM::setMaxNearestNeighbors(unsigned int k)
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

void ompl::geometric::ImplicitPRM::setDefaultConnectionStrategy()
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

void ompl::geometric::ImplicitPRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::ImplicitPRM::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::ImplicitPRM::clearValidity()
{
    foreach (const Vertex v, boost::vertices(g_))
        vertexValidityProperty_[v] = VALIDITY_UNKNOWN;
    foreach (const Edge e, boost::edges(g_))
        edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
}

void ompl::geometric::ImplicitPRM::clear()
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

void ompl::geometric::ImplicitPRM::freeMemory()
{
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();
}

#pragma endregion setup methods

#pragma region Eigen funcs

Eigen::VectorXd ompl::geometric::ImplicitPRM::StateToEigen(const ob::State* state) const {
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

// function to display the eigen vector
static std::string EigenToString(const Eigen::VectorXd& v){
    std::string res = "";
    for (int i = 0; i < v.size(); ++i) {
        int valint = std::round(v(i) * pow(10, 6));
        double val = valint / pow(10, 6);
        res += std::to_string(val) + ",";
    }
    return res;
}

// function to display the eigen vector
static std::string EigenToString(const Eigen::VectorXi& v){
    std::string res = "";
    for (int i = 0; i < v.size(); ++i) {
        res += std::to_string(v[i]) + ",";
    }
    return res;
}

#pragma endregion Eigen funcs

#pragma region type==RANDOM

void ompl::geometric::ImplicitPRM::AddAllVerticesWithNN() {
    long sampleCount = 0;
    if (prmType_ == LatticeWithNN) {
        goOverSamples(startEigen_); // add all valid samples in the map
    } else {
        while (sampleCount++ < sampleLimit_) {
            sampler_->sampleUniform(workState_);
            auto newV = StateToEigen(workState_);
            addMilestone(si_->cloneState(workState_), newV);
        }
    }
}

Eigen::VectorXd ompl::geometric::ImplicitPRM::getValidStartState() {
    sampler_ = si_->allocStateSampler();
    sampler_->sampleUniform(workState_);
    while (!si_->isValid(workState_)) {
        sampler_->sampleUniform(workState_);
    }
    return StateToEigen(workState_);
}

#pragma endregion type==RANDOM

#pragma region type==LATTICE

void ompl::geometric::ImplicitPRM::setLatticeType(LatticeType type, double delta, double epsilon) {
    LatticeType_ = type;
    delta_ = delta;
    epsilon_ = epsilon;

    CreateLatticeParameters();
}

void ompl::geometric::ImplicitPRM::CreateLatticeParameters() {
    int dim = dimRd_ * robotCount_;
    double rescale_ = 0;
    std::vector<double> resses = {1};
    for (const auto& res: resses) {
        resData_.push_back(ResParams());
        double resDelta = res * delta_;
        resData_.back().delta = resDelta;
        // this radius is defined in Dayan et. al (23')
        double resR = (2*(epsilon_ + 1)* resDelta) / std::sqrt(1+epsilon_ * epsilon_);
        resData_.back().r = resR;
        if (prmType_ == LatticeWithNN) r_ = resR;
        double resBeta = (resDelta * epsilon_) / std::sqrt(1 + pow(epsilon_, 2));
        // the generator matrices are from the paper
        switch (LatticeType_) {
            case File: {
                break;
            }
            case Zn: {
                rescale_ = (2 * resBeta) / sqrt(dim);
                // Zn rescaled generator
                resData_.back().T = Eigen::MatrixXd(dim, dim);
                for (int j = 0; j < dim; ++j) {
                    for (int w = 0; w < dim; ++w) {
                        if (j == w) {
                            resData_.back().T(j, w) = roundDbl(rescale_);
                        } else {
                            resData_.back().T(j, w) = 0;
                        }
                    }
                }
                break;
            }
            case DnStar: {
                if (dim % 2 == 0) {
                    rescale_ = sqrt(8.0 / dim) * resBeta;
                } else {
                    rescale_ =  (4*resBeta) / sqrt(2.0 * dim - 1);
                }
                // DnStar rescaled generator
                resData_.back().T = Eigen::MatrixXd(dim, dim);
                for (int j = 0; j < dim; ++j) {
                    for (int w = 0; w < dim; ++w) {
                        if (j < dim - 1) {
                            if (j == w) {
                                resData_.back().T(j, w) = roundDbl(rescale_);
                            } else {
                                resData_.back().T(j, w) = 0;
                            }
                        } else {
                            resData_.back().T(j, w) = roundDbl(rescale_ * 0.5);
                        }
                    }
                }
                resData_.back().T = resData_.back().T.transpose().eval();
                break;
            }
            case AnStar: {
                rescale_ = sqrt((12.0 * (dim + 1)) / (dim * (dim + 2))) * resBeta;
                double anstar_x = 1.0 / (dim + 1 - sqrt(dim + 1));
                // DnStar rescaled generator
                resData_.back().T = Eigen::MatrixXd(dim, dim);
                for (int j = 0; j < dim; ++j) {
                    for (int w = 0; w < dim; ++w) {
                        if (j == 0) {
                            if (w < dim - 1) {
                                resData_.back().T(j, w) = roundDbl(rescale_);
                            } else {
                                resData_.back().T(j, w) = roundDbl((rescale_ * (anstar_x - 1)));
                            }
                        } else {
                            if (w == j - 1) {
                                resData_.back().T(j, w) = roundDbl(-rescale_);
                            } else if (w < dim - 1) {
                                resData_.back().T(j, w) = 0;
                            } else {
                                resData_.back().T(j, w) = roundDbl(rescale_ * anstar_x);
                            }
                        }
                    }
                }
                break;
            }
            case AnStarReduced: {
                rescale_ = sqrt((12.0 * (dim + 1)) / (dim * (dim + 2))) * resBeta;
                double anstar_x = 1.0 / (dim + 1 - sqrt(dim + 1));
                // DnStar rescaled generator
                resData_.back().T = Eigen::MatrixXd(dim, dim);
                for (int j = 0; j < dim; ++j) {
                    for (int w = 0; w < dim; ++w) {
                        if (j == 0) {
                            if (w < dim - 1) {
                                resData_.back().T(j, w) = roundDbl(rescale_);
                            } else {
                                resData_.back().T(j, w) = roundDbl((rescale_ * (anstar_x - 1)));
                            }
                        } else {
                            if (w == j - 1) {
                                resData_.back().T(j, w) = roundDbl(-rescale_);
                            } else if (w < dim - 1) {
                                resData_.back().T(j, w) = 0;
                            } else {
                                resData_.back().T(j, w) = roundDbl(rescale_ * anstar_x);
                            }
                        }
                    }
                }

                // perform LLL reduction on a scaled-to-int version of the regular gram matrix
                mat_ZZ reducedL_NTL;
                int intFactor = 1000000;
                reducedL_NTL.SetDims(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        reducedL_NTL[i][j] = to_ZZ(static_cast<int>(resData_.back().T(j, i) * intFactor));
                    }
                }
                LLL_FP(reducedL_NTL);
                // go back to the real result
                Eigen::MatrixXd reducedL = Eigen::MatrixXd::Zero(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        reducedL(i, j) = static_cast<double>(to_int(reducedL_NTL[j][i])) / intFactor;
                    }
                }
                // copy results
                resData_.back().T = Eigen::MatrixXd(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        resData_.back().T(i, j) = reducedL(i, j);
                    }
                }
                break;
            }
            case Lc2: {
                double coverR = 1;
                Eigen::MatrixXd M(dim, dim);
                if (dim == 6) {
                    // we have the GRAM matrix
                    // L6 from the paper
                     M << 1.9982, 0.5270, -0.4170, -0.5270, 0.5270, -1.0541,
                          0.5270, 1.9982, -0.4170, -0.5270, 0.5270, -1.0541,
                          -0.4170, -0.4170, 2.1082, -1.0541, -0.4170, 0.8341,
                          -0.5270, -0.5270, -1.0541, 1.9982, -0.5270, -0.4170,
                          0.5270, 0.5270, -0.4170, -0.5270, 1.9982, -1.0541,
                          -1.0541, -1.0541, 0.8341, -0.4170, -1.0541, 2.1082;
                    double density = 2.466125;
                    double det = M.determinant();
                    coverR = pow((density * sqrt(det)) / unitBallVol(dim), 2.0 / dim);
                // } else if (dim == 8) {
                //     // irrelevant stuff for testing
                //     M <<
                //         3.1917733003, -1.5938256841, -0.0000000000, 0.0000000000, -0.0000000000, -0.0000000000, -0.0000000000, 0.8051567063,
                //         -1.5938256841, 1.9158885426, -1.1726527018, -0.0000000000, 0.0000000000, -0.0000000000, 0.0000000000, 0.0000004254,
                //         -0.0000000000, -1.1726527018, 2.3453054037, -1.1726527018, 0.0000000000, -0.0000000000, -0.0000000000, 0.0000000000,
                //         0.0000000000, -0.0000000000, -1.1726527018, 2.3453054037, -1.1726527018, -0.0000000000, -0.0000000000, 0.0000000000,
                //         -0.0000000000, 0.0000000000, 0.0000000000, -1.1726527018, 2.3453054037, -1.1726527018, -0.0000000000, -0.0000000000,
                //         -0.0000000000, -0.0000000000, -0.0000000000, -0.0000000000, -1.1726527018, 2.3453054037, -1.1726527018, 0.0000000000,
                //         -0.0000000000, 0.0000000000, -0.0000000000, -0.0000000000, -0.0000000000, -1.1726527018, 2.3453054037, -0.0000000000,
                //         0.8051567063, 0.0000004254, 0.0000000000, 0.0000000000, -0.0000000000, 0.0000000000, -0.0000000000, 1.6103149015;
                //     double det = M.determinant();
                //     coverR = 1.0;
                // } else if (dim == 10) {
                // // irrelevant stuff for testing
                //     M <<  2.0408, -1.0204, 0, 0, 0, 0, 0, 0, 0, 0,
                //           -1.0204, 2.0408, -1.0204, 0, 0, 0, 0, 0, -1.0204, 0,
                //           0, -1.0204, 2.0408, -1.0204, 0, 0, 0, 0, 0, 1.0204,
                //           0, 0, -1.0204, 2.0408, -1.0204, 0, 0, 0, 0, -1.0204,
                //           0, 0, 0, -1.0204, 2.0408, -1.0204, 0, 0, 0, 1.0204,
                //           0, 0, 0, 0, -1.0204, 2.0408, -1.0204, 0, 0, 0,
                //           0, 0, 0, 0, 0, -1.0204, 2.0408, -1.0204, 0, 0,
                //           0, 0, 0, 0, 0, 0, -1.0204, 2.0408, 0, -1.0204,
                //           0, -1.0204, 0, 0, 0, 0, 0, 0, 1.6326, -1.0204,
                //           0, 0, 1.0204, -1.0204, 1.0204, 0, 0, -1.0204, -1.0204, 3.0612;
                //     double det = M.determinant();
                //     coverR = 1.0;
                }
                // double coverR = 3.6399222478465464436606629511175356721;
                rescale_ = coverR * resBeta;

                Eigen::LLT<Eigen::MatrixXd> llt(M);
                if (llt.info() != Eigen::Success) {
                    std::cerr << "Cholesky decomposition failed. Matrix may not be SPD.\n";
                }
                std::cout << "Matrix A:\n" << M << "\n\n";

                // Extract the lower triangular matrix L
                Eigen::MatrixXd L = llt.matrixL();
                // L.transposeInPlace();
                L = L * rescale_;
                std::cout << "Cholesky matrix L:\n" << L << "\n\n";
                // get the LLL-reduced form
                mat_ZZ reducedL_NTL;
                int intFactor = 1000000;
                reducedL_NTL.SetDims(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        reducedL_NTL[i][j] = to_ZZ(static_cast<int>(L(i, j) * intFactor));
                        // reducedL_NTL[i][j] = to_ZZ(intFactor);
                    }
                }
                // LLL(reducedL_NTL, 0, 1e-6);
                LLL_FP(reducedL_NTL);

                Eigen::MatrixXd reducedL = Eigen::MatrixXd::Zero(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        reducedL(i, j) = static_cast<double>(to_int(reducedL_NTL[i][j])) / intFactor;
                    }
                }

                // Print results
                std::cout << "Reduced L:\n" << reducedL << "\n";

                // std::cout << "Redced L:\n" << reducedL_NTL << "\n";
                resData_.back().T = Eigen::MatrixXd(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        resData_.back().T(i, j) = reducedL(i, j);
                    }
                }
                resData_.back().T = resData_.back().T.transpose().eval();

                // std::cout << "Reduced L:\n" << reducedL << "\n";
                break;
            }
            case Lc1: {
                double coverR = 1;
                Eigen::MatrixXd M(dim, dim);
                if (dim == 6) {
                    // we have the GRAM matrix
                    // L6 from the paper
                    M <<  2.0550, -0.9424, 1.1126, 0.2747, -0.9424, -0.6153,
                        -0.9424, 1.9227, -0.5773, -0.7681, 0.3651, -0.3651,
                        1.1126, -0.5773, 2.0930, -0.4934, -0.5773, -0.9804,
                        0.2747, -0.7681, -0.4934, 1.7550, -0.7681, 0.7681,
                        -0.9424, 0.3651, -0.5773, -0.7681, 1.9227, -0.3651,
                        -0.6153, -0.3651, -0.9804, 0.7681, -0.3651, 1.9227;
                    std::cout << M << std::endl;
                    double asdsad = (M * M.transpose()).determinant();
                    double density = 2.464802;
                    double det = M.determinant();
                    coverR = pow((density * sqrt(det)) / unitBallVol(dim), 2.0 / dim);
                    std::cout << density << "," << coverR << std::endl;
                }
                rescale_ = coverR * resBeta;
                Eigen::LLT<Eigen::MatrixXd> llt(M);
                if (llt.info() != Eigen::Success) {
                    std::cerr << "Cholesky decomposition failed. Matrix may not be SPD.\n";
                }
                std::cout << "Matrix A:\n" << M << "\n\n";

                // Extract the lower triangular matrix L
                Eigen::MatrixXd L = llt.matrixL();
                L = L * rescale_;
                std::cout << "Cholesky matrix L:\n" << L << "\n\n";
                // get the LLL-reduced form (making sure we use it on an integer matrix)
                mat_ZZ reducedL_NTL;
                int intFactor = 1000000;
                reducedL_NTL.SetDims(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        reducedL_NTL[i][j] = to_ZZ(static_cast<int>(L(i, j) * intFactor));
                    }
                }
                LLL_FP(reducedL_NTL);

                Eigen::MatrixXd reducedL = Eigen::MatrixXd::Zero(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        reducedL(i, j) = static_cast<double>(to_int(reducedL_NTL[i][j])) / intFactor;
                    }
                }

                // Print results
                std::cout << "Reduced L:\n" << reducedL << "\n";

                // std::cout << "Redced L:\n" << reducedL_NTL << "\n";
                resData_.back().T = Eigen::MatrixXd(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        resData_.back().T(i, j) = reducedL(i, j);
                    }
                }
                resData_.back().T = resData_.back().T.transpose().eval();
                break;
            }
            case Lc1b: {
                // this lattice is from the SITE, not the paper. it should be similar to Lc1.
                double coverR = 1;
                Eigen::MatrixXd M(dim, dim);
                if (dim == 6) {
                    // we have the GRAM matrix
                    // L6 from the website
                    M <<  6.38790917413146, 2.59193944942097, 3.59193944942097, 3.59193944942097, -1, 1.79596972471048,
                    2.59193944942097, 8.79546262888381, 4.55496083132191, 4.55496083132191, 4.24050179756189, 2.27748041566095,
                    3.59193944942097, 4.55496083132191, 7.79453203804591, 2.12481713085996, 2.43014370046194, 3.89726601902295,
                    3.59193944942097, 4.55496083132191, 2.12481713085996, 7.79453203804591, 2.43014370046194, 3.89726601902295,
                    -1, 4.24050179756189, 2.43014370046194, 2.43014370046194, 7.48007300428582, 4.04992930382394,
                    1.79596972471048, 2.27748041566095, 3.89726601902295, 3.89726601902295, 4.04992930382394, 7.61834791669742;
                    // double density = 2.464802;
                    coverR = 3.6399222478465464436606629511175356721;
                    M = M / (coverR);
                    double det = M.determinant();
                    coverR = 1;
                    double density = unitBallVol(dim) * sqrt(pow(coverR, dim) / det);
                    std::cout << density << std::endl;
                }
                rescale_ = coverR * resBeta;
                Eigen::LLT<Eigen::MatrixXd> llt(M);
                if (llt.info() != Eigen::Success) {
                    std::cerr << "Cholesky decomposition failed. Matrix may not be SPD.\n";
                }
                std::cout << "Matrix A:\n" << M << "\n\n";

                // Extract the lower triangular matrix L
                Eigen::MatrixXd L = llt.matrixL();
                L = L * rescale_;
                std::cout << "Cholesky matrix L:\n" << L << "\n\n";
                // get the LLL-reduced form
                mat_ZZ reducedL_NTL;
                int intFactor = 1000000;
                reducedL_NTL.SetDims(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        reducedL_NTL[i][j] = to_ZZ(static_cast<int>(L(i, j) * intFactor));
                    }
                }
                LLL_FP(reducedL_NTL);

                Eigen::MatrixXd reducedL = Eigen::MatrixXd::Zero(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        reducedL(i, j) = static_cast<double>(to_int(reducedL_NTL[i][j])) / intFactor;
                    }
                }

                // Print results
                std::cout << "Reduced L:\n" << reducedL << "\n";

                resData_.back().T = Eigen::MatrixXd(dim, dim);
                for (int i = 0; i < dim; ++i) {
                    for (int j = 0; j < dim; ++j) {
                        resData_.back().T(i, j) = reducedL(i, j);
                    }
                }
                resData_.back().T = resData_.back().T.transpose().eval();
                break;
            }
        }
        std::cout << resData_.back().T << std::endl;
        maxEdge_ = 0; // TEMP
        OMPL_INFORM("Added LPRM Parameters: d=%d, R=%f,delta=%f,epsilon=%f",
                    dim, resR, resDelta, epsilon_);
    }
}

void ompl::geometric::ImplicitPRM::setStartAndGoalEigen(const Eigen::VectorXd &startEigen, const Eigen::VectorXd &goalEigen) {
    startEigen_ = startEigen;
    goalEigen_ = goalEigen;
}

// count samples in the ball, but ignore bounds so you dont miss points while doing the BFS
void ompl::geometric::ImplicitPRM::setSamplesInBallNoBoundsRes(int id, Eigen::VectorXd &root) {
    int d_ = dimRd_ * robotCount_;
    std::vector<LatticeNN> open;
    std::vector<LatticeNN> openTemp;
    std::unordered_map<Eigen::VectorXi, int, matrix_hashI<Eigen::VectorXi>, TVectorEqualsI> visited;
    long samples = 1;
    int power = 2;
    // add root

    Eigen::VectorXi currI = Eigen::VectorXi::Zero(d_);
    visited[currI] = 0;
    open.push_back({root, currI, 0});

    const auto& T = resData_[id].T;
    const auto r = resData_[id].r;
    std::cout << "Using the following generator matrix:" << std::endl;
    std::cout << T << std::endl;

    while (!open.empty() || !openTemp.empty()) {
        if (open.empty()) {
            // we have the next wave of neighbors
            open = openTemp;
            openTemp.clear();
        }
        const auto [sample, sampleI, g, isGoal] = open[open.size() - 1];
        open.pop_back();
        int sign = -1;
        for (int j = 0; j < 2; ++j) {
            sign = -sign;
            for (int i = 0; i < d_; ++i) {
                Eigen::VectorXd newNeighbor = sample + sign * T.col(i);
                Eigen::VectorXi newNeighborI = sampleI;
                newNeighborI[i] += sign;
                if (visited.contains(newNeighborI) && visited[newNeighborI] < 2*d_) {
                    visited[newNeighborI]++;
                    if (visited[newNeighborI] == 2 * d_)
                        visited.erase(newNeighborI);
                } else {
                    visited[newNeighborI] = 1;
                    double newvNorm = distEuclid(newNeighbor, root, d_);
                    if (newvNorm < r || compareDoubles(newvNorm, r)) {
                        int incAmount = std::pow(power, id);
                        // scale the neighborI according to the resses
                        // this is an attempt to create "resolutions" in the lattice, currently unused
                        resData_[id].neighbors.push_back({newNeighbor, incAmount * newNeighborI,
                        distEuclid(newNeighbor, root, d_)});
                        // std::cout << EigenToString(newNeighbor) << std::endl;
                        // the regular ball-search is unscaled
                        openTemp.push_back(
                            {newNeighbor, newNeighborI,
                                distEuclid(newNeighbor, root, d_)});
                    }
                }
            }
        }
    }
}

// count samples in the entire map bounds
long ompl::geometric::ImplicitPRM::countSamplesInMapBounds(int id, Eigen::VectorXd &root) {
    int d_ = dimRd_ * robotCount_;
    std::vector<LatticeNN> open;
    std::vector<LatticeNN> openTemp;
    std::unordered_map<Eigen::VectorXi, int, matrix_hashI<Eigen::VectorXi>, TVectorEqualsI> visited;
    long samples = 1;
    int power = 2;
    // add root
    Eigen::VectorXi currI = Eigen::VectorXi::Zero(d_);
    visited[currI] = 0;
    open.push_back({root, currI, 0});

    const auto& T = resData_[id].T;

    // std::cout << T << std::endl;

    while (!open.empty() || !openTemp.empty()) {
        if (open.empty()) {
            // we have the next wave of neighbors
            open = openTemp;
            openTemp.clear();
        }
        const auto [sample, sampleI, g, isGoal] = open[open.size() - 1];
        open.pop_back();
        int sign = -1;
        for (int j = 0; j < 2; ++j) {
            sign = -sign;
            for (int i = 0; i < d_; ++i) {
                Eigen::VectorXd newNeighbor = sample + sign * T.col(i);
                Eigen::VectorXi newNeighborI = sampleI;
                newNeighborI[i] += sign;
                if (visited.contains(newNeighborI) && visited[newNeighborI] < 2*d_) {
                    visited[newNeighborI]++;
                    if (visited[newNeighborI] == 2 * d_)
                        visited.erase(newNeighborI);
                } else {
                    visited[newNeighborI] = 1;

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
                    if (inArea) {
                        samples++;
                        // uncomment this if you want to see the sample count
                        // if (samples % 1000 == 0) std::cout << samples << std::endl;
                    }
                    // we need to explore all space, regardless of collisions.
                    // some samples may be "isolated" in terms of direct connection to another
                    // sample, but in a PRM r-ball they may end up connecting.
                    if (inAreaBigger) {
                        openTemp.push_back({
                            newNeighbor, newNeighborI, 0
                        });
                    }
                }
            }
        }
    }
    return samples;
}

#pragma endregion type==LATTICE

#pragma region the algorithm

ob::State* ompl::geometric::ImplicitPRM::createNewState(Eigen::VectorXd& newV) {
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

void ompl::geometric::ImplicitPRM::changeAngleToQuaternion(double u1, double u2, double u3, ob::SE3StateSpace::StateType* state) {
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

ompl::geometric::ImplicitPRM::Vertex ompl::geometric::ImplicitPRM::addMilestone(base::State *state, Eigen::VectorXd& newV)
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

    Vertex m = nullptr;
    m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    vertexValidityProperty_[m] = VALIDITY_UNKNOWN;
    unsigned long int newComponent = componentCount_++;
    vertexComponentProperty_[m] = newComponent;
    componentSize_[newComponent] = 1;
    EigenVecToVertex_[newV] = m;
    if (prmType_ == Random || prmType_ == LatticeWithNN || sameRadius_)
        nn_->add(m);

    return m;
}

ompl::base::PlannerStatus ompl::geometric::ImplicitPRM::solve(const base::PlannerTerminationCondition &ptc)
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

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    bestCost_ = opt_->infiniteCost();
    std::pair<std::size_t, std::size_t> startGoalPair;
    base::PathPtr bestSolution;
    unsigned int optimizingComponentSegments = 0;
    // add all valid vertices and ALL edges
    const long int solComponent = solutionComponent(&startGoalPair);
    Vertex startV = startM_[startGoalPair.first];
    Vertex goalV = goalM_[startGoalPair.second];
    base::PathPtr solution = constructSolutionImplicitly(startV, goalV, ptc);
    if (solution)
    {
        base::Cost c = solution->cost(opt_);
        if (opt_->isSatisfied(c))
        {
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

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);
    return bestSolution ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::ImplicitPRM::uniteComponents(Vertex a, Vertex b)
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

void ompl::geometric::ImplicitPRM::markComponent(Vertex v, unsigned long int newComponent)
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

long int ompl::geometric::ImplicitPRM::solutionComponent(std::pair<std::size_t, std::size_t> *startGoalPair) const
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

ompl::base::Cost ompl::geometric::ImplicitPRM::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}

void ompl::geometric::ImplicitPRM::getPlannerData(base::PlannerData &data) const
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

ompl::base::PathPtr ompl::geometric::ImplicitPRM::constructSolutionImplicitly(const Vertex &start, const Vertex &goal,
    const base::PlannerTerminationCondition &ptc) {
    if (prmType_ == Lattice) {
        return constructSolutionImplicitlyLattice(start, goal, ptc);
    }
    return constructSolutionImplicitlyRND(start, goal, ptc);
}

int ompl::geometric::ImplicitPRM::checkMemory() {
    sysinfo (&memInfo);

    long long totalVirtualMem = memInfo.totalram;
    //Add other values in next statement to avoid int overflow on right hand side...
    totalVirtualMem += memInfo.totalswap;
    totalVirtualMem *= memInfo.mem_unit;

    long long virtualMemUsed = memInfo.totalram - memInfo.freeram;
    //Add other values in next statement to avoid int overflow on right hand side...
    virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
    virtualMemUsed *= memInfo.mem_unit;
    int memUsage = (virtualMemUsed * 100.0) / totalVirtualMem;
    std::cout << "Memory used: " << (virtualMemUsed * 100.0) / totalVirtualMem << "%" << '\r' << std::flush;
    return memUsage;
}

ompl::base::PathPtr ompl::geometric::ImplicitPRM::constructSolutionImplicitlyLattice(const Vertex &start, const Vertex &goal,
                                                                                     const base::PlannerTerminationCondition &ptc) {
    std::unordered_map<Eigen::VectorXi, EigenAstar*, matrix_hashI<Eigen::VectorXi>, TVectorEqualsI> VISITED;
    std::unordered_map<Eigen::VectorXi, bool, matrix_hashI<Eigen::VectorXi>, TVectorEqualsI> INVALID;
    std::unordered_map<Eigen::VectorXd, bool, matrix_hash<Eigen::VectorXd>, TVectorEquals> INVALID_RND;
    std::unordered_map<EigenAstar*, EigenAstar*> PREV;
    std::vector<EigenAstar*> OPEN;
    int d_ = dimRd_ * robotCount_;
    auto compareEigens = TVectorEquals();
    // init the root node
    auto rootI = Eigen::VectorXi::Zero(d_);
    auto* startV = new EigenAstar{
        0, distEuclid(startEigen_, goalEigen_,d_), this->createNewState(startEigen_),
        1, startEigen_, rootI};
    auto tConstructionStart = std::chrono::high_resolution_clock::now();
    VISITED[rootI] = startV;
    PREV[startV] = nullptr;
    OPEN.push_back(startV);
    // heapify OPEN
    std::ranges::make_heap(OPEN, EigenGT());

    OMPL_INFORM("Using radius=%f", resData_[0].r);
    OMPL_INFORM("Calculating the regular NN ball.");
    // the neighbors set is similar no matter which point of the lattice you are on
    Eigen::VectorXd zeroV = Eigen::VectorXd::Zero(d_);
    for (int i = 0; i < resData_.size(); ++i) {
        this->setSamplesInBallNoBoundsRes(i, zeroV);
        std::cout << "Neighbor count = " << resData_[0].neighbors.size() << std::endl;
    }


    auto tConstructionEnd = std::chrono::high_resolution_clock::now();
    results_.constructionTime =
        duration_cast<std::chrono::microseconds>(tConstructionEnd - tConstructionStart).count();
    auto p(std::make_shared<PathGeometric>(si_));
    bool pathFound = false;
    OMPL_INFORM("Running A* on the implicit graph to find a solution");
    TVectorEqualsI eigenComp;

    // auto tCurr = std::chrono::high_resolution_clock::now();
    // auto tLast = tConstructionEnd;
    while (!OPEN.empty() && !ptc) {
        // Option to stop run according to mem usage
        // tCurr = std::chrono::high_resolution_clock::now();
        // if (duration_cast<std::chrono::seconds>(tCurr - tLast).count() > 2) {
        //     int memUsage = checkMemory();
        //     tLast = tCurr;
        //     if (memUsage > 95) {
        //         OMPL_INFORM("Terminating due to excess memory usage");
        //         break;
        //     }
        // }
        // get the min f-value vertex
        std::ranges::pop_heap(OPEN, EigenGT());
        EigenAstar* minV = OPEN.back();
        OPEN.pop_back();
        Eigen::VectorXd minVeigen = StateToEigen(minV->state);
        // have we reached the goal?
        if (compareEigens(minVeigen, goalEigen_)) {
            std::vector<const base::State*> resVec;
            results_.length = 0;
            results_.lengthSeparated = 0;
            while (PREV[minV] != nullptr) {
                resVec.push_back(minV->state);
                results_.length += distEuclid(StateToEigen(minV->state), StateToEigen(PREV[minV]->state), d_);
                results_.lengthSeparated += distSeparated(StateToEigen(minV->state), StateToEigen(PREV[minV]->state), d_);
                minV = PREV[minV];
            }
            resVec.push_back(minV->state);
            for (std::vector<const base::State *>::const_reverse_iterator st = resVec.rbegin(); st != resVec.rend(); ++st) {
                p->append(*st);
            }

            pathFound = true;
            break;
        }
        int i = 0;
        // is the goal near?
        bool checkGoal = false;
        if (distEuclid(minVeigen, goalEigen_, d_) < resData_[i].r) {
            checkGoal = true;
            resData_[i].neighbors.push_back({
                goalEigen_ - minVeigen, Eigen::VectorXi::Ones(d_) * INT_MAX,
                distEuclid(goalEigen_, minVeigen,d_), true});
        }
        for (auto neighbor: resData_[i].neighbors) {
            if (ptc) break;
            // lattices use a regular "ball" neighbor-structure. shift the ball to our sample.
            // the goal's IntVector is set to "max_int", as the goal doesnt have an IntVector.
            if (!neighbor.isGoal) neighbor.sampleI += minV->sampleI;
            neighbor.sample += minV->sample;
            // vertex/edge validity checking
            if (!neighbor.isGoal && INVALID.contains(neighbor.sampleI)
                && INVALID[neighbor.sampleI] == true) continue;
            auto neigbhorState = this->createNewState(neighbor.sample);
            if (!si_->isValid(neigbhorState)) {
                si_->freeState(neigbhorState);
                INVALID[neighbor.sampleI] = true;
                continue; // vertex in collision
            }
            bool isEdgeValid = si_->checkMotion(minV->state, neigbhorState);
            si_->freeState(neigbhorState);
            if (!isEdgeValid) continue; // edge in collision

            double newG = minV->g + neighbor.g;
            double currG = VISITED.contains(neighbor.sampleI) ? VISITED[neighbor.sampleI]->g : DBL_MAX;
            if (newG < currG) {
                EigenAstar* neighborV = nullptr;
                if (VISITED.contains(neighbor.sampleI)) {
                    neighborV = VISITED[neighbor.sampleI];
                } else {
                    neighborV = new EigenAstar{
                        newG, distEuclid(neighbor.sample, goalEigen_,d_),
                        this->createNewState(neighbor.sample), 0, // create new state is somehow not deleted
                        neighbor.sample, neighbor.sampleI
                    };
                    VISITED[neighbor.sampleI] = neighborV; // eigen to struct
                }
                PREV[neighborV] = minV; // new parent
                // is it in OPEN?
                bool inOpen = false;
                for (auto elem: OPEN) {
                    if (eigenComp(elem->sampleI, neighborV->sampleI)) {
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
        if (checkGoal) resData_[i].neighbors.pop_back();
    }
    std::cout << "size of visited=" << VISITED.size() << std::endl;
    auto tAstarEnd = std::chrono::high_resolution_clock::now();
    results_.astarTime =
        duration_cast<std::chrono::microseconds>(tAstarEnd - tConstructionEnd).count();
    std::cout << "final time = " << results_.astarTime / 1000000.0 << " seconds" << std::endl;
    if (!pathFound) {
        if (!ptc)
            OMPL_INFORM("Path NOT found: went through all the graph.");
        else
            OMPL_INFORM("Path NOT found: timeout.");
    }
    for (auto iter = VISITED.begin(); iter != VISITED.end(); ++iter) {
        freeEigenstarNode(iter->second);
    }
    if (pathFound) {
        OMPL_INFORM("Path found!");
        if (LatticeType_ == AnStar) {
            OMPL_INFORM("AnStar: counting total samples.");
            if (countSamplesForRND_) {
                // more then d>=6 takes too long to calc explicitly
                if (robotCount_ >= 6) {
                    double mapSide = mapExtent_.high[0] - mapExtent_.low[0];
                    // upper bound estimation
                    results_.samples = resData_[0].neighbors.size() * std::pow((mapSide * std::sqrt(2)) / (resData_[0].r), robotCount_);
                } else {
                    results_.samples = this->countSamplesInMapBounds(0, startEigen_);
                }
            }
            OMPL_INFORM("AnStar: counting total samples done, got %d samples.", results_.samples);
        }
        return p;
    }
    return nullptr;
}

ompl::base::PathPtr ompl::geometric::ImplicitPRM::constructSolutionImplicitlyRND(const Vertex &start, const Vertex &goal,
                                                                                 const base::PlannerTerminationCondition &ptc) {
    std::unordered_map<Vertex, EigenAstarRND*> VISITED;
    std::unordered_map<Vertex, bool> INVALID;
    std::unordered_map<EigenAstarRND*, EigenAstarRND*> PREV;
    std::vector<EigenAstarRND*> OPEN;
    int d_ = dimRd_ * robotCount_;
    auto compareEigens = TVectorEquals();
    // init the root node
    auto root = start;
    auto* startV = new EigenAstarRND{
        0, distEuclid(startEigen_, goalEigen_,d_), this->createNewState(startEigen_),
        1, startEigen_, start};
    auto tConstructionStart = std::chrono::high_resolution_clock::now();
    VISITED[root] = startV;
    PREV[startV] = nullptr;
    OPEN.push_back(startV);
    // heapify OPEN
    std::ranges::make_heap(OPEN, EigenGT_RND());
    OMPL_INFORM("Constructing the random points and creating a NN structure.");
    // if we are using a RND set, define the PRM* radius
    if (prmType_ == Random && !sameRadius_) { // remove the "false", this is temp
        double mapVolume = 1;
        for (int i = 0; i < d_; ++i) {
            mapVolume *= (mapExtent_.high[i % dimRd_] - mapExtent_.low[i % dimRd_]);
        }
        double unitBallVolume = (1.0 / std::sqrt(d_ * M_PI)) * std::pow((2*M_PI*M_E)/d_, d_ / 2.0);
        r_ = 2 * std::pow((1.0 + (1.0 / d_)) * (mapVolume / unitBallVolume) * (std::log(sampleLimit_)/sampleLimit_),(1.0/d_));
    }
    OMPL_INFORM("Using radius=%f", r_);
    // now add elements to the NN structure
    AddAllVerticesWithNN();
    std::cout << "added " << boost::num_vertices(g_) << " vertices." << std::endl;


    auto tConstructionEnd = std::chrono::high_resolution_clock::now();
    results_.constructionTime =
        duration_cast<std::chrono::microseconds>(tConstructionEnd - tConstructionStart).count();
    auto p(std::make_shared<PathGeometric>(si_));
    bool pathFound = false;
    OMPL_INFORM("Running A* on the implicit graph to find a solution");
    TVectorEqualsI eigenComp;
    auto tCurr = std::chrono::high_resolution_clock::now();
    auto tLast = tConstructionEnd;
    while (!OPEN.empty() && !ptc) {
        // Option to stop run according to mem usage
        // tCurr = std::chrono::high_resolution_clock::now();
        // if (duration_cast<std::chrono::seconds>(tCurr - tLast).count() > 2) {
        //     int memUsage = checkMemory();
        //     tLast = tCurr;
        //     if (memUsage > 95) {
        //         OMPL_INFORM("Terminating due to excess memory usage");
        //         break;
        //     }
        // }
        // get the min f-value vertex
        std::ranges::pop_heap(OPEN, EigenGT_RND());
        EigenAstarRND* minV = OPEN.back();
        OPEN.pop_back();
        Eigen::VectorXd minVeigen = StateToEigen(minV->state);
        // have we reached the goal?
        if (compareEigens(minVeigen, goalEigen_)) {
            std::vector<const base::State*> resVec;
            results_.length = 0;
            results_.lengthSeparated = 0;
            while (PREV[minV] != nullptr) {
                resVec.push_back(minV->state);
                results_.length += distEuclid(StateToEigen(minV->state), StateToEigen(PREV[minV]->state), d_);
                results_.lengthSeparated += distSeparated(StateToEigen(minV->state), StateToEigen(PREV[minV]->state), d_);
                minV = PREV[minV];
            }
            resVec.push_back(minV->state);
            for (std::vector<const base::State *>::const_reverse_iterator st = resVec.rbegin(); st != resVec.rend(); ++st) {
                p->append(*st);
            }

            pathFound = true;
            break;
        }
        std::vector<Vertex> neighborsV;
        nn_->nearestR(EigenVecToVertex_[StateToEigen(minV->state)], r_, neighborsV);
        for (const auto& neighborV: neighborsV) {
            auto neighborEigen = StateToEigen(stateProperty_[neighborV]);
            neighborsRND_.push_back({
                neighborEigen, neighborV,
                distEuclid(neighborEigen, minVeigen, d_), false
            });
        }

        // is the goal near?
        bool checkGoal = false;
        if (distEuclid(minVeigen, goalEigen_, d_) < r_) {
            checkGoal = true;
            neighborsRND_.push_back({
                goalEigen_, goal,
                distEuclid(goalEigen_, minVeigen,d_), true});
        }
        for (auto neighbor: neighborsRND_) {
            if (ptc) break;
            // vertex/edge validity checking
            if (!neighbor.isGoal && INVALID.contains(neighbor.sampleV)
                && INVALID[neighbor.sampleV] == true) continue;
            auto neigbhorState = this->createNewState(neighbor.sample);
            if (!si_->isValid(neigbhorState)) {
                si_->freeState(neigbhorState);
                INVALID[neighbor.sampleV] = true;
                continue; // vertex in collision
            }
            bool isEdgeValid = si_->checkMotion(minV->state, neigbhorState);
            si_->freeState(neigbhorState);
            if (!isEdgeValid) continue; // edge in collision

            double newG = minV->g + neighbor.g;
            double currG = VISITED.contains(neighbor.sampleV) ? VISITED[neighbor.sampleV]->g : DBL_MAX;
            if (newG < currG) {
                EigenAstarRND* neighborV = nullptr;
                if (VISITED.contains(neighbor.sampleV)) {
                    neighborV = VISITED[neighbor.sampleV];
                } else {
                    neighborV = new EigenAstarRND{
                        newG, distEuclid(neighbor.sample, goalEigen_,d_),
                        this->createNewState(neighbor.sample), 0,
                        neighbor.sample, neighbor.sampleV
                    };
                    VISITED[neighbor.sampleV] = neighborV; // eigen to struct
                }
                PREV[neighborV] = minV; // new parent
                // is it in OPEN?
                bool inOpen = false;
                for (auto elem: OPEN) {
                    if (elem->sampleV == neighborV->sampleV) {
                        inOpen = true;
                        break;
                    }
                }
                // if not, insert it
                if (!inOpen) {
                    // insert to heap and heapify
                    OPEN.push_back(neighborV); // insert to heap
                    std::push_heap(OPEN.begin(), OPEN.end(), EigenGT_RND()); // bubble element to its place
                }
            }
        }
        neighborsRND_.clear(); // remove the goal.
    }
    neighborsRND_.clear();
    std::cout << "size of visited=" << VISITED.size() << std::endl;
    auto tAstarEnd = std::chrono::high_resolution_clock::now();
    results_.astarTime =
        duration_cast<std::chrono::microseconds>(tAstarEnd - tConstructionEnd).count();
    std::cout << "final time: " << results_.constructionTime << " || " << results_.astarTime << std::endl;
    if (!pathFound) {
        if (!ptc)
            OMPL_INFORM("Path NOT found: went through all the graph.");
        else
            OMPL_INFORM("Path NOT found: timeout.");
    }
    for (auto iter = VISITED.begin(); iter != VISITED.end(); ++iter) {
        freeEigenstarRndNode(iter->second);
    }
    if (pathFound) {
        OMPL_INFORM("Path found!");
        return p;
    }
    return nullptr;
}

void ompl::geometric::ImplicitPRM::freeEigenstarNode(EigenAstar* node) {
    // if (sanityCheckMode_) si_->freeState(node->state); //in Random mode, all states are released already
    si_->freeState(node->state); //in Random mode, all states are released already
    delete node;
}

void ompl::geometric::ImplicitPRM::freeEigenstarRndNode(EigenAstarRND* node) {
    // if (!sanityCheckMode_) si_->freeState(node->state); //in Random mode, all states are released already
    si_->freeState(node->state); //in Random mode, all states are released already
    delete node;
}

void ompl::geometric::ImplicitPRM::goOverSamples(Eigen::VectorXd& root) {
    int d_ = dimRd_ * robotCount_;
    std::vector<LatticeNN> open;
    std::vector<LatticeNN> openTemp;
    std::unordered_map<Eigen::VectorXi, int, matrix_hashI<Eigen::VectorXi>, TVectorEqualsI> visited;
    long samples = 1;
    int power = 2;
    // add root
    Eigen::VectorXi currI = Eigen::VectorXi::Zero(d_);
    visited[currI] = 0;
    open.push_back({root, currI, 0});
    int id = 0;

    const auto& T = resData_[id].T;
    const auto r = resData_[id].r;

    std::cout << T << std::endl;

    while (!open.empty() || !openTemp.empty()) {
        if (open.empty()) {
            // we have the next wave of neighbors
            open = openTemp;
            openTemp.clear();
        }
        const auto [sample, sampleI, g, isGoal] = open[open.size() - 1];
        open.pop_back();
        int sign = -1;
        for (int j = 0; j < 2; ++j) {
            sign = -sign;
            for (int i = 0; i < d_; ++i) {
                Eigen::VectorXd newNeighbor = sample + sign * T.col(i);
                Eigen::VectorXi newNeighborI = sampleI;
                newNeighborI[i] += sign;
                if (visited.contains(newNeighborI) && visited[newNeighborI] < 2*d_) {
                    visited[newNeighborI]++;
                    if (visited[newNeighborI] == 2 * d_)
                        visited.erase(newNeighborI);
                } else {
                    visited[newNeighborI] = 1;

                    bool inArea = true;
                    // is it in the rectangle
                    for (int w = 0; w < d_; ++w) {
                        inArea = inArea && (
                            (newNeighbor[w] > mapExtent_.low[w % dimRd_] - maxEdge_
                                || compareDoubles(newNeighbor[w], mapExtent_.low[w  % dimRd_] - maxEdge_)) &&
                            (newNeighbor[w] < mapExtent_.high[w % dimRd_] + maxEdge_
                                || compareDoubles(newNeighbor[w], mapExtent_.high[w  % dimRd_] + maxEdge_)));
                    }

                    if (inArea) {
                        Vertex neighborV = addMilestone(nullptr, newNeighbor);
                        // we need to explore all space, regardless of collisions.
                        // some samples may be "isolated" in terms of direct connection to another
                        // sample, but in a PRM r-ball they may end up connecting.
                        // neighborsRND_.push_back({newNeighbor, neighborV,
                        // distEuclid(newNeighbor, root, d_)});
                        // the regular ball-search is unscaled
                        openTemp.push_back(
                            {newNeighbor, newNeighborI,
                                distEuclid(newNeighbor, root, d_)});
                    }
                }
            }
        }
    }
}

#pragma endregion the algorithm
