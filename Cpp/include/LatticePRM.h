/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ioan Sucan, Henning Kayser */

#pragma once

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <utility>
#include <vector>
#include <map>
#include <ompl/base/spaces/SE3StateSpace.h>

// struct int4
// {
//     double x1, y1, x2, y2;
//     double g;
//     double h;
//     int color;
//     int4* prev;
//
//     bool operator==(int4 const& other) const
//     {
//         return x1 == other.x1 && y1 == other.y1 && x2 == other.x2 && y2 == other.y2;
//     }
// };
//
// struct int4GT {
//     bool operator()(const int4& i1, const int4& i2) {
//         return i1.g + i1.h > i2.g + i2.h;
//     }
// };
//

struct intHash
{
     std::size_t operator()(const Eigen::VectorXi& v) const
     {
         std::size_t seed = 0;
         for (int i = 0; i < v.size(); ++i) {
             boost::hash_combine(seed, v[i]);
         }

         return seed;
     }
};
//
// double int4_dist(const int4& i1, const int4& i2) {
//     double res = 0;
//     res += std::pow(i1.x1 - i2.x1, 2);
//     res += std::pow(i1.y1 - i2.y1, 2);
//     res += std::pow(i1.x2 - i2.x2, 2);
//     res += std::pow(i1.y2 - i2.y2, 2);
//     return std::sqrt(res);
// }
//
// Eigen::VectorXd int4ToEigen(const int4& i) {
//     Eigen::VectorXd res(4);
//     res << i.x1, i.y1, i.x2, i.y2;
//     return res;
// }

namespace ompl
{
    namespace base
    {
        // Forward declare for use in implementation
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }

    namespace geometric
    {
        /**
           @anchor gLazyPRM
           @par Short description
           LazyPRM is a planner that constructs a roadmap of milestones
           that approximate the connectivity of the state space, just like PRM does.
           The difference is that the planner uses lazy collision checking.
           @par External documentation
           R. Bohlin and L.E. Kavraki
           Path Planning Using Lazy PRM
           <em>IEEE International Conference on Robotics and Automation</em>, San Francisco, pp. 521–528, 2000.
           DOI: [10.1109/ROBOT.2000.844107](http://dx.doi.org/10.1109/ROBOT.2000.844107)<br>
           [[more]](http://www.kavrakilab.org/robotics/lazyprm.html)
        */

        /** \brief Lazy Probabilistic RoadMap planner */
        class LatticePRM : public base::Planner
        {
        public:
            enum LatticeType {
                Zn,
                DnStar,
                AnStar,
                File
            };

            struct vertex_state_t
            {
                using kind = boost::vertex_property_tag;
            };

            struct vertex_flags_t
            {
                using kind = boost::vertex_property_tag;
            };

            struct vertex_component_t
            {
                using kind = boost::vertex_property_tag;
            };

            struct edge_flags_t
            {
                using kind = boost::edge_property_tag;
            };

            struct RunResults {
                long samples;
                long edges;
                long constructionTime;
                long astarTime;
            };


            /** @brief The type for a vertex in the roadmap. */
            using Vertex =
                boost::adjacency_list_traits<boost::vecS, boost::listS, boost::undirectedS>::vertex_descriptor;

            /**
             @brief The underlying roadmap graph.

             @par Any BGL graph representation could be used here. Because we
             expect the roadmap to be sparse (m<n^2), an adjacency_list is more
             appropriate than an adjacency_matrix. We use listS for the vertex list
             because vertex descriptors are invalidated by remove operations if using vecS.

             @par Obviously, a ompl::base::State* vertex property is required.
             The incremental connected components algorithm requires
             vertex_predecessor_t and vertex_rank_t properties.
             If boost::vecS is not used for vertex storage, then there must also
             be a boost:vertex_index_t property manually added.

             @par Edges should be undirected and have a weight property.
             */
            using Graph = boost::adjacency_list<
                boost::vecS, boost::listS, boost::undirectedS,
                boost::property<
                    vertex_state_t, base::State *,
                    boost::property<
                        boost::vertex_index_t, unsigned long int,
                        boost::property<vertex_flags_t, unsigned int,
                                        boost::property<vertex_component_t, unsigned long int,
                                                        boost::property<boost::vertex_predecessor_t, Vertex,
                                                                        boost::property<boost::vertex_rank_t,
                                                                                        unsigned long int>>>>>>,
                boost::property<boost::edge_weight_t, base::Cost, boost::property<edge_flags_t, unsigned int>>>;

            /** @brief The type for an edge in the roadmap. */
            using Edge = boost::graph_traits<Graph>::edge_descriptor;

            /** @brief A nearest neighbors data structure for roadmap vertices. */
            using RoadmapNeighbors = std::shared_ptr<NearestNeighbors<Vertex> >;

            /** @brief A function returning the milestones that should be
             * attempted to connect to. */
            using ConnectionStrategy = std::function<const std::vector<Vertex> &(const Vertex)>;

            /** @brief A function that can reject connections.

             This is called after previous connections from the neighbor list
             have been added to the roadmap.
             */
            using ConnectionFilter = std::function<bool (const Vertex &, const Vertex &)>;

            /** \brief Constructor */
            LatticePRM(const base::SpaceInformationPtr &si, bool starStrategy = false);

            /** \brief Constructor */
            LatticePRM(const base::PlannerData &data, bool starStrategy = false);

            ~LatticePRM() override;

            /** \brief Set the maximum length of a motion to be added to the roadmap. */
            void setRange(double distance);

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() == 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Vertex>>();
                if (!userSetConnectionStrategy_)
                    setDefaultConnectionStrategy();
                if (isSetup())
                    setup();
            }

            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

            /** \brief Set the connection strategy function that specifies the
             milestones that connection attempts will be make to for a
             given milestone.

             \par The behavior and performance of PRM can be changed drastically
             by varying the number and properties if the milestones that are
             connected to each other.

             \param pdef A function that takes a milestone as an argument and
             returns a collection of other milestones to which a connection
             attempt must be made. The default connection strategy is to connect
             a milestone's 10 closest neighbors.
             */
            void setConnectionStrategy(const ConnectionStrategy &connectionStrategy)
            {
                connectionStrategy_ = connectionStrategy;
                userSetConnectionStrategy_ = true;
            }
            /** Set default strategy for connecting to nearest neighbors */
            void setDefaultConnectionStrategy();

            /** \brief Convenience function that sets the connection strategy to the
             default one with k nearest neighbors.
             */
            void setMaxNearestNeighbors(unsigned int k);

            /** \brief Set the function that can reject a milestone connection.

             \par The given function is called immediately before a connection
             is checked for collision and added to the roadmap. Other neighbors
             may have already been connected before this function is called.
             This allows certain heuristics that use the structure of the
             roadmap (like connected components or useful cycles) to be
             implemented by changing this function.

             \param connectionFilter A function that takes the new milestone,
             a neighboring milestone and returns whether a connection should be
             attempted.
             */
            void setConnectionFilter(const ConnectionFilter &connectionFilter)
            {
                connectionFilter_ = connectionFilter;
            }

            /** \brief set the lattice params, including the type and the epsilon/delta  */
            void setLatticeType(LatticeType type, double delta, double epsilon);

            int getStateCount() const {
                return boost::num_vertices(g_);
            }

            /** \brief set the start/goal eigen vectors, for the purpose of adding them to the tree explicitly  */
            void setStartAndGoalEigen(const Eigen::VectorXd& startEigen, const Eigen::VectorXd& goalEigen);
            void changeAngleToQuaternion(double u1, double u2, double u3, base::SE3StateSpace::StateType* state);
            void setBounds(const base::RealVectorBounds& mapExtent) {
                mapExtent_ = base::RealVectorBounds(mapExtent);
            }
            void setAngleFix(const double& fix) {
                angleFix_ = fix;
            }
            void setRobotCount(int n) {
                robotCount_ = n;
            }
            void setbuildAllGraph(bool val) {
                buildAllGraph_ = val;
            }

            RunResults getResults() const {
                return results_;
            }

            double getRadius() const {
                return r_;
            }

            long getRunTime() {
                return runtime_;
            }

            long getNumVertices() {
                return boost::num_vertices(g_);
            }
            Eigen::VectorXd StateToEigen(const base::State* state) const;

            bool LazyAstar();
            // void createMilestone(base::State * workState, const Eigen::VectorXd& newV);

            /** \brief Return the number of milestones currently in the graph */
            unsigned long int milestoneCount() const
            {
                return boost::num_vertices(g_);
            }

            /** \brief Return the number of edges currently in the graph */
            unsigned long int edgeCount() const
            {
                return boost::num_edges(g_);
            }

            void getPlannerData(base::PlannerData &data) const override;
            signed long getTotalRuntime () const {
                return runtime_;
            }
            signed long getAstarRuntime () const {
                return runtimeAstar_;
            }

            void setup() override;

            void clear() override;

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for LazyPRM. */
            void clearQuery() override;

            /** \brief change the validity flag of each node and edge to VALIDITY_UNKNOWN */
            void clearValidity();

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        protected:
            class TVectorEquals {
            public:
                bool operator()(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2) const {
                    bool res = true;
                    for (int i = 0; i < v1.size(); ++i) {
                        res = res && std::fabs(v1[i] - v2[i]) < 0.0001;
                    }
                    return res;
                }
            };

            // Hash function for Eigen matrix and vector.
            // The code is from `hash_combine` function of the Boost library. See
            // http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
            template<typename T>
            struct matrix_hash : std::unary_function<T, size_t> { // moo!!
                std::size_t operator()(T const& matrix) const {
                    // Note that it is oblivious to the storage order of Eigen matrix (column- or
                    // row-major). It will give you the same hash value for two different matrices if they
                    // are the transpose of each other in different storage order.
                    size_t seed = 0;
                    for (size_t i = 0; i < matrix.size(); ++i) {
                        // auto elem = *(matrix.data() + i);
                        // auto roundElem = roundf(*(matrix.data() + i) * 10000) / 10000;
                        int roundElem = round(*(matrix.data() + i) * 10000);
                        // int elem = (int)(10000 * roundElem);
                        // std::cout << "elem:" << elem << ",";
                        seed ^= std::hash<typename T::Scalar>()(roundElem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                    }
                    // std::cout << ",  ~~~seed=" << seed << std::endl;
                    return seed;
                }
            };

            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_UNKNOWN = 0;

            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_TRUE = 1;

            const std::vector<Vertex>&  latticeNN(const Vertex);
            ///////////////////////////////////////
            // Planner progress property functions
            std::string getIterationCount() const
            {
                return std::to_string(iterations_);
            }
            std::string getBestCost() const
            {
                return std::to_string(bestCost_.value());
            }
            std::string getMilestoneCountString() const
            {
                return std::to_string(milestoneCount());
            }
            std::string getEdgeCountString() const
            {
                return std::to_string(edgeCount());
            }

            /** Go over samples around a "root" point:
             * in a cube: type == 1 w/ adding vertices.
             * in a ball: type == 0 w/ connecting edges.  */
            void addSamplesInMapRegion(const Eigen::VectorXd& root);
            // void goOverSamples2(int type, Eigen::VectorXi& root);

            double distEquclid2(const base::State *s1, const base::State *s2, int d_);

            std::vector<Eigen::VectorXd> returnSamplesInBall(Eigen::VectorXd& root);
            std::vector<Eigen::VectorXd> returnSamplesInBallNoBounds(Eigen::VectorXd& root);
            // std::vector<Eigen::VectorXi> returnSamplesInBall(const int4& root, const int4& start, const int4& goal);

            // void AddAllVertices2();
            // void AddAllVerticesRnd(const base::PlannerTerminationCondition &ptc);

            long AddAllEdges();

            // void AddAllEdges2();
            bool connectVertices(const Eigen::VectorXd& root, const Eigen::VectorXd& neighbor);

            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data
               structure
                and then connect it to the roadmap in accordance to the connection strategy. */
            Vertex addMilestone(base::State *state, Eigen::VectorXd& newV);
            // Vertex addMilestone2(base::State *state, Eigen::VectorXd& newV, bool start = false, bool goal = false);
            base::State* createNewState(Eigen::VectorXd& newV);
            // base::State* createNewState(const int4& i);

            void uniteComponents(Vertex a, Vertex b);

            void markComponent(Vertex v, unsigned long int newComponent);

            /** \brief Check if any pair of a start state and goal state are part of the same connected component.
                If so, return the id of that component. Otherwise, return -1. */
            long int solutionComponent(std::pair<std::size_t, std::size_t> *startGoalPair) const;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set
             * it as the solution */
            ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);
            ompl::base::PathPtr constructSolutionTest(const Vertex &start, const Vertex &goal);
            ompl::base::PathPtr constructSolutionTest2(const Vertex &start, const Vertex &goal);
            // ompl::base::PathPtr constructSolutionInts(const Vertex &start, const Vertex &goal);

            /** \brief Compute distance between two milestones (this is simply distance between the states of the
             * milestones) */
            double distanceFunction(const Vertex a, const Vertex b) const
            {
                return si_->distance(stateProperty_[a], stateProperty_[b]);
            }

            /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them.
                This method wraps OptimizationObjective::motionCostHeuristic */
            base::Cost costHeuristic(Vertex u, Vertex v) const;

            /** \brief Flag indicating whether the default connection strategy is the Star strategy */
            bool starStrategy_;

            /** \brief Function that returns the milestones to attempt connections with */
            ConnectionStrategy connectionStrategy_;

            /** \brief Function that can reject a milestone connection */
            ConnectionFilter connectionFilter_;

            /** \brief Flag indicating whether the employed connection strategy was set by the user (or defaults are
             * assumed) */
            bool userSetConnectionStrategy_{false};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            // /** \brief Sampler user for generating random in the state space */
            // base::StateSamplerPtr sampler_;

            /** \brief Nearest neighbors data structure */
            RoadmapNeighbors nn_;

            /** \brief Connectivity graph */
            Graph g_;

            /** \brief Array of start milestones */
            std::vector<Vertex> startM_;

            /** \brief Array of goal milestones */
            std::vector<Vertex> goalM_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, boost::vertex_index_t>::type indexProperty_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, vertex_state_t>::type stateProperty_;

            /** \brief Access to the weights of each Edge */
            boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

            /** \brief Access the connected component of a vertex */
            boost::property_map<Graph, vertex_component_t>::type vertexComponentProperty_;

            /** \brief Access the validity state of a vertex */
            boost::property_map<Graph, vertex_flags_t>::type vertexValidityProperty_;

            /** \brief Access the validity state of an edge */
            boost::property_map<Graph, edge_flags_t>::type edgeValidityProperty_;

            /** \brief Number of connected components created so far. This is used as an ID only,
                does not represent the actual number of components currently in the graph. */
            unsigned long int componentCount_{0};

            /** \brief The number of elements in each component in the LazyPRM roadmap. */
            std::map<unsigned long int, unsigned long int> componentSize_;

            /** \brief Objective cost function for PRM graph edges */
            base::OptimizationObjectivePtr opt_;

            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

            unsigned long int iterations_{0};

            // a mapping from the int-vector space to the Vertex itself
            // std::unordered_map<Eigen::VectorXi, Vertex, matrix_hash<Eigen::VectorXi>> EigenVecToVertex_2;
            // std::unordered_map<Eigen::VectorXi, Vertex, intHash> EigenVecToVertex_2;
            std::unordered_map<Eigen::VectorXd, Vertex, matrix_hash<Eigen::VectorXd>, TVectorEquals> EigenVecToVertex_;

            std::unordered_map<std::string, Vertex> EigenVecToVertexOld_;
            std::unordered_map<Vertex, Vertex> edges_;
            std::unordered_map<std::string, Eigen::VectorXd> strToEigen_;

            long goOverSamples(int type, Eigen::VectorXd& root);
            long countSamplesInMap(Eigen::VectorXd& root);
            void AddAllVerticesOld();
            long AddAllEdgesOld();

            // std::unordered_map<Vertex, std::vector<int>> VertexToIntVec_;
            Eigen::MatrixXd T_; // lattice transformation
            double r_; // lattice sample radius
            base::RealVectorBounds mapExtent_;
            int dimRd_; // dimension of the Rd part of the states (2,3)
            double angleFix_; // angle fix
            base::State *workState_;
            Eigen::VectorXd startEigen_;
            Eigen::VectorXd goalEigen_;
            // the type of space we deal with (currently support 3=>SE2, 6=>SE3)
            LatticeType LatticeType_;
            int robotCount_;
            signed long int runtime_;
            signed long int runtimeAstar_;
            bool buildAllGraph_; // full graph A* (true) or lazy (false)?
            double maxEdge_; // maximum length of a lattice base vector
            RunResults results_;
        };
    }
}
