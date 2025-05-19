//
// Created by itai on 11/5/24.
//

#pragma once

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <utility>
#include <vector>
#include <map>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <float.h>

#include "sys/types.h"
#include "sys/sysinfo.h"

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
           @anchor gImplicitPRMVamp
           @par Short description
           ImplicitPRMVamp is a planner that runs a standard lazy A* run on an implicit PRM graph:
           this is a graph in which we don't explicitly build the entire graph, but in which
           we do know at every node who the node's neighbors are.
           This algorithm has two modes of operation.
           (1) In RANDOM mode, we uniformly sample ahead of time a set number of sample points,
           constructing a NN structure ahead of time to use as a way of informing the
           algorithm's of a sample's neighbors.
           (2) In LATTICE mode, we start off by defining a lattice generator for a lattice in
           {Zn, Dn*, An*}. After that, we use the regularity of the lattice as a way of informing
           the algorithm's of a sample's neighbors.
           ### This version works on manipulators. For now, until the versions are united, this is separate.
           @par External documentation
           I. Panasoff and K. Solovey
           Effective Sampling for Robot Motion Planning Through the Lens of Lattices,
           Accepted to RSS2025. ArXiv link:
           https://arxiv.org/pdf/2502.04908
        */

        /** \brief Lazy Probabilistic RoadMap planner */
        class ImplicitPRMVamp : public base::Planner
        {
        public:
            enum LatticeType {
                Zn,
                DnStar,
                AnStar,
                AnStarReduced,
                Lc1,
                Lc1b,
                Lc2,
                File,
                Unknown
            };
            enum PrmType {
                Lattice,
                LatticeWithNN,
                Random
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
                double length;
                double lengthSeparated;
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
            explicit ImplicitPRMVamp(const base::SpaceInformationPtr &si, bool starStrategy = false);

            /** \brief Constructor */
            explicit ImplicitPRMVamp(const base::PlannerData &data, bool starStrategy = false);

            ~ImplicitPRMVamp() override;

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

            void CreateLatticeParameters();

            static double unitBallVol(int d) {
                switch (d) {
                    default:
                        return 1;
                    case 2:
                        return M_PI;
                    case 3:
                        return (4 * M_PI) / 3;
                    case 4:
                        return pow(M_PI, 2) / 2;
                    case 5:
                        return (8 * pow(M_PI, 2)) / 15;
                    case 6:
                        return pow(M_PI, 3) / 6;
                    case 7:
                        return (16 * pow(M_PI, 3)) / 105;
                    case 8:
                        return pow(M_PI, 4) / 24;
                    case 9:
                        return (32 * pow(M_PI, 4)) / 945;
                    case 10:
                        return pow(M_PI, 5) / 120;
                    case 11:
                        return (64 * pow(M_PI, 5)) / 10395;
                    case 12:
                        return pow(M_PI, 6) / 720;
                }
            }

            /** \brief  return the mem-usage percentage*/
            int checkMemory();

            int getStateCount() const {
                return boost::num_vertices(g_);
            }

            /** \brief set the start/goal eigen vectors, for the purpose of adding them to the tree explicitly  */
            void setStartAndGoalEigen(const Eigen::VectorXd& startEigen, const Eigen::VectorXd& goalEigen);

            static void changeAngleToQuaternion(double u1, double u2, double u3, base::SE3StateSpace::StateType* state);
            // void setBounds(const base::RealVectorBounds& mapExtent) {
                // mapExtent_ = base::RealVectorBounds(mapExtent);
            // }
            void setAngleFix(const double& fix) {
                angleFix_ = fix;
            }
            void setDim(int n) {
                d_ = n;
            }
            void setVolume(double vol) {
                volume_ = vol;
            }
            void setCountSamplesForRND(bool val) {
                countSamplesForRND_ = val;
            }
            void setSampleLimit(long sampleLimit) {
                sampleLimit_ = sampleLimit;
            }
            void setNNRadius(double r) {
                // resData_.push_back(ResParams());
                // resData_[0].r = r;
                r_ = r;
            }

            void setSameRadius(bool val) {
                sameRadius_ = val;
            }

            void setPrmType(PrmType type) {
                prmType_ = type;
            }

            RunResults getResults() const {
                return results_;
            }

            double getRadius() const {
                return resData_[0].r;
            }

            long getNumVertices() {
                return boost::num_vertices(g_);
            }
            Eigen::VectorXd StateToEigen(const base::State* state) const;
            Eigen::VectorXd getValidStartState();

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
            struct LatticeNN {
                Eigen::VectorXd sample;
                Eigen::VectorXi sampleI;
                double g;
                bool isGoal = false;
            };
            struct LatticeNN_RND {
                Eigen::VectorXd sample;
                Vertex sampleV;
                double g;
                bool isGoal = false;
            };

            struct EigenAstar {
                double g;
                double h;
                ompl::base::State* state;
                int visited;
                Eigen::VectorXd sample;
                Eigen::VectorXi sampleI;
                // LatticeNN* data;
                double collisionPercent;
            };
            struct EigenAstarRND {
                double g;
                double h;
                ompl::base::State* state;
                int visited;
                Eigen::VectorXd sample;
                Vertex sampleV;
                // LatticeNN* data;
            };

            struct EigenGT {
                bool operator()(const EigenAstar* v1, const EigenAstar* v2) {
                    return v1->g + v1->h > v2->g + v2->h;
                }
            };
            struct EigenGT_RND {
                bool operator()(const EigenAstarRND* v1, const EigenAstarRND* v2) {
                    return v1->g + v1->h > v2->g + v2->h;
                }
            };

            class TVectorEquals {
            public:
                bool operator()(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2) const {
                    bool res = true;
                    for (int i = 0; i < v1.size(); ++i) {
                        res = res && std::fabs(v1[i] - v2[i]) < 0.01;
                    }
                    return res;
                }
            };

            class TVectorEqualsI {
            public:
                bool operator()(const Eigen::VectorXi& v1, const Eigen::VectorXi& v2) const {
                    for (int i = 0; i < v1.size(); ++i) {
                        if (v1[i] != v2[i]) return false;
                    }
                    return true;
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

            // Hash function for Eigen matrix and vector.
            // The code is from `hash_combine` function of the Boost library. See
            // http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
            template<typename T>
            struct matrix_hashI : std::unary_function<T, size_t> { // moo!!
                std::size_t operator()(T const& matrix) const {
                    // Note that it is oblivious to the storage order of Eigen matrix (column- or
                    // row-major). It will give you the same hash value for two different matrices if they
                    // are the transpose of each other in different storage order.
                    size_t seed = 0;
                    for (size_t i = 0; i < matrix.size(); ++i) {
                        auto elem = *(matrix.data() + i);
                        // auto roundElem = roundf(*(matrix.data() + i) * 10000) / 10000;
                        // int roundElem = round(*(matrix.data() + i) * 10000);
                        // int elem = (int)(10000 * roundElem);
                        // std::cout << "elem:" << elem << ",";
                        seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                    }
                    // std::cout << ",  ~~~seed=" << seed << std::endl;
                    return seed;
                }
            };

            const char* EnumToString(const PrmType& type) {
                if (type == Lattice) {
                    return "Lattice";
                } else if (type == LatticeWithNN) {
                    return "LatticeWithNN";
                } else if (type == Random) {
                    return "Random";
                }
            }

            static double distEuclid(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2, int d_) {
                double res = 0;
                for (int w = 0; w < d_; ++w) {
                    res += std::pow(v1[w] - v2[w], 2);
                }
                return std::sqrt(res);
            }

            // double distSeparated(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2, int d_) const {
            //     double res = 0;
            //     for (int w = 0; w < robotCount_; ++w) {
            //         double resTmp = 0.0;
            //         for (int i = 0; i < dimRd_; ++i) {
            //             resTmp += std::pow(v1[2*w + i] - v2[2*w + i], 2);
            //         }
            //         res += std::sqrt(resTmp);
            //     }
            //     return res;
            // }

            static bool compareDoubles(double a, double b) {
                return fabs(a - b) < DBL_EPSILON;
            }

            static double roundDbl(double d) {
                return (int)(d * 10000) / 10000.0;
            }

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

            void AddAllVerticesWithNN(); // type==RANDOM
            std::vector<Eigen::VectorXd> returnSamplesInBallNoBounds(Eigen::VectorXd& root);
            std::vector<LatticeNN> returnSamplesInBallNoBoundsI(Eigen::VectorXd& root);
            void setSamplesInBallNoBoundsRes(int id, Eigen::VectorXd& root);
            long countSamplesInMapBounds(int id, Eigen::VectorXd& root);
            void goOverSamples(Eigen::VectorXd& root);

            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data
               structure
                and then connect it to the roadmap in accordance to the connection strategy. */
            Vertex addMilestone(base::State *state, Eigen::VectorXd& newV);

            void uniteComponents(Vertex a, Vertex b);

            void markComponent(Vertex v, unsigned long int newComponent);

            /** \brief Check if any pair of a start state and goal state are part of the same connected component.
                If so, return the id of that component. Otherwise, return -1. */
            long int solutionComponent(std::pair<std::size_t, std::size_t> *startGoalPair) const;

            /** \brief Compute distance between two milestones (this is simply distance between the states of the
             * milestones) */
            double euclidDistanceFunction(const Vertex a, const Vertex b) const
            {
                const auto* aState = stateProperty_[a];
                const auto* bState = stateProperty_[b];
                double dist = 0.0;
                for (int i = 0; i < d_; ++i) {
                    double v1 = aState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
                    double v2 = bState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
                    dist += std::pow(v1 - v2, 2);
                }
                return std::sqrt(dist);
            }

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
            base::StateSamplerPtr sampler_;

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
            bool countSamplesForRND_;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set
 * it as the solution */
            ompl::base::PathPtr constructSolutionImplicitly(const Vertex &start, const Vertex &goal,
                const base::PlannerTerminationCondition &ptc);
            ompl::base::PathPtr constructSolutionImplicitlyLattice(const Vertex &start, const Vertex &goal,
    const base::PlannerTerminationCondition &ptc);
            ompl::base::PathPtr constructSolutionImplicitlyLattice2(const Vertex &start, const Vertex &goal,
const base::PlannerTerminationCondition &ptc);
            ompl::base::PathPtr constructSolutionImplicitlyLatticeRessed(const Vertex &start, const Vertex &goal,
const base::PlannerTerminationCondition &ptc);
            ompl::base::PathPtr constructSolutionImplicitlyRND(const Vertex &start, const Vertex &goal,
    const base::PlannerTerminationCondition &ptc);
            // ompl::base::PathPtr constructSolutionImplicitlyRes(const Vertex &start, const Vertex &goal,
    // const base::PlannerTerminationCondition &ptc);
            struct ResParams {
                double delta;
                double r;
                Eigen::MatrixXd T;
                std::vector<LatticeNN> neighbors;
            };
            long countSamplesInMap(Eigen::VectorXd& root);
            ompl::base::State* createNewState(Eigen::VectorXd& newV);
            void freeEigenstarNode(EigenAstar* node);
            void freeEigenstarRndNode(EigenAstarRND* node);

            std::unordered_map<Eigen::VectorXd, Vertex, matrix_hash<Eigen::VectorXd>, TVectorEquals> EigenVecToVertex_;

            std::unordered_map<std::string, Vertex> EigenVecToVertexOld_;
            std::unordered_map<Vertex, Vertex> edges_;
            std::unordered_map<std::string, Eigen::VectorXd> strToEigen_;

            // lattice params
            Eigen::MatrixXd T_; // lattice transformation
            double r_; // lattice sample radius
            double delta_;
            double epsilon_;
            LatticeType LatticeType_;

            // problem params
            base::RealVectorBounds mapExtent_;
            double angleFix_; // angle fix
            Eigen::VectorXd startEigen_;
            Eigen::VectorXd goalEigen_;
            base::State *workState_;
            // the type of space we deal with (currently support 3=>SE2, 6=>SE3)
            int d_;
            double volume_; // maximum length of a lattice base vector
            double maxEdge_; // maximum length of a lattice base vector
            RunResults results_;
            PrmType prmType_;
            std::vector<Eigen::VectorXd> neighbors_; // used by LATTICE type IPRM
            std::vector<LatticeNN_RND> neighborsRND_; // used by LATTICE type IPRM
            // std::vector<std::vector<Eigen::VectorXd>> neighborsRes_; // used by LATTICE type IPRM
            std::vector<ResParams> resData_;
            long sampleLimit_;
            bool sameRadius_;
            struct sysinfo memInfo;
        };
    }
}
