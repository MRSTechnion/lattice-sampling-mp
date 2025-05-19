//
// Created by itai on 5/15/25.
//

#include "../include/VAMPTests.h"

//
// Created by itai on 5/15/25.
//

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>

#include <vamp/robots/panda.hh>
#include <vamp/robots/ur5.hh>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>

// planners
#include <ompl/geometric/planners/informedtrees/BITstar.h>
// #include <ompl/geometric/planners/prm/PRM.h>
#include <fmt/format.h>
#include <ompl/util/Exception.h>

#include "../include/ImplicitPRM_VAMP.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
// using Robot = vamp::robots::UR5;

static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;

static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

// Start and goal configurations
// static constexpr std::array<float, dimension> start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
// // static constexpr std::array<float, dimension> goal = {-2.37, 0.69, -1.08, -1.25 , 1.18, 2.93, -2.79}; // true goal
// static constexpr std::array<float, dimension> goal = {-2.37, 0.69, -1.08, -1.25 , 1.18, 2, -2.79}; // true goal
// b = [-2.37, 0.69, -1.08, -1.25 , 1.18, 2, -2.79]
// duck start/goal
// static constexpr std::array<float, dimension> start = {0., 1.571, 0., 0, 0., 1.571, 0.785};
// static constexpr std::array<float, dimension> goal = {0., -1.571, 0., 0, 0., 1.571, 0.785}; // true goal

static const std::vector<std::array<float, 3>> empty_problem = {

};

// // Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
// static const std::vector<std::array<float, 3>> problem = {
//     {0.55, 0, 0.25},
//     {0.35, 0.35, 0.25},
//     {0, 0.55, 0.25},
//     {-0.55, 0, 0.25},
//     {-0.35, -0.35, 0.25},
//     {0, -0.55, 0.25},
//     {0.35, -0.35, 0.25},
//     {0.35, 0.35, 0.8},
//     {0, 0.55, 0.8},
//     {-0.35, 0.35, 0.8},
//     {-0.55, 0, 0.8},
//     {-0.35, -0.35, 0.8},
//     {0, -0.55, 0.8},
//     {0.35, -0.35, 0.8},
// };
//
// static const std::vector<std::array<float, 3>> problem2_cubes = {
//  {0, 0.3, 0.75},
// };
//
// static const std::vector<std::array<float, 3>> problem2 = {  //d=2.65, samples=1500
//     // window
//     {-0.3, 0, 0.3},
//     {-0.3, 0.15, 0.3},
//     {-0.3, -0.15, 0.3},
//     {-0.3, 0, 0.9},
//     {-0.3, 0.15, 0.9},
//     {-0.3, -0.15, 0.9},
//
//     {-0.3, -0.3, 0.3},
//     {-0.3, -0.3, 0.45},
//     {-0.3, -0.3, 0.6},
//     {-0.3, -0.3, 0.75},
//     {-0.3, -0.3, 0.9},
//
//     {-0.3, 0.3, 0.3},
//     {-0.3, 0.3, 0.45},
//     {-0.3, 0.3, 0.6},
//     {-0.3, 0.3, 0.75},
//     {-0.3, 0.3, 0.9},
// };

// static const std::vector<std::array<float, 3>> sleeve = {  //d=2.65, samples=1500
//     // window
//     {-0.3, 0, 0.3},
//     {-0.3, 0.15, 0.3},
//     {-0.3, -0.15, 0.3},
//     {-0.3, 0, 0.9},
//     {-0.3, 0.15, 0.9},
//     {-0.3, -0.15, 0.9},
//
//     {-0.3, -0.3, 0.3},
//     {-0.3, -0.3, 0.45},
//     {-0.3, -0.3, 0.6},
//     {-0.3, -0.3, 0.75},
//     {-0.3, -0.3, 0.9},
//
//     {-0.3, 0.3, 0.3},
//     {-0.3, 0.3, 0.45},
//     {-0.3, 0.3, 0.6},
//     {-0.3, 0.3, 0.75},
//     {-0.3, 0.3, 0.9},
//     //window2
//     {-0.4, 0, 0.3},
//     {-0.4, 0.15, 0.3},
//     {-0.4, -0.15, 0.3},
//     {-0.4, 0, 0.9},
//     {-0.4, 0.15, 0.9},
//     {-0.4, -0.15, 0.9},
//
//     {-0.4, -0.3, 0.3},
//     {-0.4, -0.3, 0.45},
//     {-0.4, -0.3, 0.6},
//     {-0.4, -0.3, 0.75},
//     {-0.4, -0.3, 0.9},
//
//     {-0.4, 0.3, 0.3},
//     {-0.4, 0.3, 0.45},
//     {-0.4, 0.3, 0.6},
//     {-0.4, 0.3, 0.75},
//     {-0.4, 0.3, 0.9},
//
//     {-0.5, 0, 0.3},
//     {-0.5, 0.15, 0.3},
//     {-0.5, -0.15, 0.3},
//     {-0.5, 0, 0.9},
//     {-0.5, 0.15, 0.9},
//     {-0.5, -0.15, 0.9},
//
//     {-0.5, -0.3, 0.3},
//     {-0.5, -0.3, 0.45},
//     {-0.5, -0.3, 0.6},
//     {-0.5, -0.3, 0.75},
//     {-0.5, -0.3, 0.9},
//
//     {-0.5, 0.3, 0.3},
//     {-0.5, 0.3, 0.45},
//     {-0.5, 0.3, 0.6},
//     {-0.5, 0.3, 0.75},
//     {-0.5, 0.3, 0.9},
//
//     {-0.6, 0, 0.3},
//     {-0.6, 0.15, 0.3},
//     {-0.6, -0.15, 0.3},
//     {-0.6, 0, 0.9},
//     {-0.6, 0.15, 0.9},
//     {-0.6, -0.15, 0.9},
//
//     {-0.6, -0.3, 0.3},
//     {-0.6, -0.3, 0.45},
//     {-0.6, -0.3, 0.6},
//     {-0.6, -0.3, 0.75},
//     {-0.6, -0.3, 0.9},
//
//     {-0.6, 0.3, 0.3},
//     {-0.6, 0.3, 0.45},
//     {-0.6, 0.3, 0.6},
//     {-0.6, 0.3, 0.75},
//     {-0.6, 0.3, 0.9},
//
//     {-0.7, 0, 0.3},
//     {-0.7, 0.15, 0.3},
//     {-0.7, -0.15, 0.3},
//     {-0.7, 0, 0.9},
//     {-0.7, 0.15, 0.9},
//     {-0.7, -0.15, 0.9},
//
//     {-0.7, -0.3, 0.3},
//     {-0.7, -0.3, 0.45},
//     {-0.7, -0.3, 0.6},
//     {-0.7, -0.3, 0.75},
//     {-0.7, -0.3, 0.9},
//
//     {-0.7, 0.3, 0.3},
//     {-0.7, 0.3, 0.45},
//     {-0.7, 0.3, 0.6},
//     {-0.7, 0.3, 0.75},
//     {-0.7, 0.3, 0.9},
// };
//
// static const std::vector<std::array<float, 3>> duck = {  //d=2.65, samples=1500
//     {-0.3, 0.3, 0.0},
//     {-0.3, 0.3, 0.1},
//     {-0.3, 0.3, 0.2},
//     {-0.3, 0.3, 0.3},
//     {-0.3, 0.3, 0.4},
//     {-0.3, 0.3, 0.5},
//     {-0.3, 0.3, 0.6},
//     {-0.3, 0.3, 0.7},
//     {-0.3, 0.3, 0.8},
//
//     {-0.3, -0.3, 0.0},
//     {-0.3, -0.3, 0.1},
//     {-0.3, -0.3, 0.2},
//     {-0.3, -0.3, 0.3},
//     {-0.3, -0.3, 0.4},
//     {-0.3, -0.3, 0.5},
//     {-0.3, -0.3, 0.6},
//     {-0.3, -0.3, 0.7},
//     {-0.3, -0.3, 0.8},
//
//     {-0.3, -0.2, 0.55},
//     {-0.3, -0.1, 0.55},
//     {-0.3, 0, 0.55},
//     {-0.3, 0.1, 0.55},
//     {-0.3, 0.2, 0.55},
//
//     {-0.3, -0.2, 0.6},
//     {-0.3, -0.1, 0.6},
//     {-0.3, 0, 0.6},
//     {-0.3, 0.1, 0.6},
//     {-0.3, 0.2, 0.6},
//
// };

// static const std::vector<std::array<float, 3>> problem3 = { // d=2.85, samples = 19000
// {-0.3, -0.3, 0.9},
// {0, -0.3, 0.5},
// {0, -0.3, 0.2},
// {0.1, -0.2, 0.6},
// {-0.2, -0.3, 0.7},
// {0, -0.2, 0},
//
// {-0.3, 0.3, 0.9},
// {0, 0.3, 0.5},
// {0, 0.3, 0.2},
// {0.1, 0.2, 0.6},
// {-0.2, 0.3, 0.7},
//
// {0, 0, 0.9},
// {-0.2, 0, 1.0},
// };

// // d=2.9 standing
// static const std::vector<std::array<std::array<float, 3>,3>> window_cubes = {
//     {{
//     {0, 0, -0.03},
//     {0, 0, 0},
//     {1, 1, 0.03},
//     }},
//     {{
//         {-0.5, -0.4, 0.6},
//         {0, 0, 0},
//         {0.05, 0.1, 0.3}
//     }},
//     {{
//         {-0.5, 0.4, 0.6},
//         {0, 0, 0},
//         {0.05, 0.1, 0.3}
//     }},
//     {{
//         {-0.5, 0, 0.9},
//         {0, 0, 0},
//         {0.05, 0.3, 0.1}
//     }},
//     {{
//         {-0.5, 0.0, 0.3},
//         {0, 0, 0},
//         {0.05, 0.3, 0.1}
//     }},
// };
//
// // d=2.0 laying down
// static const std::vector<std::array<std::array<float, 3>,3>> sleeve_cubes = {
//     {{
//         {0, 0, -0.03},
//         {0, 0, 0},
//         {1, 1, 0.03},
//     }},
//     {{
//         {-0.5, -0.4, 0.6},
//         {0, 0, 0},
//         {0.35, 0.1, 0.3}
//     }},
//     {{
//         {-0.5, 0.4, 0.6},
//         {0, 0, 0},
//         {0.35, 0.1, 0.3}
//     }},
//     {{
//         {-0.5, 0, 0.9},
//         {0, 0, 0},
//         {0.35, 0.3, 0.1}
//     }},
//     {{
//         {-0.5, 0.0, 0.3},
//         {0, 0, 0},
//         {0.35, 0.3, 0.1}
//     }}
// };


/// a basic structure to keep tests in
class VampTestData {
public:
    std::string name;
    std::vector<double> start;
    std::vector<double> goal;
    double delta;
    std::vector<std::array<float, 3>> spheres;
    std::vector<std::array<std::array<float, 3>,3>> cubes;
};

const static VampTestData empty = {
    "empty",
    {0., -0.785, 0., -2.356, 0., 1.571, 0.785},
    {-2.37, 0.69, -1.08, -1.25 , 1.18, 2, -2.79},
    2.9,
    {},
    {}
};

// test #1
const static VampTestData high_window = {
    "window",
    {0., -0.785, 0., -2.356, 0., 1.571, 0.785},
    {-2.37, 0.69, -1.08, -1.25 , 1.18, 2, -2.79},
    2.9,
    {},
{
        {{
            {0, 0, -0.03},
            {0, 0, 0},
            {1, 1, 0.03},
            }},
            {{
                {-0.5, -0.4, 0.6},
                {0, 0, 0},
                {0.05, 0.1, 0.3}
            }},
            {{
                {-0.5, 0.4, 0.6},
                {0, 0, 0},
                {0.05, 0.1, 0.3}
            }},
            {{
                {-0.5, 0, 0.9},
                {0, 0, 0},
                {0.05, 0.3, 0.1}
            }},
            {{
                {-0.5, 0.0, 0.3},
                {0, 0, 0},
                {0.05, 0.3, 0.1}
            }},
        }

};

// test #2
const static VampTestData sleeve = {
    "sleeve",
    {0., -0.785, 0., -2.356, 0., 1.571, 0.785},
    {-2.37, 0.69, -1.08, -1.25 , 1.18, 2, -2.79},
    2,
    {},
{
        {{
            {0, 0, -0.03},
            {0, 0, 0},
            {1, 1, 0.03},
        }},
        {{
            {-0.5, -0.4, 0.6},
            {0, 0, 0},
            {0.35, 0.1, 0.3}
        }},
        {{
            {-0.5, 0.4, 0.6},
            {0, 0, 0},
            {0.35, 0.1, 0.3}
        }},
        {{
            {-0.5, 0, 0.9},
            {0, 0, 0},
            {0.35, 0.3, 0.1}
        }},
        {{
            {-0.5, 0.0, 0.3},
            {0, 0, 0},
            {0.35, 0.3, 0.1}
        }}
}

};

// test #3
const static VampTestData limbo = {
    "limbo",
    {0., -0.785, 0., -2.356, 0., 1.571, 0.785},
    {-2.37, 0.69, -1.08, -1.25 , 1.18, 2, -2.79},
    1.6,
    {},
{
        {{
            {0, 0, -0.03},
            {0, 0, 0},
            {1, 1, 0.03},
        }},
        {{
            {-0.3, 0.3, 0.5},
            {0, 0, 0},
            {0.03, 0.03, 0.5}
        }},
        {{
            {-0.3, -0.3, 0.5},
            {0, 0, 0},
            {0.03, 0.03, 0.5}
        }},
        {{
            {-0.3, 0.0, 0.725},
            {0, 0, 0},
            {0.03, 0.3, 0.05}
        }},
    }
};

// test #4
const static VampTestData between_walls = {
    "low_room",
    {0., 1.571, 0., 0, 0., 1.571, 0.785},
    {0., -1.571, 0., 0, 0., 1.571, 0.785},
    2.5,
    {},
{
        {{
            {0, 0, -0.03},
            {0, 0, 0},
            {1, 1, 0.03},
        }},
        {{
            {0, 0, 0.75},
            {0, 0, 0},
            {1, 1, 0.03}
        }},
    }
};

// test #5
const static VampTestData low_window_with_balls = {
    "complex",
    {0., 1.571, 0., 0, 0., 1.571, 0.785},
    {0., -1.571, 0., 0, 0., 1.571, 0.785},
    1.3,
    {
        {-0.2, -0.2, 0.2},
        {0.1, 0.2, 0.1},
        {-0.8, -0.2, 0.2},
    },
    {
        {{
            {0, 0, -0.03},
            {0, 0, 0},
            {1, 1, 0.03},
            }},
            {{
                {-0.5, -0.4, 0.3},
                {0, 0, 0},
                {0.05, 0.1, 0.3}
            }},
            {{
                {-0.5, 0.4, 0.3},
                {0, 0, 0},
                {0.05, 0.1, 0.3}
            }},
            {{
                {-0.5, 0, 0.6},
                {0, 0, 0},
                {0.05, 0.3, 0.1}
            }},
            {{
                {-0.5, 0.0, 0.0},
                {0, 0, 0},
                {0.05, 0.3, 0.1}
            }},
        {{
            {-0.2, -0.2, 0.0},
            {0, 0, 0},
            {0.15, 0.15, 0.05}
        }},
        {{
            {-0.8, -0.2, 0.0},
            {0, 0, 0},
            {0.15, 0.15, 0.05}
        }},
    }
};

const static std::vector<VampTestData> tests = {
    empty, high_window, sleeve, limbo, between_walls, low_window_with_balls
};

// Radius for obstacle spheres
static constexpr float radius = 0.05;

// Maximum planning time
static constexpr float planning_time = 100;

// Maximum simplification time
static constexpr float simplification_time = 0.0;

// Convert an OMPL state into a VAMP vector
inline static auto ompl_to_vamp(const ob::State *state) -> Configuration
{
    // Create an aligned memory buffer to load VAMP vector from
    alignas(Configuration::S::Alignment)
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
            aligned_buffer;

    // Copy OMPL data into aligned buffer
    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    for (auto i = 0U; i < dimension; ++i)
    {
        aligned_buffer[i] = static_cast<float>(as->values[i]);
    }

    // Create configuration from aligned buffer data
    return Configuration(aligned_buffer.data());
}

// Convert a VAMP vector to an OMPL state
inline static auto vamp_to_ompl(const Configuration &c, ob::State *state)
{
    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    for (auto i = 0U; i < dimension; ++i)
    {
        as->values[i] = static_cast<double>(c[{i, 0}]);
    }
}

// State validator using VAMP
struct VAMPStateValidator : public ob::StateValidityChecker
{
    VAMPStateValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    VAMPStateValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    auto isValid(const ob::State *state) const -> bool override
    {
        // Convert OMPL to VAMP vector and validate
        auto configuration = ompl_to_vamp(state);
        // return si_->satisfiesBounds(state) && vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_v);
        bool res = vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_v);
        if (!res) return false;
        for (int i = 0; i < 7; ++i) {
            double value = state->as<ob::RealVectorStateSpace::StateType>()->values[i];
            if (i == 3) {
                res = res && (value > -3.15 && value < 0.09);
            } else if (i == 5) {
                res = res && (value > -0.09 && value < 3.85);
            } else {
                // res = res && (abs(value) < 1.1 * M_PI);
                res = res && (abs(value) < 1.1 * M_PI);
            }
        }
        return res;
    }

    const EnvironmentVector &env_v;
};

struct VAMPMotionValidator : public ob::MotionValidator
{
    VAMPMotionValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    VAMPMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
    {
        // Convert OMPL states to VAMP vectors and check motion between states
        return vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
            ompl_to_vamp(s1), ompl_to_vamp(s2), env_v);
    }

    auto
    checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &o) const -> bool override
    {
        // throw ompl::Exception("Not implemented!");
        return vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
    ompl_to_vamp(s1), ompl_to_vamp(s2), env_v);
    }

    const EnvironmentVector &env_v;
};

// std::tuple<double, double> PRM(bool optimize = false) {
//     // Build sphere cage environment
//     EnvironmentInput environment;
//     for (const auto &sphere : problem3)
//     {
//         environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
//     }
//
//
//     environment.sort();
//     auto env_v = EnvironmentVector(environment);
//
//     // Create OMPL state space
//     auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
//
//     // Get bounds from VAMP Robot information, scale 0/1 config to min/max
//     static constexpr std::array<float, dimension> zeros = {0., 0., 0., 0., 0., 0., 0.};
//     static constexpr std::array<float, dimension> ones = {1., 1., 1., 1., 1., 1., 1.};
//
//     auto zero_v = Configuration(zeros);
//     auto one_v = Configuration(ones);
//
//     Robot::scale_configuration(zero_v);
//     Robot::scale_configuration(one_v);
//
//     std::cout << zero_v << std::endl;
//     std::cout << one_v << std::endl;
//     // auto a = zero_v.data
//     ob::RealVectorBounds bounds(dimension);
//     // auto a = zero_v.data._M_elems[0];
//     for (auto i = 0U; i < dimension; ++i)
//     {
//         bounds.setLow(i, zero_v[{0, i}]);
//         bounds.setHigh(i, one_v[{0, i}]);
//         // bounds.setLow(i, zero_v.data._M_elems[0][i]);
//         // bounds.setHigh(i, one_v.data._M_elems[0][i]);
//     }
//
//     space->setBounds(bounds);
//
//     // Create space information and set state validator and custom VAMP motion validator
//     auto si = std::make_shared<ob::SpaceInformation>(space);
//
//     si->setStateValidityChecker(std::make_shared<VAMPStateValidator>(si, env_v));
//     si->setMotionValidator(std::make_shared<VAMPMotionValidator>(si, env_v));
//     si->setup();
//
//     // Set start and goal
//     ob::ScopedState<> start_ompl(space), goal_ompl(space);
//     for (auto i = 0U; i < dimension; ++i)
//     {
//         start_ompl[i] = start[i];
//         goal_ompl[i] = goal[i];
//     }
//
//     auto pdef = std::make_shared<ob::ProblemDefinition>(si);
//     pdef->setStartAndGoalStates(start_ompl, goal_ompl);
//
//     // Set optimization objective
//     auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
//     pdef->setOptimizationObjective(obj);
//
//     if (not optimize)
//     {
//         // Set planner to terminate as soon as a solution is found.
//         obj->setCostThreshold(obj->infiniteCost());
//     }
//
//     // Create planner - BITstar by default, but you can change this to use other geometric planners instead
//     auto planner = std::make_shared<og::PRM>(si);
//
//     planner->setProblemDefinition(pdef);
//     planner->setup();
//
//     // Solve the problem
//     auto start_time = std::chrono::steady_clock::now();
//     ob::PlannerStatus solved = planner->ob::Planner::solve(planning_time);
//     auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
//
//     // Only accept exact solutions
//     if (solved == ob::PlannerStatus::EXACT_SOLUTION)
//     {
//         std::cout << "Found solution in " << nanoseconds / 1e6 << "ms! Simplfying..." << std::endl;
//
//         // Simplify the path using OMPL's path simplification
//         const ob::PathPtr &path = pdef->getSolutionPath();
//         og::PathGeometric &path_geometric = static_cast<og::PathGeometric &>(*path);
//
//         auto initial_cost = path_geometric.cost(obj);
//
//         og::PathSimplifier simplifier(si, pdef->getGoal(), obj);
//         if (not simplifier.simplify(path_geometric, simplification_time))
//         {
//             std::cout << "Path not valid!" << std::endl;
//         }
//
//         auto simplified_cost = path_geometric.cost(obj);
//
//         // Output statistics
//         std::cout << "Found initial solution with cost " << initial_cost.value() << std::endl;
//         std::cout << "Simplified solution to cost " << simplified_cost.value() << std::endl;
//         std::cout << "Simplified solution:" << std::endl;
//
//         path_geometric.print(std::cout);
//         return {nanoseconds / 1e6, simplified_cost.value()};
//     }
//     else
//     {
//         std::cout << "No solution found" << std::endl;
//     }
//     return {-1, -1};
// }

std::tuple<double, double> iRndPRM(VampTestData test, double timeLimit, bool optimize = false, double astarRadius = 1, long samples = 0) {
    // Build sphere cage environment
    EnvironmentInput environment;
    // for (const auto &sphere : problem2)
    // {
    //     environment.spheres.emplace_back(vamp::collision::factory::sphere::array({sphere[0], sphere[1] - 0.1, sphere[2]}, radius));
    // }
    // for (const auto &sphere : duck)
    // {
        // environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    // }
    for (const auto &sphere : test.spheres)
    {
    environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }
    for (const auto &cube : test.cubes)
    {
    environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(cube[0], cube[1], cube[2]));
    }


    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // Create OMPL state space
    auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);

    // Get bounds from VAMP Robot information, scale 0/1 config to min/max
    static constexpr std::array<float, dimension> zeros = {0., 0., 0., 0., 0., 0., 0.};
    static constexpr std::array<float, dimension> ones = {1., 1., 1., 1., 1., 1., 1.};

    auto zero_v = Configuration(zeros);
    auto one_v = Configuration(ones);

    Robot::scale_configuration(zero_v);
    Robot::scale_configuration(one_v);

    std::cout << zero_v << std::endl;
    std::cout << one_v << std::endl;
    // auto a = zero_v.data
    ob::RealVectorBounds bounds(dimension);
    // auto a = zero_v.data._M_elems[0];
    double volume = 1;
    for (auto i = 0U; i < dimension; ++i)
    {
        bounds.setLow(i, zero_v[{0, i}]);
        bounds.setHigh(i, one_v[{0, i}]);
        volume *= one_v[{0, i}] - zero_v[{0, i}];
        // bounds.setLow(i, zero_v.data._M_elems[0][i]);
        // bounds.setHigh(i, one_v.data._M_elems[0][i]);
    }

    space->setBounds(bounds);

    // Create space information and set state validator and custom VAMP motion validator
    auto si = std::make_shared<ob::SpaceInformation>(space);

    si->setStateValidityChecker(std::make_shared<VAMPStateValidator>(si, env_v));
    si->setMotionValidator(std::make_shared<VAMPMotionValidator>(si, env_v));
    si->setup();

    // Set start and goal
    ob::ScopedState<> start_ompl(space), goal_ompl(space);
    for (auto i = 0U; i < dimension; ++i)
    {
        start_ompl[i] = test.start[i];
        goal_ompl[i] = test.goal[i];
    }

    Eigen::VectorXd startEigen(test.start.size());
    Eigen::VectorXd goalEigen(test.goal.size());
    for (int i = 0; i < test.start.size(); ++i) {
        startEigen[i] = test.start[i];
        goalEigen[i] = test.goal[i];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start_ompl, goal_ompl);

    // Set optimization objective
    auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    pdef->setOptimizationObjective(obj);

    if (not optimize)
    {
        // Set planner to terminate as soon as a solution is found.
        obj->setCostThreshold(obj->infiniteCost());
    }

    // Create planner - BITstar by default, but you can change this to use other geometric planners instead
    auto planner = std::make_shared<og::ImplicitPRMVamp>(si);

    planner->setProblemDefinition(pdef);

    // iPRM specific
    planner->setStartAndGoalEigen(startEigen, goalEigen);
    planner->setDim(test.start.size());
    planner->setVolume(volume);
    planner->setPrmType(ompl::geometric::ImplicitPRMVamp::Random);
    planner->as<og::ImplicitPRMVamp>()->setNNRadius(astarRadius);
    planner->setSampleLimit(samples); //13000
    // planner->setSampleLimit(10000); //13000
    planner->setup();

    // Solve the problem
    auto start_time = std::chrono::steady_clock::now();
    // timeLimit = std::min(100 * timeLimit, 100.0);
    // ob::PlannerStatus solved = planner->ob::Planner::solve(100);
    ob::PlannerStatus solved = planner->ob::Planner::solve(100 * timeLimit);
    auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

    // Only accept exact solutions
    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "Found solution in " << nanoseconds / 1e6 << "ms! Simplfying..." << std::endl;

        // Simplify the path using OMPL's path simplification
        const ob::PathPtr &path = pdef->getSolutionPath();
        og::PathGeometric &path_geometric = static_cast<og::PathGeometric &>(*path);

        auto initial_cost = path_geometric.cost(obj);

        og::PathSimplifier simplifier(si, pdef->getGoal(), obj);
        if (not simplifier.simplify(path_geometric, simplification_time))
        {
            std::cout << "Path not valid!" << std::endl;
        }

        auto simplified_cost = path_geometric.cost(obj);

        // Output statistics
        std::cout << "Found initial solution with cost " << initial_cost.value() << std::endl;
        std::cout << "Simplified solution to cost " << simplified_cost.value() << std::endl;
        std::cout << "Simplified solution:" << std::endl;

        path_geometric.print(std::cout);

        return {nanoseconds / 1e6, simplified_cost.value()};
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }

    return {-1, -1};
}


std::tuple<double, long, double> iPRM(VampTestData test, bool optimize = false) {
    // Build sphere cage environment
    EnvironmentInput environment;
    // for (const auto &sphere : problem2)
    // {
    //     environment.spheres.emplace_back(vamp::collision::factory::sphere::array({sphere[0], sphere[1] - 0.1, sphere[2]}, radius));
    // }
    // for (const auto &sphere : duck)
    // {
        // environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    // }
    for (const auto &sphere : test.spheres)
    {
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }
    for (const auto &cube : test.cubes)
    {
        environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(cube[0], cube[1], cube[2]));
    }

    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // Create OMPL state space
    auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);

    // Get bounds from VAMP Robot information, scale 0/1 config to min/max
    static constexpr std::array<float, dimension> zeros = {0., 0., 0., 0., 0., 0., 0.};
    static constexpr std::array<float, dimension> ones = {1., 1., 1., 1., 1., 1., 1.};

    auto zero_v = Configuration(zeros);
    auto one_v = Configuration(ones);

    Robot::scale_configuration(zero_v);
    Robot::scale_configuration(one_v);

    std::cout << zero_v << std::endl;
    std::cout << one_v << std::endl;
    // auto a = zero_v.data
    ob::RealVectorBounds bounds(dimension);
    // auto a = zero_v.data._M_elems[0];
    double volume = 1;
    for (auto i = 0U; i < dimension; ++i)
    {
        bounds.setLow(i, zero_v[{0, i}]);
        bounds.setHigh(i, one_v[{0, i}]);
        volume *= one_v[{0, i}] - zero_v[{0, i}];
        // bounds.setLow(i, zero_v.data._M_elems[0][i]);
        // bounds.setHigh(i, one_v.data._M_elems[0][i]);
    }

    space->setBounds(bounds);

    // Create space information and set state validator and custom VAMP motion validator
    auto si = std::make_shared<ob::SpaceInformation>(space);

    si->setStateValidityChecker(std::make_shared<VAMPStateValidator>(si, env_v));
    si->setMotionValidator(std::make_shared<VAMPMotionValidator>(si, env_v));
    si->setup();

    // Set start and goal
    ob::ScopedState<> start_ompl(space), goal_ompl(space);
    for (auto i = 0U; i < dimension; ++i)
    {
        start_ompl[i] = test.start[i];
        goal_ompl[i] = test.goal[i];
    }

    Eigen::VectorXd startEigen(test.start.size());
    Eigen::VectorXd goalEigen(test.goal.size());
    for (int i = 0; i < test.start.size(); ++i) {
        startEigen[i] = test.start[i];
        goalEigen[i] = test.goal[i];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start_ompl, goal_ompl);

    // Set optimization objective
    auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    pdef->setOptimizationObjective(obj);

    if (not optimize)
    {
        // Set planner to terminate as soon as a solution is found.
        obj->setCostThreshold(obj->infiniteCost());
    }

    // Create planner - BITstar by default, but you can change this to use other geometric planners instead
    auto planner = std::make_shared<og::ImplicitPRMVamp>(si);

    planner->setProblemDefinition(pdef);

    // iPRM specific
    planner->setStartAndGoalEigen(startEigen, goalEigen);
    planner->setDim(test.start.size());
    // planner->setVolume(volume);
    planner->setPrmType(ompl::geometric::ImplicitPRMVamp::Lattice);
    planner->setCountSamplesForRND(false); // false == comparison only between lattices, true = compare to RND
    planner->setLatticeType(ompl::geometric::ImplicitPRMVamp::AnStar, test.delta, 10); // 2.3 for 0.1, 2.75 for 0.05

    planner->setup();

    // Solve the problem
    auto start_time = std::chrono::steady_clock::now();
    ob::PlannerStatus solved = planner->ob::Planner::solve(planning_time);
    auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

    // Only accept exact solutions
    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {

        // Simplify the path using OMPL's path simplification
        const ob::PathPtr &path = pdef->getSolutionPath();
        og::PathGeometric &path_geometric = static_cast<og::PathGeometric &>(*path);

        auto initial_cost = path_geometric.cost(obj);

        og::PathSimplifier simplifier(si, pdef->getGoal(), obj);
        if (not simplifier.simplify(path_geometric, simplification_time))
        {
            std::cout << "Path not valid!" << std::endl;
        }

        auto simplified_cost = path_geometric.cost(obj);

        // Output statistics
        std::cout << "Found initial solution with cost " << initial_cost.value() << std::endl;
        std::cout << "Simplified solution to cost " << simplified_cost.value() << std::endl;

        std::cout << "T = " << nanoseconds / 1e6 << "ms, L = " <<simplified_cost.value() << std::endl;

        std::cout << "Simplified solution:" << std::endl;

        path_geometric.print(std::cout);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
    auto res = planner->as<og::ImplicitPRMVamp>()->getResults();
    return {planner->as<og::ImplicitPRMVamp>()->getRadius(), res.samples, nanoseconds / 1e6};
}

int VAMPTests::runVampTests(int argc, const std::string& folderBase)
{
    bool optimize = false;  // Flag - if true, will spend entire planning budget optimizing, otherwise exit on
                            // first solution

    // Set optimize flag if another argument is provided
    if (argc == 2)
    {
        optimize = true;
    }
    int powers = 1;
    std::vector<std::string> res;
    for (int j = 0; j < powers; ++j) {
        for (const auto& test: {tests[4]}) {
            std::cout << fmt::format("Starting test [{}], with [delta={}]", test.name, test.delta) << std::endl;
            auto [radius, samples, runtime] = iPRM(test, optimize);

            // return 0;
            double avgT = 0;
            double avgL = 0;
            int success = 0;
            int RUNS = 100;
            for (int i = 0; i < RUNS; ++i) {
                auto [time, length] = iRndPRM(test, std::pow(10, j) * runtime / 1000.0, optimize, radius, std::pow(10, j) * samples);
                if (time > 0) {
                    avgT += time;
                    avgL += length;
                    success++;
                }
            }
            res.push_back(fmt::format("Test = {}, delta = {}", test.name, test.delta));
            res.push_back(fmt::format("Finished iPRM with samples={} and runtime={}ms. Reported radius={}",
                samples, runtime, radius));
            res.push_back(fmt::format("[{}/{}]: PRM avgT = {}ms, AvgL = {}", success, RUNS, avgT / success,
                avgL / success));
            res.push_back("====================================================================\n");
        }
    }

    // print results
    // create dir
    std::ofstream outFile;
    std::filesystem::create_directories(folderBase);
    outFile.open(folderBase + "vampTests.txt");
    std::cout << "Printing lattice results" << std::endl;
    if (outFile.is_open()) {
        for (const auto& elem: res) {
            std::cout << elem << std::endl;
            outFile << elem << std::endl;
        }
    }

    return 0;
}
