#pragma once

#include "SGSet.h"
#include "ompl/base/samplers/deterministic/DeterministicSequence.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


// This is a problem-specific sampler that automatically generates valid
// states; it doesn't need to call SpaceInformation::isValid. This is an
// example of constrained sampling. If you can explicitly describe the set valid
// states and can draw samples from it, then this is typically much more
// efficient than generating random samples from the entire state space and
// checking for validity.
namespace ompl
{
    namespace base
    {

        class SGSet
        {
        private:
            int _dim;
            //int _axis;
            //int _axisCounter;
            //double _width;
            double _delta;
            double _cubeSide;
            double _base;
            bool shiftHappened;
            std::vector<double> _axisIndices;

        public:
            std::vector<double> _currentSample;

            SGSet() {}
            SGSet(int dim/*, double width*/, double delta) :/* _axis(0), _axisCounter(0), _width(width),*/ _delta(delta), _dim(dim) {
                for (int i = 0; i < dim; i++)
                {
                    _currentSample.push_back(delta);
                    _axisIndices.push_back(0);
                }
                _cubeSide = std::sqrt(8.0 / dim) * delta;
                _base = delta;
                shiftHappened = false;
            }
            // get the next sample in line until we get the max number of samples.
            std::vector<double> GetNextSample();
        };

        class StaggeredGrid : public ob::DeterministicSequence
        {
        public:
            /** \brief Constructor, only specifiying the dimensions, first n primes will be used
            as bases. */
            StaggeredGrid(unsigned int dimensions, double delta);

            /** \brief Returns the next sample in the interval [0,1] */
            std::vector<double> sample() override;
            //StaggeredGrid(const ob::SpaceInformation* si/*, int dim, double delta*/) : ValidStateSampler(si)
            //{
            //    //this->sg = SGSet(3, 0.1);// dim, delta);
            //    name_ = "SG Sampler";
            //}
            // Generate a sample in the valid part of the R^dim state space
            //std::vector<double> sample(ob::State* state) override
            //{
            //    //double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
            //    ////std::vector<double> sample = sg.GetNextSample();
            //    ////if (sample.size() == 0) return false;
            //    ////double* samplevals = sample.data();
            //    //for (int i = 0; i < 3; i++)
            //    //{
            //    //    val[i] = 0.2;
            //    //}
            //    //assert(si_->isValid(state));
            //    //return true;
            //    std::vector<double> samples;
            //    for (auto& seq : halton_sequences_1d_)
            //    {
            //        samples.push_back(seq.sample());
            //    }
            //    return samples;
            //}
            //// We don't need this in the example below.
            //bool sampleNear(ob::State* /*state*/, const ob::State* /*near*/, const double /*distance*/) override
            //{
            //    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
            //    return false;
            //}
        //protected:
            SGSet sg_;
            //    //ompl::RNG rng_;
        };

        class testSampler : public ob::ValidStateSampler
        {
        public:
            testSampler(const ob::SpaceInformation* si) : ValidStateSampler(si)
            {
                name_ = "my sampler";
            }
            // Generate a sample in the valid part of the R^3 state space
            // Valid states satisfy the following constraints:
            // -1<= x,y,z <=1
            // if .25 <= z <= .5, then |x|>.8 and |y|>.8
            bool sample(ob::State* state) override
            {
                double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
                double z = rng_.uniformReal(-1, 1);

                if (z > .25 && z < .5)
                {
                    double x = rng_.uniformReal(0, 1.8), y = rng_.uniformReal(0, .2);
                    switch (rng_.uniformInt(0, 3))
                    {
                    case 0: val[0] = x - 1;  val[1] = y - 1;  break;
                    case 1: val[0] = x - .8; val[1] = y + .8; break;
                    case 2: val[0] = y - 1;  val[1] = x - 1;  break;
                    case 3: val[0] = y + .8; val[1] = x - .8; break;
                    }
                }
                else
                {
                    val[0] = rng_.uniformReal(-1, 1);
                    val[1] = rng_.uniformReal(-1, 1);
                }
                val[2] = z;
                assert(si_->isValid(state));
                return true;
            }
            // We don't need this in the example below.
            bool sampleNear(ob::State* /*state*/, const ob::State* /*near*/, const double /*distance*/) override
            {
                throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
                return false;
            }
        protected:
            ompl::RNG rng_;
        };

    }
}