#include "pch.h"
#include "Samplers.h"
namespace ompl
{
    namespace base
    {

        std::vector<double> SGSet::GetNextSample()
        {
            std::vector<double> res;

            for (int i = 0; i < this->_dim; i++)
            {
                if (this->_base + this->_axisIndices[i] * this->_cubeSide > 1 - this->_delta && i < this->_dim) {
                    if (i + 1 == this->_dim) {
                        if (shiftHappened)
                        {
                            // cycle back to the first sample
                            this->_base = this->_delta;
                            shiftHappened = false;
                            return this->GetNextSample();
                        }
                        this->_base = this->_delta + (this->_cubeSide / 2.0);
                        for (int j = 0; j < this->_dim; j++)
                        {
                            this->_axisIndices[j] = 0;
                        }
                        shiftHappened = true;
                        // reset to the corner
                        for (int j = 0; j < this->_dim; j++)
                        {
                            _currentSample[j] = this->_base;
                        }
                    }
                    else {
                        this->_axisIndices[i] = 0;
                        this->_axisIndices[i + 1]++;
                    }
                }

                _currentSample[i] = this->_base + this->_axisIndices[i] * this->_cubeSide;
                res.push_back(_currentSample[i]);
            }
            this->_axisIndices[0]++;
            return res;
        }

        StaggeredGrid::StaggeredGrid(unsigned int dimensions, double delta) :
            DeterministicSequence(dimensions), sg_(SGSet(dimensions, delta)) {}

        std::vector<double> StaggeredGrid::sample()
        {

            //std::vector<double> res1 = sg_.GetNextSample();
            //std::vector<double> res;
            //std::cout << "===========" << std::endl;
            //for (double v : sg_.GetNextSample()) {
            //    std::cout << v << std::endl;
            //    res1.push_back(0);
            //}
            //std::cout << "~~~~~~~~~" << std::endl;
            //std::cout << res1.size() << std::endl;
            //std::cout << "!!!!!!!!!!" << std::endl;
            //for (double v : res1) {
            //    std::cout << v << std::endl;
            //}
            //res.push_back(0);
            //res.push_back(0);
            return sg_.GetNextSample();
        }
    }
}