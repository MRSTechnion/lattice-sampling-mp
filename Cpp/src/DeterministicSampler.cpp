//
// Created by itai on 7/22/24.
//

#include "../include/DeterministicSampler.h"
#include <boost/stacktrace.hpp>
#include "../include/Lattices.h"

/*
 * This sampler only samples a finite amount of discrete samples.
 * current options are:
 * (1) file   LatticeType type, const std::string& sampleFilePath
 * (2) TODO Zn, Dn*, An*
 */
DeterministicSampler::DeterministicSampler(const ob::SpaceInformation *si):
ValidStateSampler(si), noMoreSamples_(false), totalSampleCount_(0), sampleCount_(0) {
    OMPL_INFORM("Creating a deterministic sampler");
    // set up params
    noMoreSamples_ = false;
    params_.declareParam<bool>("SamplingDone",
                        [this](bool param) {;},
                        [this]{ return getSamplingStatus(); });
    setSamplingType(AnStar, 3, "");
}

void DeterministicSampler::setSamplingType(const LatticeType &type, const int& dim, const std::string &sampleFilePath) {
    type_ = type;
    switch (type) {
        case Zn:
            name_ = "Zn";
            d_ = dim;
            // allSamples_ = Lattices::createSamples(dim, 1, 0.5, Zn);
            // std::cout << "finished creating samples " << allSamples_->size() << std::endl;
            // totalSampleCount_ = allSamples_->size();
            // Zn_ = new Lattices::Zn(dim, 3, 500);
            // Zn_ = new Lattices::Lattice(dim, 3.0);
            std::cout << "allocated Zn sampler " << std::endl;
            break;
        case DnStar:
            name_ = "Dn*";
            break;
        case AnStar:
            name_ = "An*";
            break;
        case File:
            if (sampleFilePath.empty()) {
                OMPL_ERROR("Requested to get the samples from a file, but didn't supply the file");
                return;
            }
            std::string base_filename = sampleFilePath.substr(sampleFilePath.find_last_of("/\\") + 1);
            std::string::size_type const p(base_filename.find_last_of('.'));
            std::string file_without_extension = base_filename.substr(0, p);
            name_ = file_without_extension;
            // std::string sampleFile = "/home/itai/ompl3/ompl-1.6.0/tests/resources/halton/halton_2d.txt";
            auto fileSamples_ = new ob::PrecomputedSequence(sampleFilePath, 2);
            // load the file
            std::string line;
            std::ifstream samplefile(sampleFilePath);
            while (std::getline(samplefile, line)) {
                ++totalSampleCount_;
            }
            // load the samples
            allSamples_ = new std::vector<std::vector<double> *>;
            while (++sampleCount_ <= totalSampleCount_) {
                allSamples_->push_back(new std::vector<double>(fileSamples_->sample()));
            }
            sampleCount_ = 0;
            break;
    }
    css_ = this->si_->getStateSpace()->as<ob::CompoundStateSpace>();
}


bool DeterministicSampler::getSamplingStatus() const {
    return noMoreSamples_;
}

// Generate a sample from the sample set
// bool DeterministicSampler::sample(ob::State *state) {
//     // std::cout << sampleCount_ << std::endl;
//     if (sampleCount_ == totalSampleCount_) {
//         if (!noMoreSamples_) {
//             noMoreSamples_ = true;
//         }
//         return false;
//     }
//     const std::vector<double>& sample = *((*allSamples_)[sampleCount_++]);
//     // std::cout << sampleCount_ << ": " << sample[0] << "," << sample[1] << "," << sample[2] << std::endl;
//     // auto val_x = this->si_->getStateSpace()->as<ob::CompoundStateSpace>()->getValueAddressAtIndex(state, 0);
//     // auto val_y = this->si_->getStateSpace()->as<ob::CompoundStateSpace>()->getValueAddressAtIndex(state, 1);
//     // *val_x = sample[0];
//     // *val_y = sample[1];
//     for (int i = 0; i < d_; i++) {
//         // auto val = (this->si_->getStateSpace()->as<ob::CompoundStateSpace>()->getValueAddressAtIndex(state, i);
//         // auto s = state->as<ob::RealVectorStateSpace::StateType>();
//         // s[0] = 1;
//         // state->as<ob::RealVectorStateSpace::StateType>()->values[i] = sample[i];
//         auto val = css_->getValueAddressAtIndex(state, i);
//         // auto val_y = this->si_->getStateSpace()->as<ob::CompoundStateSpace>()->getValueAddressAtIndex(state, 1);
//
//         // if (sample[2] != 0) {
//             // int x  =1;
//         // }
//         *val = sample[i];
//     }
//     // if (sample[0] == 0 && sample[1] == 0 && sample[2] == 0) {
//         // int x = 1;
//     // }
//     /* Use this is you want to see the valid samples.
//     if (si_->isValid(state)) {
//         std::cout << *val_x << "," << *val_y << std::endl;
//     }
//     */
//     // return si_->isValid(state);
//     return true;
// }

bool DeterministicSampler::sample(ob::State *state) {
    // std::cout << sampleCount_ << std::endl;
    // if (sampleCount_ == totalSampleCount_) {
        // if (!noMoreSamples_) {
            // noMoreSamples_ = true;
        // }
        // return false;
    // }
    std::vector<double> sample = Zn_->sample();
    // std::cout << sampleCount_ << ": " << sample[0] << "," << sample[1] << "," << sample[2] << std::endl;
    // auto val_x = this->si_->getStateSpace()->as<ob::CompoundStateSpace>()->getValueAddressAtIndex(state, 0);
    // auto val_y = this->si_->getStateSpace()->as<ob::CompoundStateSpace>()->getValueAddressAtIndex(state, 1);
    // *val_x = sample[0];
    // *val_y = sample[1];
    for (int i = 0; i < d_; i++) {
        // auto val = (this->si_->getStateSpace()->as<ob::CompoundStateSpace>()->getValueAddressAtIndex(state, i);
        // auto s = state->as<ob::RealVectorStateSpace::StateType>();
        // s[0] = 1;
        // state->as<ob::RealVectorStateSpace::StateType>()->values[i] = sample[i];
        auto val = css_->getValueAddressAtIndex(state, i);
        // auto val_y = this->si_->getStateSpace()->as<ob::CompoundStateSpace>()->getValueAddressAtIndex(state, 1);

        // if (sample[2] != 0) {
        // int x  =1;
        // }
        *val = sample[i];
    }
    return si_->isValid(state);
    // return true;
}

// We don't need this in the example below.
bool DeterministicSampler::sampleNear(ob::State * /*state*/, const ob::State * /*near*/, const double /*distance*/) {
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
    return false;
}

ob::ValidStateSamplerPtr allocValidDeterministicSampler(const ob::SpaceInformation *si)
{
    return std::make_shared<DeterministicSampler>(si);
}
