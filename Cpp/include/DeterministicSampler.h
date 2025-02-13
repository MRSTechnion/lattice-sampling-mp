//
// Created by itai on 7/22/24.
//

/*
 *
 * This is a "Deterministic Sampler". You can use this sampler to get:
 * (1) samples listed in a file
 * (2) samples from the Lattices Zn,Dn*,An*
 *
 */

#pragma once

#include <algorithm>
#include <random>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/samplers/deterministic/PrecomputedSequence.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <fstream>
#include "../include/DeterministicPRM.h"
namespace ob = ompl::base;

class DeterministicSampler : public ob::ValidStateSampler {
public:
    DeterministicSampler() : ValidStateSampler(nullptr), Zn_(nullptr),
    noMoreSamples_(false), totalSampleCount_(0), sampleCount_(0), type_() {};
    explicit DeterministicSampler(const ob::SpaceInformation *si);

    void setSamplingType(const LatticeType &type, const int& dim, const std::string& sampleFilePath = "");
    bool getSamplingStatus() const;
    void setter(bool a);

    bool sample(ob::State *state) override;
    bool sampleNear(ob::State * /*state*/, const ob::State * /*near*/, const double /*distance*/) override;
protected:
    ompl::RNG rng_;
private:
    ob::PrecomputedSequence* fileSamples_{};
    std::vector<std::vector<double>*>* allSamples_{};
    int d_;
    bool noMoreSamples_;
    int totalSampleCount_;
    int sampleCount_;
    LatticeType type_;
    Lattices::Zn* Zn_;
    ompl::base::CompoundStateSpace* css_;

};

ob::ValidStateSamplerPtr allocValidDeterministicSampler(const ob::SpaceInformation *si);