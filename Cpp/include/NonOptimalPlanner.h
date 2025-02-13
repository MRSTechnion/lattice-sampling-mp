#pragma once

#include "MyValidStateSampler.h"
#include "MRMPlanner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class NonOptimalPlanner : public MRMPlanner
{
protected:
	ob::PlannerPtr allocatePlanner(const ob::SpaceInformationPtr& si, int plannerType);
	//static ob::ValidStateSamplerPtr allocMyValidStateSampler(const ob::SpaceInformation* si);
public:
	NonOptimalPlanner(LPEXTFUNCRESPOND log) : MRMPlanner(log) {}
	static ob::ValidStateSamplerPtr allocateSampler(const ob::SpaceInformation* si);
	void plan(PlanData pData);
};

