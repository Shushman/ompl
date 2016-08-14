/* Author : Shushman Choudhury */

#include "ompl/base/samplers/informed/RejectionInfPrecomputedSampler.h"
#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        RejectionInfPrecomputedSampler::RejectionInfPrecomputedSampler(const ProblemDefinitionPtr &probDefn,
                                           const StateSpace *space, const std::vector<const State *> &states)
            : InformedSampler(probDefn, states.size()), 
              space_(space), 
              states_(states), 
              numStates(states.size()), 
              currIndex(0)
        {
            if (InformedSampler::opt_->hasCostToGoHeuristic() == false)
            {
                OMPL_WARN("RejectionInfPrecomputedSampler: The optimization objective does not have a cost-to-go heuristic "
                          "defined. Informed sampling will likely have little to no effect.");
            }
        }

        bool RejectionInfPrecomputedSampler::sampleUniform(State *statePtr, const Cost &maxCost)
        {
            bool foundSample = false;

            for(; currIndex < numStates && foundSample == false; currIndex++)
            {
                space_ -> copyState(statePtr, states_[currIndex]);

                foundSample = 
                    InformedSampler::opt_ -> isCostBetterThan(InformedSampler::heuristicSolnCost(statePtr), maxCost);
            }

            return foundSample;
        }


        bool RejectionInfPrecomputedSampler::sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost)
        {
            bool foundSample = false;

            for(; currIndex < numStates && foundSample == false; currIndex++)
            {
                foundSample = sampleUniform(statePtr, maxCost);

                if (foundSample == true)
                {
                    Cost sampledCost = InformedSampler::heuristicSolnCost(statePtr);

                    foundSample = InformedSampler::opt_->isCostEquivalentTo(minCost, sampledCost) ||
                                  InformedSampler::opt_->isCostBetterThan(minCost, sampledCost);
                }
            }

            return foundSample;
        }

        double RejectionInfPrecomputedSampler::getInformedMeasure(const Cost & /*currentCost*/) const
        {
            return InformedSampler::space_->getMeasure();
        }

        double RejectionInfPrecomputedSampler::getInformedMeasure(const Cost & /*minCost*/, const Cost & /*maxCost*/) const
        {
            return InformedSampler::space_->getMeasure();
        }

        bool RejectionInfPrecomputedSampler::areStatesExhausted()
        {
            return (currIndex >= numStates);
        }

    } // namespace base
} // namespace ompl