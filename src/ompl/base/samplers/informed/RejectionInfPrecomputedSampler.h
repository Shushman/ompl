/* Shushman Choudhury */

#ifndef OMPL_BASE_SAMPLERS_INFORMED_REJECTION_INFORMED_PRECOMPUTED_SAMPLER_
#define OMPL_BASE_SAMPLERS_INFORMED_REJECTION_INFORMED_PRECOMPUTED_SAMPLER_

#include "ompl/base/samplers/InformedStateSampler.h"

namespace ompl
{
    namespace base
    {


        class RejectionInfPrecomputedSampler : public InformedSampler
        {
        public:

            RejectionInfPrecomputedSampler(const ProblemDefinitionPtr &probDefn,
                                           const StateSpace *space, const std::vector<const State *> &states);

            bool sampleUniform(State *statePtr, const Cost &maxCost) override;

            bool sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost) override;

            double getInformedMeasure(const Cost & /*currentCost*/) const override;

            double getInformedMeasure(const Cost & /*minCost*/, const Cost & /*maxCost*/) const override;

            //All states have been sampled
            bool areStatesExhausted();

        protected:

            const StateSpace *space_;

            const std::vector<const State *> &states_;

            // The size of the vector of states
            size_t numStates;

            // The current Index to be sampled 
            size_t currIndex;

        };
    } // namespace base
} // namespace ompl

#endif // OMPL_BASE_SAMPLERS_INFORMED_REJECTION_INFORMED_PRECOMPUTED_SAMPLER_