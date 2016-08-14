//Author : Shushman Choudhury

#include "ompl/geometric/planners/bitstar/SDstarNoBatch.h"

#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/base/samplers/informed/RejectionInfPrecomputedSampler.h"

namespace ompl
{
    namespace geometric
    {
        SDstarNoBatch::SDstarNoBatch(const ompl::base::SpaceInformationPtr &si, const std::string &name)
        : ompl::geometric::SDstarBase(si,name)
        {
        }

        SDstarNoBatch::~SDstarNoBatch() = default;

        void SDstarNoBatch::newBatch()
        {
            if(numBatches_ >= 1)
            {
                return;
            }

            ++numBatches_;

            intQueue_->reset();

            if (Planner::pis_.haveMoreStartStates() == true || Planner::pis_.haveMoreGoalStates() == true)
            {
                // There are new starts/goals to get.
                this->updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition());
            }

            this->prune();

            costSampled_ = minCost_;

            this -> updateNearestTerms();

            for (auto &newSample : newSamples_)
            {
                // If the sample still exists, mark as old. It can get pruned during a resort.
                if (newSample->isPruned() == false)
                {
                    newSample->markOld();
                }
                // No else, this sample has been pruned and will shortly disappear
            }

            // Clear the list of new samples
            newSamples_.clear();

            // Make the recycled vertices to new:
            newSamples_ = recycledSamples_;

            // Clear the list of recycled
            recycledSamples_.clear();

            this -> updateSamples();

        }


        void SDstarNoBatch::updateSamples()
        {
            ompl::base::Cost costReqd = bestCost_;

            if (opt_->isCostBetterThan(costSampled_, costReqd))
            {
                while (sampler_ -> areStatesExhausted() == false)
                {
                    VertexPtr newState = std::make_shared<BITstar::Vertex>(Planner::si_, opt_);

                    bool sample_res = sampler_->sampleUniform(newState->state(), costSampled_, costReqd);

                    if(sample_res)
                    {
                        ++numStateCollisionChecks_;
                        if (Planner::si_->isValid(newState->stateConst()) == true)
                        {
                            // Add the new state as a sample
                            this->addSample(newState);

                            // Update the number of uniformly distributed states
                            ++numUniformStates_;

                            // Update the number of sample
                            ++numSamples_;
                        }
                    }
                }

                OMPL_INFORM("SDstarNoBatch : All states exhausted!");
                costSampled_ = costReqd;
            }
        }


        void SDstarNoBatch::updateNearestTerms()
        {
            //Should only be called once
            if(numBatches_ >= 1)
            {
                return;
            }

            unsigned int N = numTotalSamples_;

            //Be lazy
            if (N == 0u)
            {
                k_ = startVertices_.size() + goalVertices_.size();
                r_ = std::numeric_limits<double>::infinity();
            }
            else
            {
                if (useKNearest_ == true)
                {
                    //All but self
                    k_ = N - 1;
                }
                else
                {
                    //Infinite since only 1 batch
                    r_ = std::numeric_limits<double>::infinity();
                }
            }
        }

    }
}