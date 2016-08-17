//Author : Shushman Choudhury

#include "ompl/geometric/planners/bitstar/SDstarEdgeBatch.h"
#include <iostream>
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/base/samplers/informed/RejectionInfPrecomputedSampler.h"

namespace ompl
{
    namespace geometric
    {
        SDstarEdgeBatch::SDstarEdgeBatch(const ompl::base::SpaceInformationPtr &si, const std::string &name)
        : ompl::geometric::SDstarBase(si,name),
          radInflFactor_(0.0)
        {
        }

        SDstarEdgeBatch::~SDstarEdgeBatch() = default;

        void SDstarEdgeBatch::setRadiusInflationFactor(double radInflFactor)
        {
            radInflFactor_ = radInflFactor;
        }

        double SDstarEdgeBatch::getRadiusInflationFactor() const
        {
            return radInflFactor_;
        }


        void SDstarEdgeBatch::newBatch()
        {
            OMPL_INFORM("New Batch called!");

            ++numBatches_;

            intQueue_->reset();

            if (Planner::pis_.haveMoreStartStates() == true || Planner::pis_.haveMoreGoalStates() == true)
            {
                // There are new starts/goals to get.
                this->updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition());
            }


            this->prune();

            
            if(numBatches_ > 1)
            {
                std::vector<VertexPtr> vertsInTree;
                vertexNN_ -> list(vertsInTree);
                for(auto &vert : vertsInTree)
                {
                    vert->markUnexpandedToVertices();
                }
            }
            
            /*
            std::cout<<"VertexNN size is "<<vertexNN_->size()<<std::endl;
            std::cout<<"freeStateNN size is "<<freeStateNN_->size()<<std::endl;
            std::cout<<"Integrated Queue has "<<intQueue_->numEdges()<<" edges and "<<intQueue_->numVertices()<<" vertices"<<std::endl;
            */

            //int a;
            //std::cin >> a;
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

        void SDstarEdgeBatch::updateSamples()
        {
            if(numBatches_ > 1)
            {
                return;
            }

            ompl::base::Cost costReqd = bestCost_;

            if (opt_->isCostBetterThan(costSampled_, costReqd))
            {
                while (sampler_ -> areStatesExhausted() == false)
                {
                    VertexPtr newState = std::make_shared<BITstar::Vertex>(Planner::si_, opt_);

                    bool sample_res = sampler_->sampleUniform(newState->state(), costReqd);

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
            
                OMPL_INFORM("SDstarNoBatch : All states exhausted and %d samples!",numSamples_);
                costSampled_ = costReqd;
            }
        }


        void SDstarEdgeBatch::updateNearestTerms()
        {
            unsigned int N = numTotalSamples_;

            //Be lazy
            if (N == 0u)
            {
                k_ = startVertices_.size() + goalVertices_.size();
                r_ = std::numeric_limits<double>::infinity();
            }
            else
            {
                if(numBatches_ > 1)
                {
                    if (useKNearest_ == true)
                    {
                        k_ = k_ * radInflFactor_;
                    }
                    else
                    {
                        r_ = r_ * radInflFactor_;
                    }
                }
                else
                {
                    if (useKNearest_ == true)
                    {
                        k_ = this->calculateK(N);
                    }
                    else
                    {
                        r_ = this->calculateR(N);
                    }
                }
            }
            if(useKNearest_)
                OMPL_INFORM("Current K is %d",k_);
            else
                OMPL_INFORM("Current Radius is %f",r_);
        }

    } // namespace geometric
} // namespace ompl