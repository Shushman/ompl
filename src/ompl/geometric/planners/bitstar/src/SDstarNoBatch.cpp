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
            setIsUsingStates(false);
        }

        SDstarNoBatch::~SDstarNoBatch() = default;

        void SDstarNoBatch::newBatch()
        {
            if(numBatches_ >= 1)
            {
                return;
            }

            if(isUsingStates_)
            {
                intQueue_->setNearSamplesFunc(std::bind(&SDstarNoBatch::nearestSamples, this, std::placeholders::_1, std::placeholders::_2 ));
                intQueue_->setNearVerticesFunc(std::bind(&SDstarNoBatch::nearestVertices, this, std::placeholders::_1, std::placeholders::_2 ));
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

        unsigned int SDstarNoBatch::nearestVertices(const VertexPtr &vertex, std::vector<VertexPtr> *neighbourVertices)
        {
            ++numNearestNeighbours_;
            
            *neighbourVertices = vertexStates_;
            return 0u;
        }

        unsigned int SDstarNoBatch::nearestSamples(const VertexPtr &vertex, std::vector<VertexPtr> *neighbourSamples)
        {
            std::chrono::time_point<std::chrono::high_resolution_clock> start,end;
            start = std::chrono::high_resolution_clock::now();
            ++numNearestNeighbours_;
            
            *neighbourSamples = freeStates_;
            end = std::chrono::high_resolution_clock::now();
            nbrs_time += static_cast< std::chrono::duration<double> >(end-start);
            return 0u;
        }

        /*
        void SDstarNoBatch::addSample(const VertexPtr &newSample)
        {
            // Mark as new
            newSample->markNew();

            // Add to the list of new samples
            newSamples_.push_back(newSample);

            // Add to the NN structure:
            freeStateNN_->add(newSample);

            //Add to full set
            freeStates_.push_back(newSample);
        }

        void SDstarNoBatch::addVertex(const VertexPtr &newVertex, const bool &removeFromFree)
        {
            // Make sure it's connected first, so that the queue gets updated properly. This is a day of debugging I'll
            // never get back
            if (newVertex->isInTree() == false)
            {
                throw ompl::Exception("Vertices must be connected to the graph before adding");
            }

            // Remove the vertex from the list of samples (if it even existed)
            if (removeFromFree == true)
            {
                freeStateNN_->remove(newVertex);
                for(std::vector<VertexPtr>::iterator it = freeStates_.begin(); it != freeStates_.end(); ++ it)
                {
                    if(*it == newVertex){
                        freeStates_.erase(it);
                        break;
                    }
                }
            }
            // No else

            // Add to the NN structure:
            vertexNN_->add(newVertex);

            vertexStates_.push_back(newVertex);

            // Add to the queue:
            intQueue_->insertVertex(newVertex);

            // Increment the number of vertices added:
            ++numVertices_;
        }
        */

    }
}