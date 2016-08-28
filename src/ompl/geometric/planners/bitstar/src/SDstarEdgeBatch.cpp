//Author : Shushman Choudhury

#include <iostream>
#include <functional>
#include <thread>
#include "ompl/geometric/planners/bitstar/SDstarEdgeBatch.h"
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

            //Commented out as
            //this->prune();

            if(numBatches_ > 1)
            {
                std::vector<VertexPtr> vertsInTree;
                vertexNN_ -> list(vertsInTree);
                for(auto &vert : vertsInTree)
                {   
                    if(vert -> isPruned() == false){
                        vert->markUnexpandedToVertices();
                        vert->markUnexpandedToSamples();
                    }
                }
            }

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
                        //if (Planner::si_->isValid(newState->stateConst()) == true)
                        //{
                            // Add the new state as a sample
                            this->addSample(newState);

                            // Update the number of uniformly distributed states
                            ++numUniformStates_;

                            // Update the number of sample
                            ++numSamples_;
                        //}
                    }
                }
            
                OMPL_INFORM("SDstarEdgeBatch : All states exhausted and %d samples!",numSamples_);
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
                r_ = std::numeric_limits<double>::infinity();;
                prevRadius_ = 0.0;
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
                        prevRadius_ = r_;
                        r_ = std::min(r_ * radInflFactor_,r_max_dynamic);
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
                        prevRadius_ = 0.0;
                        if(this->getIsHaltonSeq())
                            //From Janson-Pavone paper
                            r_ = this->calculateRHalton(N)*2.5;
                        else
                            r_ = this->calculateR(N);
                    }
                }
            }
            if(useKNearest_)
                OMPL_INFORM("Current K is %d",k_);
            else
                OMPL_INFORM("Current Radius is %f",r_);
        }

        /*unsigned int SDstarEdgeBatch::nearestNewRadVertices(const VertexPtr &vertex, std::vector<VertexPtr> *neighbourVertices)
        {
            ++numNearestNeighbours_;

            if (useKNearest_ == true)
            {
                vertexNN_->nearestK(vertex, k_, *neighbourVertices);
                return k_;
            }
            else
            {
                std::vector<VertexPtr> oldNbrs, newNbrs;

                if(prevRadius_ == 0.0)
                {
                    vertexNN_->nearestR(vertex, r_, *neighbourVertices);
                }
                else{
                    //Get old nbrs
                    vertexNN_->nearestR(vertex, prevRadius_, oldNbrs);

                    //Get new nbrs
                    vertexNN_->nearestR(vertex, r_, newNbrs);

                    unsigned int oldSize = oldNbrs.size();
                    neighbourVertices = new std::vector<VertexPtr>(newNbrs.begin() + oldSize,newNbrs.end());

                }
                return 0u;
            }
        }*/

        bool SDstarEdgeBatch::checkEdge(const VertexConstPtrPair &edge)
        {
            ++numEdgeCollisionChecks_;
            VertexIdPair fwdpair = std::make_pair(edge.first->getId(), edge.second->getId());
            VertexIdPair backpair = std::make_pair(edge.second->getId(), edge.first->getId());
            bool res = false;
            if(edgeCheckStatus.find(fwdpair) != edgeCheckStatus.end())
            {
                res = edgeCheckStatus[fwdpair];
            }
            else if(edgeCheckStatus.find(backpair) != edgeCheckStatus.end())
            {
                res = edgeCheckStatus[backpair];
            }
            else
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> start,end;
                start = std::chrono::high_resolution_clock::now();
                double dist = Planner::si_->distance(edge.first->stateConst(), edge.second->stateConst());
                std::this_thread::sleep_for(std::chrono::duration<double>(0.00001*dist));
                res = Planner::si_->checkMotion(edge.first->stateConst(), edge.second->stateConst());
                end = std::chrono::high_resolution_clock::now();
                collcheck_time += static_cast< std::chrono::duration<double> >(end-start);
                edgeCheckStatus.insert(std::make_pair(std::make_pair(edge.first->getId(), edge.second->getId()),res));
            }
            
            return res;
        }


    } // namespace geometric
} // namespace ompl
