//Author : Shushman Choudhury
#include <thread>
#include <iostream>
#include <functional>
#include <algorithm>
#include "ompl/geometric/planners/bitstar/SDstarHybridBatch.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/base/samplers/informed/RejectionInfPrecomputedSampler.h"

namespace ompl
{
    namespace geometric
    {

        SDstarHybridBatch::SDstarHybridBatch(const ompl::base::SpaceInformationPtr &si, const std::string &name)
        : ompl::geometric::SDstarBase(si,name),
          initVertexSize_(0u),
          vertInflFactor_(0.0),
          nextVertexTarget_(0u),
          radInflFactor_(0.0),
          currMode_(VERTEX)
          {
          }

        SDstarHybridBatch::~SDstarHybridBatch() = default;

        void SDstarHybridBatch::setInitVertexSize(unsigned int initVertexSize)
        {
            initVertexSize_ = initVertexSize;
        }

        unsigned int SDstarHybridBatch::getInitVertexSize() const
        {
            return initVertexSize_;
        }

        void SDstarHybridBatch::setVertexInflationFactor(double vertInflFactor)
        {
            vertInflFactor_ = vertInflFactor;
        }

        double SDstarHybridBatch::getVertexInflationFactor() const
        {
            return vertInflFactor_;
        }

        void SDstarHybridBatch::setRadiusInflationFactor(double radInflFactor)
        {
            radInflFactor_ = radInflFactor;
        }

        double SDstarHybridBatch::getRadiusInflationFactor() const
        {
            return radInflFactor_;
        }

        void SDstarHybridBatch::newBatch()
        {
            OMPL_INFORM("New Batch called!");

            ++numBatches_;

            intQueue_->reset();

            if (Planner::pis_.haveMoreStartStates() == true || Planner::pis_.haveMoreGoalStates() == true)
            {
                // There are new starts/goals to get.
                this->updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition());
            }

            this -> prune();

            //Do we need? (YES)
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

            //int a;
            //std::cin>>a;

        }


        void SDstarHybridBatch::updateSamples()
        {

            if(currMode_ == EDGE){
                return;
            }

            ompl::base::Cost costReqd = bestCost_;


            if (opt_->isCostBetterThan(costSampled_, costReqd))
            {
                while (numSamples_ < nextVertexTarget_ && sampler_ -> areStatesExhausted() == false)
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
                std::cout<<"Now "<<numSamples_<<" samples !"<<std::endl;
                if(sampler_ -> areStatesExhausted()){
                    OMPL_INFORM("SDstarHybridBatch : All states exhausted and %d samples!",numSamples_);
                    currMode_ = EDGE;
                }
                costSampled_ = costReqd;
            }
        }

        void SDstarHybridBatch::updateNearestTerms()
        {
            unsigned int N = numTotalSamples_;
            if (N == 0u)
            {
                k_ = startVertices_.size() + goalVertices_.size();
                r_ = std::numeric_limits<double>::infinity();
            }
            else
            {
                if(currMode_ == VERTEX)
                {
                    //std::cout<<"Inside VERTEX mode!"<<std::endl;
                    if(numBatches_ == 0u)
                    {
                        r_ = std::numeric_limits<double>::infinity();
                    }
                    else if(numBatches_ == 1)
                    {
                        nextVertexTarget_ = initVertexSize_;
                    }
                    else
                    {
                        nextVertexTarget_ = std::min(numTotalSamples_,static_cast<unsigned int>(nextVertexTarget_ * vertInflFactor_));
                    }

                    if(this->getIsHaltonSeq())
                        r_ = this->calculateRHalton(nextVertexTarget_)*2.5;
                    else
                        r_ = this->calculateR(nextVertexTarget_);
                }

                else
                {
                    //AFTER 1st search with full samples and radius
                    r_ = std::min(r_ * radInflFactor_,r_max_dynamic);
                }

            }
            

        }

        bool SDstarHybridBatch::checkEdge(const VertexConstPtrPair &edge)
        {
            
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
                ++numEdgeCollisionChecks_;
                std::chrono::time_point<std::chrono::high_resolution_clock> start,end;
                start = std::chrono::high_resolution_clock::now();
                res = Planner::si_->checkMotion(edge.first->stateConst(), edge.second->stateConst());
                end = std::chrono::high_resolution_clock::now();
                collcheck_time += static_cast< std::chrono::duration<double> >(end-start);
                edgeCheckStatus.insert(std::make_pair(std::make_pair(edge.first->getId(), edge.second->getId()),res));
            }
            
            return res;
        }
    }
}