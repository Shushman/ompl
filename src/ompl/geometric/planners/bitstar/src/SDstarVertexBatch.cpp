//Author : Shushman Choudhury
#include <thread>
#include <iostream>
#include <functional>
#include <algorithm>
#include "ompl/geometric/planners/bitstar/SDstarVertexBatch.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/base/samplers/informed/RejectionInfPrecomputedSampler.h"

namespace ompl
{
    namespace geometric
    {
        SDstarVertexBatch::SDstarVertexBatch(const ompl::base::SpaceInformationPtr &si, const std::string &name)
        : ompl::geometric::SDstarBase(si,name),
          initVertexSize_(0u),
          vertInflFactor_(0.0),
          nextVertexTarget_(0u)
          {
          }

        SDstarVertexBatch::~SDstarVertexBatch() = default;

        void SDstarVertexBatch::setInitVertexSize(unsigned int initVertexSize)
        {
            initVertexSize_ = initVertexSize;
        }

        unsigned int SDstarVertexBatch::getInitVertexSize() const
        {
            return initVertexSize_;
        }

        void SDstarVertexBatch::setVertexInflationFactor(double vertInflFactor)
        {
            vertInflFactor_ = vertInflFactor;
        }

        double SDstarVertexBatch::getVertexInflationFactor() const
        {
            return vertInflFactor_;
        }

        void SDstarVertexBatch::newBatch()
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

        }

        void SDstarVertexBatch::updateSamples()
        {
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
                
                if(sampler_ -> areStatesExhausted())
                    OMPL_INFORM("SDstarVertexBatch : All states exhausted and %d samples!",numSamples_);
                costSampled_ = costReqd;
            }
        }


        void SDstarVertexBatch::updateNearestTerms()
        {
            unsigned int N = numTotalSamples_;

            if (N == 0u)
            {
                k_ = startVertices_.size() + goalVertices_.size();
                r_ = std::numeric_limits<double>::infinity();
            }
            else
            {
                if(numBatches_ == 1)
                {
                    nextVertexTarget_ = initVertexSize_;
                }
                else
                {
                    nextVertexTarget_ = static_cast<unsigned int>(nextVertexTarget_ * vertInflFactor_);
                }

                //Fully connected
                k_ = nextVertexTarget_;
                r_ = std::numeric_limits<double>::infinity();

            }
            
        }

        bool SDstarVertexBatch::checkEdge(const VertexConstPtrPair &edge)
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
            
                ompl::base::State* st1 = Planner::si_ -> allocState();
                Planner::si_ -> copyState(st1, edge.first->stateConst());
                ompl::base::State* st2 = Planner::si_ -> allocState();
                Planner::si_ -> copyState(st2, edge.second->stateConst());


                double* vals1 = st1->as<
                            ompl::base::RealVectorStateSpace::StateType>()->values;
                double* vals2 = st2->as<
                            ompl::base::RealVectorStateSpace::StateType>()->values;
                cv::Point start_point((int)(vals1[0]*1000),(int)(vals1[1]*1000));
                cv::Point end_point((int)(vals2[0]*1000),(int)(vals2[1]*1000));

                edgeCheckPoints.push_back(std::make_pair(start_point,end_point));

                Planner::si_ -> freeState(st1);
                Planner::si_ -> freeState(st2); 

                if(numEdgeCollisionChecks_ % 500 == 0)
                {
                    //std::cout<<"NOW EDGE CHECKS IS "<<numEdgeCollisionChecks_<<std::endl;
                    std::stringstream oss;
                    oss<<"vertexbatch_checks_"<<numEdgeCollisionChecks_<<".png";
                    std::string imName(oss.str());

                    getDebugImage(imName);
                }

            }
            
            return res;
        }


    } //namespace geometric
}//namespace ompl