//Author : Shushman Choudhury
#include <thread>
#include <iostream>
#include <functional>
#include <algorithm>
#include "ompl/geometric/planners/bitstar/SDstarCombined.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/base/samplers/informed/RejectionInfPrecomputedSampler.h"


namespace ompl
{
    namespace geometric
    {

        SDstarCombined::SDstarCombined(const ompl::base::SpaceInformationPtr &si, const std::string &name)
        : ompl::geometric::SDstarBase(si,name)
          {
          }


        SDstarCombined::~SDstarCombined() = default;


        void SDstarCombined::updateSamples()
        {
            // Variable
            // The required cost to contain the neighbourhood of this vertex:
            ompl::base::Cost costReqd = bestCost_;

            // Check if we need to generate new samples inorder to completely cover the neighbourhood of the vertex
            if (opt_->isCostBetterThan(costSampled_, costReqd))
            {
                // Variable
                // The total number of samples we wish to have.
                unsigned int totalReqdSamples;

                totalReqdSamples = numSamples_ + samplesPerBatch_;
                //}

                // Actually generate the new samples
                while (numSamples_ < totalReqdSamples && sampler_ -> areStatesExhausted() == false)
                {
                    // Variable
                    // The new state:
                    VertexPtr newState = std::make_shared<BITstar::Vertex>(Planner::si_, opt_);

                    // Sample in the interval [costSampled_, costReqd):
                    sampler_->sampleUniform(newState->state(), costSampled_, costReqd);

                    // If the state is collision free, add it to the list of free states
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
                    // No else
                }

                // Mark that we've sampled all cost spaces (This is in preparation for JIT sampling)
                costSampled_ = costReqd;
            }
            OMPL_INFORM("Now %d samples!",numSamples_);
            // No else, the samples are up to date
        }

        void SDstarCombined::updateNearestTerms()
        {
            //Be lazy
            if (numBatches_ == 0)
            {
                k_ = startVertices_.size() + goalVertices_.size();
                r_ = std::numeric_limits<double>::infinity();;
            }
            else
            {
                unsigned int N = std::min(numSamples_+samplesPerBatch_,numTotalSamples_);

                k_ = this->calculateK(N);
                if(this->getIsHaltonSeq())
                    r_ = this->calculateRHalton(N)*9.0;
                else
                    r_ = this->calculateR(N);
            }
            
            OMPL_INFORM("Current Radius is %f",r_);
        }

        bool SDstarCombined::checkEdge(const VertexConstPtrPair &edge)
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

        void SDstarCombined::iterate()
        {
            // Info:
            ++numIterations_;

            // If we're using strict queue ordering, make sure the queues are up to date
            if (useStrictQueueOrdering_ == true)
            {
                // The queues will be resorted if the graph has been rewired.
                this->resort();
            }

            // Is the edge queue empty
            if (intQueue_->isEmpty() == true)
            {
                // Is it also unsorted?
                if (intQueue_->isSorted() == false)
                {
                    // If it is, then we've hit a rare condition where we emptied it without having to sort it, so
                    // address that
                    this->resort();
                }
                else
                {
                    // If not, then we're either just starting the problem, or just finished a batch. Either way, make a
                    // batch of samples and/or edges and fill the queue for the first time:

                    if(sampler_ -> areStatesExhausted() == true)
                    {
                        hasFullySearched_ = true;
                        return;
                    }  

                    this->newBatch();
                    hasFullySearched_ = false;
                }
            }
            else
            {
                // If the edge queue is not empty, then there is work to do!

                // Variables:
                // The current edge:
                VertexPtrPair bestEdge;

                // Pop the minimum edge
                ++numEdgesProcessed_;
                intQueue_->popFrontEdge(&bestEdge);

                // In the best case, can this edge improve our solution given the current graph?
                // g_t(v) + c_hat(v,x) + h_hat(x) < g_t(x_g)
                if (opt_->isCostBetterThan(this->combineCosts(bestEdge.first->getCost(),
                                                              this->edgeCostHeuristic(bestEdge),
                                                              this->costToGoHeuristic(bestEdge.second)),
                                           bestCost_) == true)
                {
                    // Variables:
                    // The true cost of the edge:
                    ompl::base::Cost trueEdgeCost;

                    // Get the true cost of the edge
                    trueEdgeCost = this->trueEdgeCost(bestEdge);

                    // Can this actual edge ever improve our solution?
                    // g_hat(v) + c(v,x) + h_hat(x) < g_t(x_g)
                    if (opt_->isCostBetterThan(this->combineCosts(this->costToComeHeuristic(bestEdge.first),
                                                                  trueEdgeCost,
                                                                  this->costToGoHeuristic(bestEdge.second)),
                                               bestCost_) == true)
                    {
                        // Does this edge have a collision?
                        if (this->checkEdge(bestEdge) == true)
                        {
                            // Does the current edge improve our graph?
                            // g_t(v) + c(v,x) < g_t(x)
                            if (opt_->isCostBetterThan(opt_->combineCosts(bestEdge.first->getCost(), trueEdgeCost),
                                                       bestEdge.second->getCost()) == true)
                            {
                                // YAAAAH. Add the edge! Allowing for the sample to be removed from free if it is not
                                // currently connected and otherwise propagate cost updates to descendants.
                                // addEdge will update the queue and handle the extra work that occurs if this edge
                                // improves the solution.
                                this->addEdge(bestEdge, trueEdgeCost, true, true);

                                // Prune the edge queue of any unnecessary incoming edges
                                intQueue_->pruneEdgesTo(bestEdge.second);

                                // We will only prune the whole graph/samples on a new batch.
                            }
                            // No else, this edge may be useful at some later date.
                        }
                        // No else, we failed
                    }
                    // No else, we failed
                }
                else if (intQueue_->isSorted() == false)
                {
                    // The edge cannot improve our solution, but the queue is imperfectly sorted, so we must resort
                    // before we give up.
                    this->resort();
                }
                else
                {
                    // Else, I cannot improve the current solution, and as the queue is perfectly sorted and I am the
                    // best edge, no one can improve the current solution . Give up on the batch:
                    intQueue_->finish();
                }
            }  // Integrated queue not empty.

            
        }
    }
}