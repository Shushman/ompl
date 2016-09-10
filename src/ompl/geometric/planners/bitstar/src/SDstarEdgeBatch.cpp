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


            if(numBatches_ > 1)
            {
                this->prune();

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

                freeStateNN_ -> add(newSamples_);
            
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

        void SDstarEdgeBatch::updateStartAndGoalStates(const base::PlannerTerminationCondition &ptc)
        {
            // Variable
            // Whether we've added a start or goal:
            bool addedGoal = false;
            bool addedStart = false;
            // Whether we have to rebuid the queue, i.e.. whether we've called updateStartAndGoalStates before
            bool rebuildQueue = false;

            // Add the new starts and goals to the lists of said vertices.
            // Do goals first, as they are only added as samples.
            // We do this as nested conditions so we always call nextGoal(ptc) at least once (regardless of whether
            // there are moreGoalStates or not)
            // in case we have been given a non trivial PTC that wants us to wait, but do *not* call it again if there
            // are no more goals
            //(as in the nontrivial PTC case, doing so would cause us to wait out the ptc and never try to solve
            //anything)
            do
            {
                // Variable
                // A new goal pointer, if there are none, it will be nullptr.
                // We will wait for the duration of PTC for a new goal to appear.
                const ompl::base::State *newGoal = Planner::pis_.nextGoal(ptc);

                // Check if it's valid
                if (static_cast<bool>(newGoal) == true)
                {
                    // It is valid and we are adding a goal, we will need to rebuild the queue if any starts have
                    // previously been added as their (and any descendents') heuristic cost-to-go may change:
                    rebuildQueue = (startVertices_.size() > 0u);

                    // Allocate the vertex pointer
                    goalVertices_.push_back(std::make_shared<BITstar::Vertex>(Planner::si_, opt_));

                    // Copy the value into the state
                    Planner::si_->copyState(goalVertices_.back()->state(), newGoal);

                    // And add this goal to the set of samples:
                    this->addSample(goalVertices_.back());
                    freeStateNN_->add(goalVertices_.back());


                    // Mark that we've added:
                    addedGoal = true;
                }
                // No else, there was no goal.
            } while (Planner::pis_.haveMoreGoalStates() == true);

            // And then do the for starts. We do this last as the starts are added to the queue, which uses a cost-to-go
            // heuristic in it's ordering, and for that we want all the goals updated.
            // As there is no way to wait for new *start* states, this loop can be cleaner
            // There is no need to rebuild the queue when we add start vertices, as the queue is ordered on current
            // cost-to-come, and adding a start doesn't change that.
            while (Planner::pis_.haveMoreStartStates() == true)
            {
                // Variable
                // A new start pointer
                const ompl::base::State *newStart = Planner::pis_.nextStart();

                // Allocate the vertex pointer:
                startVertices_.push_back(std::make_shared<BITstar::Vertex>(Planner::si_, opt_, true));

                // Copy the value into the state:
                Planner::si_->copyState(startVertices_.back()->state(), newStart);

                // Add this start to the queue. It is not a sample, so skip that step:
                this->addVertex(startVertices_.back(), false);

                // Mark that we've added:
                addedStart = true;
            }

            // Now, if we added a new start and have previously pruned goals, we may want to readd them.
            if (addedStart == true && prunedGoalVertices_.empty() == false)
            {
                // Variable
                // An iterator to the list of pruned goals
                auto pgIter = prunedGoalVertices_.begin();

                // Consider each one
                while (pgIter != prunedGoalVertices_.end())
                {
                    // Mark as unpruned
                    (*pgIter)->markUnpruned();

                    // Check if it should be readded (i.e., would it be pruned *now*?)
                    if (intQueue_->vertexPruneCondition(*pgIter) == true)
                    {
                        // It would be pruned, so remark as pruned
                        (*pgIter)->markPruned();

                        // and move onto the next:
                        ++pgIter;
                    }
                    else
                    {
                        // It would not be pruned now, so readd it!
                        // Add back to the list:
                        goalVertices_.push_back(*pgIter);

                        // Add as a sample
                        this->addSample(*pgIter);
                        freeStateNN_->add(*pgIter);


                        // Mark what we've added:
                        addedGoal = true;

                        // Remove the start from the list, this returns the next iterator
                        pgIter = prunedGoalVertices_.erase(pgIter);

                        // Just like the other new goals, we will need to rebuild the queue if any starts have
                        // previously been added. Which was a condition to be here in the first place
                        rebuildQueue = true;
                    }
                }
            }

            // Now, if we added a goal and have previously pruned starts, we will have to do the same on those
            if (addedGoal == true && prunedStartVertices_.empty() == false)
            {
                // Variable
                // An iterator to the list of pruned starts
                auto psIter = prunedStartVertices_.begin();

                // Consider each one
                while (psIter != prunedStartVertices_.end())
                {
                    // Mark as unpruned
                    (*psIter)->markUnpruned();

                    // Check if it should be readded (i.e., would it be pruned *now*?)
                    if (intQueue_->vertexPruneCondition(*psIter) == true)
                    {
                        // It would be pruned, so remark as pruned
                        (*psIter)->markPruned();

                        // and move onto the next:
                        ++psIter;
                    }
                    else
                    {
                        // It would not be pruned, readd it!
                        // Add it back to the list
                        startVertices_.push_back(*psIter);

                        // Add to the queue as a vertex. It is not a sample, so skip that step:
                        this->addVertex(*psIter, false);

                        // Mark what we've added:
                        addedStart = true;

                        // Remove the start from the list, this returns the next iterator
                        psIter = prunedStartVertices_.erase(psIter);
                    }
                }
            }

            // If we've added a state, we have some updating to do.
            if (addedGoal == true || addedStart == true)
            {
                // Update the minimum cost
                for (std::list<VertexPtr>::const_iterator sIter = startVertices_.begin(); sIter != startVertices_.end();
                     ++sIter)
                {
                    // Take the better of the min cost so far and the cost-to-go from this start
                    minCost_ = opt_->betterCost(minCost_, this->costToGoHeuristic(*sIter));
                }

                // If we have at least one start and goal, allocate a sampler
                /*if (startVertices_.size() > 0u && goalVertices_.size() > 0u)
                {
                    // There is a start and goal, allocate
                    //sampler_ =
                        //opt_->allocInformedStateSampler(Planner::pdef_, std::numeric_limits<unsigned int>::max());
                      sampler_ = std::make_shared< ompl::base::RejectionInfPrecomputedSampler >(Planner::pdef_, Planner::si_->getStateSpace(), states_);
                }*/
                // No else, this will get allocated when we get the updated start/goal.

                // Was there an existing queue that needs to be rebuilt?
                if (rebuildQueue == true)
                {
                    // There was, inform
                    OMPL_INFORM("%s: Updating starts/goals and rebuilding the queue.", Planner::getName().c_str());

                    // Flag the queue as unsorted downstream from every existing start.
                    for (std::list<VertexPtr>::const_iterator sIter = startVertices_.begin();
                         sIter != startVertices_.end(); ++sIter)
                    {
                        intQueue_->markVertexUnsorted(*sIter);
                    }

                    // Resort the queue.
                    this->resort();
                }
                // No else
            }
            // No else, why were we called?

            // Make sure that if we have a goal, we also have a start, since there's no way to wait for more *starts*
            if (goalVertices_.empty() == false && startVertices_.empty() == true)
            {
                OMPL_WARN("%s, The problem has a goal but not a start. As PlannerInputStates provides no method to "
                          "wait for a _start_ state, this will likely be problematic.",
                          Planner::getName().c_str());
            }
            // No else
        }

        bool SDstarEdgeBatch::checkEdge(const VertexConstPtrPair &edge)
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


        void SDstarEdgeBatch::addSample(const VertexPtr &newSample)
        {
            newSample->markNew();

            newSamples_.push_back(newSample);

            //freeStateNN_->add(newSample);
        }


        void SDstarEdgeBatch::pruneSamples()
        {
            samplesNotPruned_.clear();
            // Are we dropping samples anytime we prune?
            if (dropSamplesOnPrune_ == true)
            {
                // We are, update the pruned counter
                numFreeStatesPruned_ = numFreeStatesPruned_ + freeStateNN_->size();

                // and the number of uniform samples
                numUniformStates_ = 0u;

                // Then remove all of the samples
                freeStateNN_->clear();
            }
            else
            {
                // Variable:
                // The list of samples:
                std::vector<VertexPtr> samples;

                // Get the list of samples
                freeStateNN_->list(samples);

                // Iterate through the list and remove any samples that have a heuristic larger than the bestCost_
                for (auto &sample : samples)
                {
                    // Check if this state should be pruned:
                    if (intQueue_->samplePruneCondition(sample) == true)
                    {
                        // Yes, remove it
                        this->dropSample(sample);
                    }
                    else
                    {
                        samplesNotPruned_.push_back(sample);
                    }
                    
                    // No else, keep.
                }
            }

            //Rebuild free State NN
            freeStateNN_ -> clear();
            freeStateNN_ -> add(samplesNotPruned_);


        }

        void SDstarEdgeBatch::dropSample(const VertexPtr &oldSample)
        {
            // Update the counter:
            ++numFreeStatesPruned_;

            // Mark the sample as pruned
            oldSample->markPruned();
        }


    } // namespace geometric
} // namespace ompl
