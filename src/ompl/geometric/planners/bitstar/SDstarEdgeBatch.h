//Author : Shushman Choudhury

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTAREDGEBATCH_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTAREDGEBATCH_


#include <string>
#include <utility>
#include <vector>
#include <list>

#include "ompl/base/Planner.h"

#include "ompl/geometric/planners/bitstar/datastructures/IdGenerator.h"
#include "ompl/geometric/planners/bitstar/datastructures/IntegratedQueue.h"
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"
#include "ompl/geometric/planners/bitstar/SDstarBase.h"

// SDstarEdgeBatch
// Adds all vertices at start and gradually adds more edges
// based on the radius of connectivity

namespace ompl
{
    namespace geometric
    {

        class SDstarEdgeBatch : public ompl::geometric::SDstarBase
        {
        public:

            //Constructor

            SDstarEdgeBatch(const base::SpaceInformationPtr &si, const std::string &name = "SDstarEdgeBatch");

            ~SDstarEdgeBatch() override;

            void setRadiusInflationFactor(double radInflFactor);

            double getRadiusInflationFactor() const;


        protected:

            void newBatch() override;

            void pruneSamples() override;

            void updateSamples() override;

            void updateStartAndGoalStates(const base::PlannerTerminationCondition &ptc) override;

            void addSample(const VertexPtr &newSample) override;

            void dropSample(const VertexPtr &oldSample) override;

            void updateNearestTerms() override;

            bool checkEdge(const VertexConstPtrPair &edge) override;

            unsigned int nearestNewRadVertices(const VertexPtr &vertex, std::vector<VertexPtr> *neighbourVertices);

            double radInflFactor_;

            double prevRadius_;

            std::vector<VertexPtr> samplesNotPruned_;

        };
    } // namespace geometric
} // namespace ompl

#endif //OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARNOBATCH_