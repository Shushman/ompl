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

// SDstarNoBatch
// Does not do any vertex or edge batching. Adds all vertices 



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

            void updateSamples() override;

            void updateNearestTerms() override;

            double radInflFactor_;

        };
    } // namespace geometric
} // namespace ompl

#endif //OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARNOBATCH_