//Author : Shushman Choudhury

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARNOBATCH_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARNOBATCH_


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

        class SDstarNoBatch : public ompl::geometric::SDstarBase
        {
        public:

            //Constructor

            SDstarNoBatch(const base::SpaceInformationPtr &si, const std::string &name = "SDstarNoBatch");

            ~SDstarNoBatch() override;


        protected:

            //void addSample(const VertexPtr &newSample) override;

            //void addVertex(const VertexPtr &newVertex, const bool &removeFromFree) override; 

            void newBatch() override;

            void updateSamples() override;

            void updateNearestTerms() override;

            unsigned int nearestSamples(const VertexPtr &vertex, std::vector<VertexPtr> *neighbourSamples);

            unsigned int nearestVertices(const VertexPtr &vertex, std::vector<VertexPtr> *neighbourVertices);

        };
    } // namespace geometric
} // namespace ompl

#endif //OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARNOBATCH_