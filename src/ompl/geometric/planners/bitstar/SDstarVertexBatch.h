//Author : Shushman Choudhury

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARVERTEXBATCH_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARVERTEXBATCH_


#include <string>
#include <utility>
#include <vector>
#include <list>

#include "ompl/base/Planner.h"

#include "ompl/geometric/planners/bitstar/datastructures/IdGenerator.h"
#include "ompl/geometric/planners/bitstar/datastructures/IntegratedQueue.h"
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"
#include "ompl/geometric/planners/bitstar/SDstarBase.h"

// SDstarVertexBatch
// Adds a subset of vertices and searches the fully connected subgraph each time
// Needs additional membvars for number of nodes to begin with
// and inflation factor for nodes

namespace ompl
{
    namespace geometric
    {
        class SDstarVertexBatch : public ompl::geometric::SDstarBase
        {
        public:

            //Constructor
            SDstarVertexBatch(const base::SpaceInformationPtr &si, const std::string &name = "SDstarVertexBatch");

            ~SDstarVertexBatch() override;

            void setInitVertexSize(unsigned int initVertexSize);

            unsigned int getInitVertexSize() const;

            void setVertexInflationFactor(double vertInflFactor);

            double getVertexInflationFactor() const;

        protected:

            void newBatch() override;

            void updateSamples() override;

            void updateNearestTerms() override;

            unsigned int initVertexSize_;

            unsigned int nextVertexTarget_;

            double vertInflFactor_;

        };
    } // namespace geometric
} // namespace ompl

#endif// OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARVERTEXBATCH_