//Author : Shushman Choudhury

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARHYBRIDBATCH_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARHYBRIDBATCH_

#include <string>
#include <utility>
#include <vector>
#include <list>

#include "ompl/base/Planner.h"

#include "ompl/geometric/planners/bitstar/datastructures/IdGenerator.h"
#include "ompl/geometric/planners/bitstar/datastructures/IntegratedQueue.h"
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"
#include "ompl/geometric/planners/bitstar/SDstarBase.h"

// SDstarHybridBatch
// Does vertex batching with |E| = O(|V|) till N
// Then does edge batching

namespace ompl
{
    namespace geometric
    {
        class SDstarHybridBatch : public ompl::geometric::SDstarBase
        {
        public:

            SDstarHybridBatch(const base::SpaceInformationPtr &si, const std::string &name = "SDstarHybridBatch");

            ~SDstarHybridBatch() override;

            void setInitVertexSize(unsigned int initVertexSize);

            unsigned int getInitVertexSize() const;

            void setVertexInflationFactor(double vertInflFactor);

            double getVertexInflationFactor() const;

            void setRadiusInflationFactor(double radInflFactor);

            double getRadiusInflationFactor() const;

        protected:

            enum Mode
            {
                VERTEX,
                EDGE,
            };

            void newBatch() override;

            void updateSamples() override;

            void updateNearestTerms() override;

            bool checkEdge(const VertexConstPtrPair &edge) override;

            unsigned int initVertexSize_;

            unsigned int nextVertexTarget_;

            double vertInflFactor_;

            double radInflFactor_;

            Mode currMode_;
        };
    }//namespace geometric
}//namespace ompl

#endif //OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARHYBRIDBATCH_