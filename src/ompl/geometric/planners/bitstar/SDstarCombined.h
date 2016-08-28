//Author : Shushman Choudhury

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARCOMBINED_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARCOMBINED_


#include <string>
#include <utility>
#include <vector>
#include <list>

#include "ompl/base/Planner.h"

#include "ompl/geometric/planners/bitstar/datastructures/IdGenerator.h"
#include "ompl/geometric/planners/bitstar/datastructures/IntegratedQueue.h"
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"
#include "ompl/geometric/planners/bitstar/SDstarBase.h"

//  SDstarCombined
// BIT* on a fixed set of samples
// Won't necessarily converge to optimum

namespace ompl
{
    namespace geometric
    {
        class SDstarCombined : public ompl::geometric::SDstarBase
        {
        public:

            SDstarCombined(const base::SpaceInformationPtr &si, const std::string &name = "SDstarCombined");

            ~SDstarCombined() override;

            void setInitVertexSize(unsigned int initVertexSize);

            unsigned int getInitVertexSize() const;

            void setVertexInflationFactor(double vertInflFactor);

            double getVertexInflationFactor() const;

        protected:

            void updateSamples() override;

            void updateNearestTerms() override;

            void iterate() override;

            bool checkEdge(const VertexConstPtrPair &edge) override;

            unsigned int initVertexSize_;

            unsigned int nextVertexTarget_;

            double vertInflFactor_;

        };
    }
}

#endif //OMPL_GEOMETRIC_PLANNERS_BITSTAR_SDSTARCOMBINED_