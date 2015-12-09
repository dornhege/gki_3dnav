#ifndef FREESPACE_MECHANISM_HEURISTIC_H
#define FREESPACE_MECHANISM_HEURISTIC_H

#include <string>
#include <vector>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>

namespace freespace_mechanism_heuristic
{

class HeuristicCostMap
{
    friend void computeCosts(const EnvNAVXYTHETALATConfig_t & cfg, int theta, HeuristicCostMap & costmaps);

    public:
        /// How are queries outside the costmap answered
        enum OutOfMapBehavior
        {
            OutOfMapMaxCost,        ///< use max value of map
            OutOfMapInfiniteCost,   ///< use INFINITECOST
            OutOfMapAssert,         ///< Assert, i.e. should not happen
        };

        HeuristicCostMap(unsigned int height, unsigned int width, unsigned int numThetaDirs, bool allocateMaps,
                enum OutOfMapBehavior outOfMapBehaviour);
        HeuristicCostMap(const std::string & mapfile, enum OutOfMapBehavior outOfMapBehaviour);
        ~HeuristicCostMap();

        unsigned int getCost(int dx, int dy, int startTheta, int endTheta) const;

        bool loadCostMap(const std::string & mapfile);
        bool saveCostMap(const std::string & mapfile) const;

        /// Save costmap as PPM.
        /**
         * \param [in] endTheta end pose theta for costmap image. If < 0, chooses min cost over all end thetas
         */
        bool saveCostMapImage(const std::string & ppmfile, int startTheta, int endTheta) const;

        void updateMaxCost();

    protected:
        void allocateMaps();
        void deallocateMaps();

        /// Raw access for writing
        unsigned int*** getCostMap(unsigned int theta) { return costmaps_[theta]; }

    protected:
        unsigned int height_;
        unsigned int width_;
        unsigned int numThetaDirs_;
        enum OutOfMapBehavior outOfMapBehaviour_;

        /// One costmap for each theta in numThetaDirs.
        /**
         * Each costmap is array of size height_ x width_ x numThetaDirs for
         * the cost of deltax, deltay, endtheta, where
         * deltax, deltay = 0, 0 is assumed to be at height_/2, width_/2.
         *
         * Each costmap contains the costs to get from (0, 0, start theta) -> each cells (dx, dy, end theta).
         */
        std::vector<unsigned int ***> costmaps_;

        unsigned int maxCost_;
};

}

#endif

