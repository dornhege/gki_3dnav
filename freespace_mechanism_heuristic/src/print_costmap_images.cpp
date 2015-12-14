#include "freespace_mechanism_heuristic/freespace_mechanism_heuristic.h"
#include <stdio.h>
#include <sstream>
#include <iomanip>

int main(int argc, char** argv)
{
    if(argc != 2) {
        fprintf(stderr, "No costmaps filename given\n");
        return 1;
    }

    std::string fname = argv[1];

    freespace_mechanism_heuristic::HeuristicCostMap costmap(fname, freespace_mechanism_heuristic::HeuristicCostMap::OutOfMapAssert);

    for(int sth = 0; sth < costmap.getNumThetaDirs(); ++sth) {
        for(int eth = 0; eth < costmap.getNumThetaDirs(); ++eth) {
            std::stringstream ss;
            ss << "costmap_" << std::setfill('0') << std::setw(2) << sth << "_" << std::setw(2) << eth << ".ppm";
            costmap.saveCostMapImage(ss.str(), sth, eth);
        }
        std::stringstream ss;
        ss << "costmap_" << std::setfill('0') << std::setw(2) << sth << "_bestTh.ppm";
        costmap.saveCostMapImage(ss.str(), sth, -1);
    }

    return 0;
}

