#include "freespace_mechanism_heuristic/freespace_mechanism_heuristic.h"
#include <sbpl/utils/key.h>
#include <stdio.h>
#include <assert.h>
#include <fstream>

namespace freespace_mechanism_heuristic
{

HeuristicCostMap::HeuristicCostMap(unsigned int height, unsigned int width, unsigned int numThetaDirs,
        bool allocate, enum OutOfMapBehavior outOfMapBehaviour) :
    height_(height), width_(width), numThetaDirs_(numThetaDirs), outOfMapBehaviour_(outOfMapBehaviour)
{
    if(allocate) {
        allocateMaps(); 
    }
}

HeuristicCostMap::HeuristicCostMap(const std::string & mapfile, enum OutOfMapBehavior outOfMapBehaviour) :
    outOfMapBehaviour_(outOfMapBehaviour)
{
    if(!loadCostMap(mapfile)) {
        fprintf(stderr, "Loading map from \"%s\" failed.", mapfile.c_str());
        width_ = height_ = numThetaDirs_ = 0;
    }
}

HeuristicCostMap::~HeuristicCostMap()
{
    deallocateMaps();
}

void HeuristicCostMap::allocateMaps()
{
    assert(costmaps_.empty());
    for(unsigned int i = 0; i < numThetaDirs_; ++i) {
        unsigned int*** cost_map = new unsigned int**[width_];
        for(unsigned int x = 0; x < width_; ++x) {
            cost_map[x] = new unsigned int*[height_];
            for(unsigned int y = 0; y < height_; ++y) {
                cost_map[x][y] = new unsigned int[numThetaDirs_];
                for(unsigned int th = 0; th < numThetaDirs_; ++th) {
                    cost_map[x][y][th] = INFINITECOST;
                }
            }
        }
        costmaps_.push_back(cost_map);
    }
}

void HeuristicCostMap::deallocateMaps()
{
    for(unsigned int i = 0; i < costmaps_.size(); ++i) {
        unsigned int*** cost_map = costmaps_.at(i);
        if(cost_map) {
            for(unsigned int x = 0; x < width_; ++x) {
                for(unsigned int y = 0; y < height_; ++y) {
                    delete [] cost_map[x][y];
                }
                delete [] cost_map[x];
            }
            delete [] cost_map;
        }
    }
    costmaps_.clear();
}


bool HeuristicCostMap::loadCostMap(const std::string & mapfile)
{
    std::ifstream f(mapfile.c_str(), std::ios_base::binary);
    if(!f.good()) {
        return false;
    }

    // header infos: sx, sy, num thetas
    f.read(reinterpret_cast<char*>(&width_), sizeof(int));
    f.read(reinterpret_cast<char*>(&height_), sizeof(int));
    f.read(reinterpret_cast<char*>(&numThetaDirs_), sizeof(int));
    // delete/allocate
    deallocateMaps();
    allocateMaps();
    // Then one costmap per start theta
    assert(costmaps_.size() == numThetaDirs_);
    for(unsigned int i = 0; i < costmaps_.size(); ++i) {
        // costmap is costs as int in x, y, endtheta
        for(unsigned int x = 0; x < width_; ++x) {
            for(unsigned int y = 0; y < height_; ++y) {
                for(unsigned int th = 0; th < numThetaDirs_; ++th) {
                    int cost;
                    f.read(reinterpret_cast<char*>(&cost), sizeof(int));
                    costmaps_[i][x][y][th] = cost;
                }
            }
        }
    }
    f.close();

    updateMaxCost();
    return true;
}

bool HeuristicCostMap::saveCostMap(const std::string & mapfile) const
{
    std::ofstream f(mapfile.c_str(), std::ios_base::binary);
    if(!f.good()) {
        return false;
    }

    // header infos: sx, sy, num thetas
    f.write(reinterpret_cast<const char*>(&width_), sizeof(int));
    f.write(reinterpret_cast<const char*>(&height_), sizeof(int));
    f.write(reinterpret_cast<const char*>(&numThetaDirs_), sizeof(int));
    // Then one costmap per start theta
    assert(costmaps_.size() == numThetaDirs_);
    for(unsigned int i = 0; i < costmaps_.size(); ++i) {
        // costmap is costs as int in x, y, endtheta
        for(unsigned int x = 0; x < width_; ++x) {
            for(unsigned int y = 0; y < height_; ++y) {
                for(unsigned int th = 0; th < numThetaDirs_; ++th) {
                    int cost = costmaps_[i][x][y][th];
                    f.write(reinterpret_cast<const char*>(&cost), sizeof(int));
                }
            }
        }
    }
    f.close();
    return true;
}

bool HeuristicCostMap::saveCostMapImage(const std::string & ppmfile, int startTheta, int endTheta) const
{
    std::ofstream f(ppmfile.c_str());
    if(!f.good())
        return false;

    f << "P3 " << width_ << " " << height_ << " 255" << std::endl;

    // image save, i.e.: iterate lines by height and start with "top line", i.e. the last/max index
    for(int y = height_ - 1; y >= 0; --y) {
        for(unsigned int x = 0; x < width_; ++x) {
            unsigned int cost = 255;
            if(endTheta < 0) {
                cost = costmaps_[startTheta][x][y][0];
                for(unsigned int th = 1; th < numThetaDirs_; ++th) {
                    if(costmaps_[startTheta][x][y][th] < cost)
                        cost = costmaps_[startTheta][x][y][th];
                }
            } else {
                cost = costmaps_[startTheta][x][y][endTheta];
            }
            if(cost == INFINITECOST) {
                f << "255 0 255 ";
                continue;
            }
            int c = int(double(cost)/maxCost_ * 240.0);
            //printf("Writing cost: %d as %d for th %d (max %d)\n", cost, c, theta, g_maxCost);
            f << c << " " << c << " " << c << " ";
        }
        f << std::endl;
    }

    f.close();
    return true;
}

void HeuristicCostMap::updateMaxCost()
{
    maxCost_ = 0; 
    for(unsigned int i = 0; i < costmaps_.size(); ++i) {
        for(unsigned int x = 0; x < width_; ++x) {
            for(unsigned int y = 0; y < height_; ++y) {
                for(unsigned int th = 0; th < numThetaDirs_; ++th) {
                    if(costmaps_[i][x][y][th] == INFINITECOST)
                        continue;
                    if(costmaps_[i][x][y][th] > maxCost_)
                        maxCost_ = costmaps_[i][x][y][th];
                }
            }
        }
    }
    if(maxCost_ == 0)
        maxCost_ = 1;
}


unsigned int HeuristicCostMap::getCost(int dx, int dy, int startTheta, int endTheta) const
{
    int ind_x = dx + width_/2;
    int ind_y = dy + height_/2;
    if(ind_x < 0 || ind_x >= width_ || ind_y < 0 || ind_y >= height_
            || startTheta < 0 || startTheta >= numThetaDirs_ || endTheta < 0 || endTheta >= numThetaDirs_) {
        switch(outOfMapBehaviour_) {
            case OutOfMapMaxCost:
                return maxCost_;
            case OutOfMapInfiniteCost:
                return INFINITECOST;
            case OutOfMapAssert:
                assert(ind_x < 0 || ind_x >= width_ || ind_y < 0 || ind_y >= height_
                        || startTheta < 0 || startTheta >= numThetaDirs_ || endTheta < 0 || endTheta >= numThetaDirs_);
                return 0;
        }
    }

    assert(costmaps_.size() == numThetaDirs_);
    return costmaps_[startTheta][ind_x][ind_y][endTheta];
}

}

