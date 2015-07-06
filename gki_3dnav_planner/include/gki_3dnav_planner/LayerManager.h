#ifndef GKI_3DNAV_SLICER_H
#define GKI_3DNAV_SLICER_H

#include <gki_3dnav_planner/environment_xyt_3d_collisions.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <vector>
#include <boost>


namespace gki_3dnav_planner
{
  class Layer()
  {
  public:
    boost::shared_ptr<costmap_2d::Costmap2DROS>
  }
  
  class Slicer
  {
  public:

    Slicer();

    ~Slicer() { };

  private:
    std::vector<LayerPtr> layers_;

    
    zRange IndexToZRange(int i);
  };
};

#endif

