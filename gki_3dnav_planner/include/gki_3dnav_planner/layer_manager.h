/*
 * Copyright (c) 2015, Philipp Jankov
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef GKI_3DNAV_LAYER_MANAGER_H
#define GKI_3DNAV_LAYER_MANAGER_H

#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <map>
#include <vector>

namespace gki_3dnav_planner
{
  class ZRange
  {
  public:
    ZRange(const ZRange &range);
    ZRange(double from, double to);
    ~ZRange(){};
    double from() const {return from_;};
    double to() const { return to_;};

    bool operator<( const ZRange& other) const {
        return (from_ == other.from_) ? to_ < other.to_ : from_ < other.from_;
    };

  private:
    double from_;
    double to_;
  };

  typedef boost::shared_ptr< ::gki_3dnav_planner::ZRange > ZRangePtr;
  typedef boost::shared_ptr< ::gki_3dnav_planner::ZRange const > ZRangeConstPtr;

  class LayerManager
  {
  public:

    LayerManager();
    ~LayerManager();

    ZRangePtr ZToZRange(double z);
    void InCollision(robot_state::RobotState& rstate);
    void InCollision(robot_state::RobotStatePtr rstate);
    void Update(planning_scene::PlanningScenePtr ps);

  private:
    std::vector<ZRangePtr> z_ranges_;
    /*
     * The map containing the slice paired with its respective z-Range representing the world
     */
    std::map<ZRange, costmap_2d::Costmap2DROS> world_layers_;
    // std::vector<boost::shared_ptr<costmap_2d::Costmap2D> > world_layers_;
    // entkoppeln von world_layers_ und ZRange

    /*
     * The map containing the slice paired with its respective z-Range representing the robot
     */
    std::map<ZRange, costmap_2d::Costmap2DROS> robot_layers_;
    /*
     * The map containing locations of obstacles that are presents in all layers possibly traversed by the robot
     */
    std::map<ZRange, costmap_2d::Costmap2DROS> lethal_layer_;
  };

  typedef boost::shared_ptr< ::gki_3dnav_planner::LayerManager > LayerManagerPtr;
  typedef boost::shared_ptr< ::gki_3dnav_planner::LayerManager const > LayerManagerConstPtr;

  class NotImplementedException : public std::logic_error
  {
  public:
    NotImplementedException() : std::logic_error("Function not yet implemented.") {}
  };
};

#endif

