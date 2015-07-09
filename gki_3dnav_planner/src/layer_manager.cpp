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

#include <gki_3dnav_planner/layer_manager.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

namespace gki_3dnav_planner
{
  /************************************************************************************
   * ZRange
   ************************************************************************************/
  ZRange::ZRange(const ZRange &range)
  {
    from_ = range.from_;
    to_ = range.to_;
  }
  
  ZRange::ZRange(double from, double to)
  {
    from_ = from;
    to_ = to;
  }

  /************************************************************************************
   * LayerManager
   ************************************************************************************/
  LayerManager::LayerManager()
  {
        ROS_INFO_STREAM("LM Construted");
  }

  LayerManager::~LayerManager()
  {
    throw new NotImplementedException();    
  }

  ZRangePtr LayerManager::ZToZRange(double z)
  {
    forEach(ZRangePtr range, z_ranges_)
      if (z >= range->from() && z <= range->to()) return boost::shared_ptr<ZRange>(new ZRange(*range));
    return boost::shared_ptr<ZRange>();
  }

  void LayerManager::InCollision(robot_state::RobotState& rstate)
  {
    throw new NotImplementedException();
  }
  
  void LayerManager::InCollision(robot_state::RobotStatePtr rstate)
  {
    throw new NotImplementedException();
  }

  void LayerManager::Update(planning_scene::PlanningScenePtr ps)
  {
    ROS_DEBUG_STREAM("Updating layers using current PlanningScene");
        //ROS_DEBUG_STREAM("z_ranges size: " << z_ranges_.size());
    // Create robot slices and use the genereated ZRanges to setup the worldlayers empty and fill it with information from the world octree
    collision_detection::World::ObjectConstPtr obj = ps->getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
    const shapes::OcTree *ot_shape = dynamic_cast<const shapes::OcTree*>(obj->shapes_[0].get());
    if (ot_shape)
    {
      boost::shared_ptr<const octomap::OcTree> octree = ot_shape->octree;
      for(octomap::OcTree::leaf_iterator it_leaf = octree->begin_leafs(), end_leaf = octree->end_leafs(); it_leaf!= end_leaf; ++it_leaf)
      {
        ZRangePtr range = ZToZRange(it_leaf.getZ());
          ROS_DEBUG_STREAM("Node value: " << it_leaf->getValue() << " with Z " << it_leaf.getZ()); 
        if(range)
        {
          costmap_2d::Costmap2DROS &cmap = world_layers_.at(*range);
          // update cmap cell to 1 but how?
        }         
      }
    }

  }
}
