/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  Copyright (c) 2015, Philipp Jankov
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Philipp Jankov*/
/* Author: Ioan Sucan */

#ifndef GKI_3DNAV_VISUALIZATION_SLICE_DISPLAY_RVIZ_SLICE_DISPLAY_
#define GKI_3DNAV_VISUALIZATION_SLICE_DISPLAY_RVIZ_SLICE_DISPLAY_

#include <rviz/display.h>

#ifndef Q_MOC_RUN
#include <moveit/rviz_plugin_render_tools/planning_scene_render.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/background_processing/background_processing.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class Robot;
class Property;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class ColorProperty;
class EnumProperty;
}

namespace gki_3dnav_rviz_plugin
{

  class RobotSliceDisplay : public rviz::Display
  {
    Q_OBJECT

    public:
      static const std::string INTERACTIVE_MARKER_TOPIC;
      RobotSliceDisplay(bool listen_to_planning_scene = true,
                          bool show_scene_robot = true);
      virtual ~RobotSliceDisplay();

      virtual void load(const rviz::Config& config);
      virtual void save(rviz::Config config) const;

      virtual void update(float wall_dt, float ros_dt);
      virtual void reset();

      void setLinkColor(const std::string &link_name, const QColor &color);
      void unsetLinkColor(const std::string& link_name);

      void queueRenderSceneGeometry();

      // pass the execution of this function call to a separate thread that runs in the background
      void addBackgroundJob(const boost::function<void()> &job, const std::string &name);

      // queue the execution of this function for the next time the main update() loop gets called
      void addMainLoopJob(const boost::function<void()> &job);

      void waitForAllMainLoopJobs();

      // remove all queued jobs
      void clearJobs();

      const std::string getMoveGroupNS() const;
      const robot_model::RobotModelConstPtr& getRobotModel() const;
      planning_scene_monitor::LockedPlanningSceneRO getPlanningSceneRO() const;
      planning_scene_monitor::LockedPlanningSceneRW getPlanningSceneRW();
      const planning_scene_monitor::PlanningSceneMonitorPtr& getPlanningSceneMonitor();

    private Q_SLOTS:

      // ******************************************************************************************
      // Slot Event Functions
      // ******************************************************************************************
      void changedMoveGroupNS();
      void changedRobotDescription();
      void changedSceneName();
      void changedSceneEnabled();
      void changedSceneRobotVisualEnabled();
      void changedSceneRobotCollisionEnabled();
      void changedRobotSceneAlpha();
      void changedSceneAlpha();
      void changedSceneColor();
      void changedAttachedBodyColor();
      void changedSceneDisplayTime();
      void changedOctreeRenderMode();
      void changedOctreeColorMode();

    protected:

      /// This function reloads the robot model and reinitializes the PlanningSceneMonitor
      /// It can be called either from the Main Loop or from a Background thread
      void loadRobotModel();

      /// This function is used by loadRobotModel() and should only be called in the MainLoop
      /// You probably should not call this function directly
      void clearRobotModel();

      /// This function constructs a new planning scene. Probably this should be called in a background thread
      /// as it may take some time to complete its execution
      virtual planning_scene_monitor::PlanningSceneMonitorPtr createPlanningSceneMonitor();

      /// This is an event called by loadRobotModel() in the MainLoop; do not call directly
      virtual void onRobotModelLoaded();

      /**
      * \brief Set the scene node's position, given the target frame and the planning frame
      */
      void calculateOffsetPosition();

      void executeMainLoopJobs();
      void sceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);
      void renderPlanningScene();
      void setLinkColor(rviz::Robot* robot, const std::string& link_name, const QColor &color);
      void unsetLinkColor(rviz::Robot* robot, const std::string& link_name);
      void setGroupColor(rviz::Robot* robot, const std::string& group_name, const QColor &color);
      void unsetGroupColor(rviz::Robot* robot, const std::string& group_name);
      void unsetAllColors(rviz::Robot* robot);

      // overrides from Display
      virtual void onInitialize();
      virtual void onEnable();
      virtual void onDisable();
      virtual void fixedFrameChanged();

      // new virtual functions added by this plugin
      virtual void updateInternal(float wall_dt, float ros_dt);
      virtual void onSceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);

      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
      bool model_is_loading_;
      boost::mutex robot_model_loading_lock_;

      moveit::tools::BackgroundProcessing background_process_;
      std::deque<boost::function<void()> > main_loop_jobs_;
      boost::mutex main_loop_jobs_lock_;
      boost::condition_variable main_loop_jobs_empty_condition_;

      Ogre::SceneNode* planning_scene_node_;            ///< displays planning scene with everything in it

      // render the planning scene  
      moveit_rviz_plugin::RobotStateVisualizationPtr planning_scene_robot_;
      moveit_rviz_plugin::PlanningSceneRenderPtr planning_scene_render_;

      /// interactive Marker Stuff
      boost::shared_ptr<interactive_markers::MenuHandler> menu_handler_;
      interactive_markers::MenuHandler::EntryHandle update_planning_scene_entry_;
      interactive_markers::MenuHandler::EntryHandle print_current_posestamped_entry_;
      boost::shared_ptr<visualization_msgs::InteractiveMarker> int_marker_;
      boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
      void initInteractiveMarker();
      void resetInteractiveMarker();
      void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
      void updatePlanningScene();
      void printCurrentPoseStamped();
      void colorCollidingLinksRed(planning_scene_monitor::LockedPlanningSceneRW &ps);
      void colorCollidingLinksRed(planning_scene_monitor::LockedPlanningSceneRO &ps);

      bool planning_scene_needs_render_;
      float current_scene_time_;

      rviz::Property* scene_category_;
      rviz::Property* robot_category_;

      rviz::StringProperty* move_group_ns_property_;
      rviz::StringProperty* robot_description_property_;
      rviz::StringProperty* scene_name_property_;
      rviz::BoolProperty* scene_enabled_property_;
      rviz::BoolProperty* scene_robot_visual_enabled_property_;
      rviz::BoolProperty* scene_robot_collision_enabled_property_;
      rviz::RosTopicProperty* planning_scene_topic_property_;
      rviz::FloatProperty* robot_alpha_property_;
      rviz::FloatProperty* scene_alpha_property_;
      rviz::ColorProperty* scene_color_property_;
      rviz::ColorProperty* attached_body_color_property_;
      rviz::FloatProperty* scene_display_time_property_;
      rviz::EnumProperty* octree_render_property_;
      rviz::EnumProperty* octree_coloring_property_;
  };

} // namespace gki_3dnav_rviz_plugin

#endif
