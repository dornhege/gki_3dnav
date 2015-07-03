/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  Copyright (c) 2013, Ioan A. Sucan
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

#include <gki_3dnav_slice_rviz_plugin/gki_3dnav_slice_display.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit/rviz_plugin_render_tools/octomap_render.h>

#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>

#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <boost/foreach.hpp>

namespace gki_3dnav_rviz_plugin
{
  const std::string RobotSliceDisplay::INTERACTIVE_MARKER_TOPIC = "robot_slice_interactive_marker_topic";
  // ******************************************************************************************
  // Base class contructor
  // ******************************************************************************************
  RobotSliceDisplay::RobotSliceDisplay(bool listen_to_planning_scene, bool show_scene_robot) :
    Display(),
    model_is_loading_(false),
    planning_scene_needs_render_(true),
    current_scene_time_(0.0f)
  {
    move_group_ns_property_ =
      new rviz::StringProperty( "Move Group Namespace", "", "The name of the ROS namespace in which the move_group node is running",
                                this,
                                SLOT( changedMoveGroupNS() ), this );
    robot_description_property_ =
      new rviz::StringProperty( "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
                                this,
                                SLOT( changedRobotDescription() ), this );

    // Planning scene category -------------------------------------------------------------------------------------------
    scene_category_ = new rviz::Property( "Scene Geometry", QVariant(), "", this );

    scene_name_property_ =
      new rviz::StringProperty( "Scene Name", "(noname)", "Shows the name of the planning scene",
                                scene_category_,
                                SLOT( changedSceneName() ), this );
    scene_name_property_->setShouldBeSaved(false);
    scene_enabled_property_ =
      new rviz::BoolProperty( "Show Scene Geometry", true, "Indicates whether planning scenes should be displayed",
                              scene_category_,
                              SLOT( changedSceneEnabled() ), this );

    scene_alpha_property_ =
      new rviz::FloatProperty( "Scene Alpha", 0.9f, "Specifies the alpha for the scene geometry",
                              scene_category_,
                              SLOT( changedSceneAlpha() ), this );
    scene_alpha_property_->setMin( 0.0 );
    scene_alpha_property_->setMax( 1.0 );

    scene_color_property_ = new rviz::ColorProperty( "Scene Color", QColor(50, 230, 50), "The color for the planning scene obstacles (if a color is not defined)",
                                                    scene_category_,
                                                    SLOT( changedSceneColor() ), this );

    octree_render_property_ = new rviz::EnumProperty( "Voxel Rendering", "Occupied Voxels",
                                                "Select voxel type.",
                                                scene_category_, SLOT( changedOctreeRenderMode() ), this );

    octree_render_property_->addOption( "Occupied Voxels",  moveit_rviz_plugin::OCTOMAP_OCCUPIED_VOXELS );
    octree_render_property_->addOption( "Free Voxels",  moveit_rviz_plugin::OCTOMAP_FREE_VOXELS );
    octree_render_property_->addOption( "All Voxels",  moveit_rviz_plugin::OCTOMAP_FREE_VOXELS | moveit_rviz_plugin::OCTOMAP_OCCUPIED_VOXELS);

    octree_coloring_property_ = new rviz::EnumProperty( "Voxel Coloring", "Z-Axis",
                                                  "Select voxel coloring mode",
                                                  scene_category_, SLOT( changedOctreeColorMode() ), this );

    octree_coloring_property_->addOption( "Z-Axis",  moveit_rviz_plugin::OCTOMAP_Z_AXIS_COLOR );
    octree_coloring_property_->addOption( "Cell Probability",  moveit_rviz_plugin::OCTOMAP_PROBABLILTY_COLOR );

    scene_display_time_property_ =
      new rviz::FloatProperty( "Scene Display Time", 0.2f, "The amount of wall-time to wait in between rendering updates to the planning scene (if any)",
                              scene_category_,
                              SLOT( changedSceneDisplayTime() ), this );
    scene_display_time_property_->setMin(0.0001);

    if (show_scene_robot)
    {
      robot_category_  = new rviz::Property( "Scene Robot",   QVariant(), "", this );

      scene_robot_visual_enabled_property_ =
        new rviz::BoolProperty( "Show Robot Visual", true, "Indicates whether the robot state specified by the planning scene should be displayed as defined for visualisation purposes.",
                                robot_category_,
                                SLOT( changedSceneRobotVisualEnabled() ), this );

      scene_robot_collision_enabled_property_ =
        new rviz::BoolProperty("Show Robot Collision", false, "Indicates whether the robot state specified by the planning scene should be displayed as defined for collision detection purposes.",
                              robot_category_,
                              SLOT(changedSceneRobotCollisionEnabled()), this);

      robot_alpha_property_ =
        new rviz::FloatProperty( "Robot Alpha", 0.5f, "Specifies the alpha for the robot links",
                                robot_category_,
                                SLOT( changedRobotSceneAlpha() ), this );
      robot_alpha_property_->setMin( 0.0 );
      robot_alpha_property_->setMax( 1.0 );

      attached_body_color_property_ = new rviz::ColorProperty( "Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies",
                                                              robot_category_,
                                                              SLOT( changedAttachedBodyColor() ), this );
    }
    else
    {
      robot_category_ = NULL;
      scene_robot_visual_enabled_property_ = NULL;
      scene_robot_collision_enabled_property_ = NULL;
      robot_alpha_property_ = NULL;
      attached_body_color_property_ = NULL;
    }
  }

  // ******************************************************************************************
  // Deconstructor
  // ******************************************************************************************
  RobotSliceDisplay::~RobotSliceDisplay()
  {
    clearJobs();

    planning_scene_render_.reset();
    if (context_ && context_->getSceneManager() && planning_scene_node_)
      context_->getSceneManager()->destroySceneNode(planning_scene_node_->getName());
    if (planning_scene_robot_)
      planning_scene_robot_.reset();
    planning_scene_monitor_.reset();
  }

  void RobotSliceDisplay::clearJobs()
  {
    background_process_.clear();
    {
      boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
      main_loop_jobs_.clear();
    }
  }

  void RobotSliceDisplay::onInitialize()
  {
    Display::onInitialize();
    // Initialize the interactive marker server, menu and the marker itself
    initInteractiveMarker();

    // the scene node that contains everything
    planning_scene_node_ = scene_node_->createChildSceneNode();

    if (robot_category_)
    {
      planning_scene_robot_.reset(new moveit_rviz_plugin::RobotStateVisualization(planning_scene_node_, context_, "Slice Scene", robot_category_));
      planning_scene_robot_->setVisible(true);
      planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
      planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
    }
  }

  void RobotSliceDisplay::reset()
  {
    planning_scene_render_.reset();
    if (planning_scene_robot_)
      planning_scene_robot_->clear();

    addBackgroundJob(boost::bind(&RobotSliceDisplay::loadRobotModel, this), "loadRobotModel");
    Display::reset();

    if (planning_scene_robot_)
    {
      planning_scene_robot_->setVisible(true);
      planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
      planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
    }
  }

  void RobotSliceDisplay::addBackgroundJob(const boost::function<void()> &job, const std::string &name)
  {
    background_process_.addJob(job, name);
  }

  void RobotSliceDisplay::addMainLoopJob(const boost::function<void()> &job)
  {
    boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
    main_loop_jobs_.push_back(job);
  }

  void RobotSliceDisplay::waitForAllMainLoopJobs()
  {
    boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
    while (!main_loop_jobs_.empty())
      main_loop_jobs_empty_condition_.wait(ulock);
  }

  void RobotSliceDisplay::executeMainLoopJobs()
  {
    main_loop_jobs_lock_.lock();
    while (!main_loop_jobs_.empty())
    {
      boost::function<void()> fn = main_loop_jobs_.front();
      main_loop_jobs_.pop_front();
      main_loop_jobs_lock_.unlock();
      try
      {
        fn();
      }
      catch(std::runtime_error &ex)
      {
        ROS_ERROR("Exception caught executing main loop job: %s", ex.what());
      }
      catch(...)
      {
        ROS_ERROR("Exception caught executing main loop job");
      }
      main_loop_jobs_lock_.lock();
    }
    main_loop_jobs_empty_condition_.notify_all();
    main_loop_jobs_lock_.unlock();
  }

  const planning_scene_monitor::PlanningSceneMonitorPtr& RobotSliceDisplay::getPlanningSceneMonitor()
  {
    return planning_scene_monitor_;
  }

  const std::string RobotSliceDisplay::getMoveGroupNS() const
  {
    return move_group_ns_property_->getStdString();
  }

  const robot_model::RobotModelConstPtr& RobotSliceDisplay::getRobotModel() const
  {
    if (planning_scene_monitor_)
      return planning_scene_monitor_->getRobotModel();
    else
    {
      static robot_model::RobotModelConstPtr empty;
      return empty;
    }
  }

  planning_scene_monitor::LockedPlanningSceneRO RobotSliceDisplay::getPlanningSceneRO() const
  {
    return planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
  }

  planning_scene_monitor::LockedPlanningSceneRW RobotSliceDisplay::getPlanningSceneRW()
  {
    return planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_);
  }

  void RobotSliceDisplay::changedAttachedBodyColor()
  {
    queueRenderSceneGeometry();
  }

  void RobotSliceDisplay::changedSceneColor()
  {
    queueRenderSceneGeometry();
  }

  void RobotSliceDisplay::changedMoveGroupNS()
  {
    if (isEnabled())
      reset();
  }

  void RobotSliceDisplay::changedRobotDescription()
  {
    if (isEnabled())
      reset();
  }

  void RobotSliceDisplay::changedSceneName()
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = getPlanningSceneRW();
    if (ps)
      ps->setName(scene_name_property_->getStdString());
  }

  void RobotSliceDisplay::renderPlanningScene()
  {
    if (planning_scene_render_ && planning_scene_needs_render_)
    {
      QColor color = scene_color_property_->getColor();
      rviz::Color env_color(color.redF(), color.greenF(), color.blueF());
      if (attached_body_color_property_)
        color = attached_body_color_property_->getColor();
      rviz::Color attached_color(color.redF(), color.greenF(), color.blueF());

      try
      {
        const planning_scene_monitor::LockedPlanningSceneRO &ps = getPlanningSceneRO();
        planning_scene_render_->renderPlanningScene(ps, env_color,
                                                    attached_color,
                                                    static_cast<moveit_rviz_plugin::OctreeVoxelRenderMode>(octree_render_property_->getOptionInt()),
                                                    static_cast<moveit_rviz_plugin::OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt()),
                                                    scene_alpha_property_->getFloat());
      }
      catch(...)
      {
        ROS_ERROR("Exception thrown while rendering planning scene");
      }
      planning_scene_needs_render_ = false;
      planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());
    }
  }

  void RobotSliceDisplay::changedSceneAlpha()
  {
    queueRenderSceneGeometry();
  }

  void RobotSliceDisplay::changedRobotSceneAlpha()
  {
    if (planning_scene_robot_)
    {
      planning_scene_robot_->setAlpha(robot_alpha_property_->getFloat());
      queueRenderSceneGeometry();
    }
  }  

  void RobotSliceDisplay::changedSceneDisplayTime()
  {
  }

  void RobotSliceDisplay::changedOctreeRenderMode()
  {
    planning_scene_needs_render_ = true;
  }

  void RobotSliceDisplay::changedOctreeColorMode()
  {
    planning_scene_needs_render_ = true;
  }

  void RobotSliceDisplay::changedSceneRobotVisualEnabled()
  {
    if (isEnabled() && planning_scene_robot_)
    {
      planning_scene_robot_->setVisible(true);
      planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
    }
  }

  void RobotSliceDisplay::changedSceneRobotCollisionEnabled()
  {
    if (isEnabled() && planning_scene_robot_)
    {
      planning_scene_robot_->setVisible(true);
      planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
    }
  }

  void RobotSliceDisplay::changedSceneEnabled()
  {
    if (planning_scene_render_)
      planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());
  }

  void RobotSliceDisplay::setGroupColor(rviz::Robot* robot, const std::string& group_name, const QColor &color)
  {
    if (getRobotModel())
    {
      const robot_model::JointModelGroup *jmg = getRobotModel()->getJointModelGroup(group_name);
      if (jmg)
      {
        const std::vector<std::string> &links = jmg->getLinkModelNamesWithCollisionGeometry();
        for (std::size_t i = 0 ; i < links.size() ; ++i)
          setLinkColor(robot, links[i], color);
      }
    }
  }

  void RobotSliceDisplay::unsetAllColors(rviz::Robot* robot)
  {
    if (getRobotModel())
    {
      const std::vector<std::string> &links = getRobotModel()->getLinkModelNamesWithCollisionGeometry();
      for (std::size_t i = 0 ; i < links.size() ; ++i)
        unsetLinkColor(robot, links[i]);
    }
  }

  void RobotSliceDisplay::unsetGroupColor(rviz::Robot* robot, const std::string& group_name )
  {
    if (getRobotModel())
    {
      const robot_model::JointModelGroup *jmg = getRobotModel()->getJointModelGroup(group_name);
      if (jmg)
      {
        const std::vector<std::string> &links = jmg->getLinkModelNamesWithCollisionGeometry();
        for (std::size_t i = 0 ; i < links.size() ; ++i)
          unsetLinkColor(robot, links[i]);
      }
    }
  }

  void RobotSliceDisplay::setLinkColor(const std::string& link_name, const QColor &color)
  {
    if (planning_scene_robot_)
      setLinkColor(&planning_scene_robot_->getRobot(), link_name, color );
  }

  void RobotSliceDisplay::unsetLinkColor(const std::string& link_name)
  {
    if (planning_scene_robot_)
      unsetLinkColor(&planning_scene_robot_->getRobot(), link_name);
  }

  void RobotSliceDisplay::setLinkColor(rviz::Robot* robot, const std::string& link_name, const QColor &color )
  {
    rviz::RobotLink *link = robot->getLink(link_name);

    // Check if link exists
    if (link)
      link->setColor( color.redF(), color.greenF(), color.blueF() );
  }

  void RobotSliceDisplay::unsetLinkColor(rviz::Robot* robot, const std::string& link_name )
  {
    rviz::RobotLink *link = robot->getLink(link_name);

    // Check if link exists
    if (link)
      link->unsetColor();
  }

  // ******************************************************************************************
  // Load
  // ******************************************************************************************
  planning_scene_monitor::PlanningSceneMonitorPtr RobotSliceDisplay::createPlanningSceneMonitor()
  {
    return planning_scene_monitor::PlanningSceneMonitorPtr
      (new planning_scene_monitor::PlanningSceneMonitor(robot_description_property_->getStdString(),
                                                        context_->getFrameManager()->getTFClientPtr(),
                                                        getNameStd() + "_planning_scene_monitor"));
  }

  void RobotSliceDisplay::clearRobotModel()
  {
    planning_scene_render_.reset();
    planning_scene_monitor_.reset(); // this so that the destructor of the PlanningSceneMonitor gets called before a new instance of a scene monitor is constructed
  }

  void RobotSliceDisplay::loadRobotModel()
  {
    // wait for other robot loadRobotModel() calls to complete;
    boost::mutex::scoped_lock _(robot_model_loading_lock_);
    model_is_loading_ = true;

    // we need to make sure the clearing of the robot model is in the main thread,
    // so that rendering operations do not have data removed from underneath,
    // so the next function is executed in the main loop
    addMainLoopJob(boost::bind(&RobotSliceDisplay::clearRobotModel, this));

    waitForAllMainLoopJobs();

    planning_scene_monitor::PlanningSceneMonitorPtr psm = createPlanningSceneMonitor();
    if (psm->getPlanningScene())
    {
      planning_scene_monitor_.swap(psm);
      addMainLoopJob(boost::bind(&RobotSliceDisplay::onRobotModelLoaded, this));
      setStatus(rviz::StatusProperty::Ok, "PlanningScene", "Planning Scene Loaded Successfully");
      waitForAllMainLoopJobs();
    }
    else
    {
      setStatus(rviz::StatusProperty::Error, "PlanningScene", "No Planning Scene Loaded");
    }

    if (planning_scene_monitor_)
      planning_scene_monitor_->addUpdateCallback(boost::bind(&RobotSliceDisplay::sceneMonitorReceivedUpdate, this, _1));

    model_is_loading_ = false;
  }

  void RobotSliceDisplay::onRobotModelLoaded()
  {
    updatePlanningScene();
    planning_scene_render_.reset(new moveit_rviz_plugin::PlanningSceneRender(planning_scene_node_, context_, planning_scene_robot_));
    planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());

    const planning_scene_monitor::LockedPlanningSceneRO &ps = getPlanningSceneRO();
    if (planning_scene_robot_)
    {
      planning_scene_robot_->load(*getRobotModel()->getURDF());
      robot_state::RobotState *rs = new robot_state::RobotState(ps->getCurrentState());
      rs->update();
      planning_scene_robot_->update(robot_state::RobotStateConstPtr(rs));
    }

    bool oldState = scene_name_property_->blockSignals(true);
    scene_name_property_->setStdString(ps->getName());
    scene_name_property_->blockSignals(oldState);
  }

  void RobotSliceDisplay::sceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
  {
    onSceneMonitorReceivedUpdate(update_type);
  }

  void RobotSliceDisplay::onSceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
  {
    bool oldState = scene_name_property_->blockSignals(true);
    getPlanningSceneRW()->getCurrentStateNonConst().update();
    scene_name_property_->setStdString(getPlanningSceneRO()->getName());
    scene_name_property_->blockSignals(oldState);

    planning_scene_needs_render_ = true;
  }

  void RobotSliceDisplay::onEnable()
  {
    Display::onEnable();

    addBackgroundJob(boost::bind(&RobotSliceDisplay::loadRobotModel, this), "loadRobotModel");

    if (planning_scene_robot_)
    {
      planning_scene_robot_->setVisible(true);
      planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
      planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
    }
    if (planning_scene_render_)
      planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());

    calculateOffsetPosition();
  }

  // ******************************************************************************************
  // Disable
  // ******************************************************************************************
  void RobotSliceDisplay::onDisable()
  {
    if (planning_scene_monitor_)
    {
      //planning_scene_monitor_->stopSceneMonitor();
      if (planning_scene_render_)
        planning_scene_render_->getGeometryNode()->setVisible(false);
    }
    if (planning_scene_robot_)
      planning_scene_robot_->setVisible(false);

    server_->clear();
    server_->applyChanges();

    Display::onDisable();
  }

  void RobotSliceDisplay::queueRenderSceneGeometry()
  {
    planning_scene_needs_render_ = true;
  }

  void RobotSliceDisplay::update(float wall_dt, float ros_dt)
  {
    Display::update(wall_dt, ros_dt);

    executeMainLoopJobs();

    if (planning_scene_monitor_)
      updateInternal(wall_dt, ros_dt);
  }

  void RobotSliceDisplay::updateInternal(float wall_dt, float ros_dt)
  {
    current_scene_time_ += wall_dt;
    if (current_scene_time_ > scene_display_time_property_->getFloat())
    {
      renderPlanningScene();
      current_scene_time_ = 0.0f;
    }
  }

  void RobotSliceDisplay::load( const rviz::Config& config )
  {
    Display::load(config);
  }

  void RobotSliceDisplay::save( rviz::Config config ) const
  {
    Display::save(config);
  }

  // ******************************************************************************************
  // Calculate Offset Position
  // ******************************************************************************************
  void RobotSliceDisplay::calculateOffsetPosition()
  {
    if (!getRobotModel())
      return;

    tf::Stamped<tf::Pose> pose(tf::Pose::getIdentity(), ros::Time(0), getRobotModel()->getModelFrame());
    static const unsigned int max_attempts = 10;
    unsigned int attempts = 0;
    while (!context_->getTFClient()->canTransform(fixed_frame_.toStdString(), getRobotModel()->getModelFrame(), ros::Time(0)) && attempts < max_attempts)
    {
      ros::Duration(0.1).sleep();
      attempts++;
    }

    if (attempts < max_attempts)
    {
      try
      {
        context_->getTFClient()->transformPose(fixed_frame_.toStdString(), pose, pose);
      }
      catch (tf::TransformException& e)
      {
        ROS_ERROR( "Error transforming from frame '%s' to frame '%s'", pose.frame_id_.c_str(), fixed_frame_.toStdString().c_str() );
      }
    }

    Ogre::Vector3 position(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
    const tf::Quaternion &q = pose.getRotation();
    Ogre::Quaternion orientation( q.getW(), q.getX(), q.getY(), q.getZ() );
    planning_scene_node_->setPosition(position);
    planning_scene_node_->setOrientation(orientation);
  }

  void RobotSliceDisplay::fixedFrameChanged()
  {
    Display::fixedFrameChanged();
    calculateOffsetPosition();
    resetInteractiveMarker();
  }

  // ******************************************************************************************
  // Interactive Marker
  // ******************************************************************************************
  void RobotSliceDisplay::initInteractiveMarker()
  {
    geometry_msgs::PoseStamped poseStamped;
    tf::poseStampedTFToMsg(tf::Stamped<tf::Pose>(tf::Pose::getIdentity(), ros::Time(0), fixed_frame_.toStdString()), poseStamped);
    server_.reset(new interactive_markers::InteractiveMarkerServer(INTERACTIVE_MARKER_TOPIC));
    menu_handler_.reset(new interactive_markers::MenuHandler());
    int_marker_.reset(new visualization_msgs::InteractiveMarker(robot_interaction::makePlanarXYMarker("interactive_slice_marker", poseStamped, 1.0, false)));
    update_planning_scene_entry_ = menu_handler_->insert("Update Planning Scene",  boost::bind(&RobotSliceDisplay::updatePlanningScene, this));
    print_current_posestamped_entry_ = menu_handler_->insert("PrintCurrentPoseStamped",  boost::bind(&RobotSliceDisplay::printCurrentPoseStamped, this));
  }

  void RobotSliceDisplay::resetInteractiveMarker()
  {
    if (!getRobotModel()) return;
    planning_scene_monitor::LockedPlanningSceneRO ps = getPlanningSceneRO();
    robot_state::RobotStatePtr rs = robot_state::RobotStatePtr(new robot_state::RobotState(ps->getCurrentState()));

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time(0);
    poseStamped.header.frame_id = getRobotModel()->getModelFrame();

    geometry_msgs::Pose gpose;
    Eigen::Affine3d epose = rs->getJointTransform(getRobotModel()->getRootJoint());
    tf::poseEigenToMsg(epose, gpose);
    poseStamped.pose = gpose;

    server_->clear();
    server_->insert(*int_marker_);
    server_->setPose(int_marker_->name, poseStamped.pose, poseStamped.header);
    server_->setCallback(int_marker_->name, boost::bind(&RobotSliceDisplay::processFeedback, this, _1));

    menu_handler_->apply(*server_, "interactive_slice_marker");
    server_->applyChanges();
    colorCollidingLinksRed(ps);
  }

  void RobotSliceDisplay::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    geometry_msgs::Pose pose = feedback->pose;
    if (!planning_scene_robot_ || !getRobotModel()) return;
    planning_scene_monitor::LockedPlanningSceneRW ps = getPlanningSceneRW();
    robot_state::RobotState & rs = ps->getCurrentStateNonConst();

    Eigen::Affine3d epose;
    tf::poseMsgToEigen(pose, epose);
    rs.setJointPositions(getRobotModel()->getRootJoint(), epose);
    rs.update();
    planning_scene_robot_->update(robot_state::RobotStateConstPtr(new robot_state::RobotState(rs)));
    colorCollidingLinksRed(ps);
    server_->setPose(feedback->marker_name, pose);
  }

  void RobotSliceDisplay::colorCollidingLinksRed(planning_scene_monitor::LockedPlanningSceneRW &ps)
  {
    if (!planning_scene_robot_) return;
    unsetAllColors(&planning_scene_robot_->getRobot());
    if(ps->isStateColliding()>0){
      std::vector<std::string> collision_links;
      ps->getCollidingLinks(collision_links, ps->getCurrentState());

      const QColor color = QColor::fromHsv (0, 255, 255, 1);
      BOOST_FOREACH(std::string cln, collision_links){
        rviz::RobotLink *link = planning_scene_robot_->getRobot().getLink(cln); 
        if(link) link->setColor(color.redF(), color.greenF(), color.blueF());
      }
    }
  }

  void RobotSliceDisplay::colorCollidingLinksRed(planning_scene_monitor::LockedPlanningSceneRO &ps)
  {
    if (!planning_scene_robot_) return;
    unsetAllColors(&planning_scene_robot_->getRobot());
    if(ps->isStateColliding()>0){
      std::vector<std::string> collision_links;
      ps->getCollidingLinks(collision_links, ps->getCurrentState());

      const QColor color = QColor::fromHsv (0, 255, 255, 1);
      BOOST_FOREACH(std::string cln, collision_links){
        rviz::RobotLink *link = planning_scene_robot_->getRobot().getLink(cln); 
        if(link) link->setColor(color.redF(), color.greenF(), color.blueF());
      }
    }
  }

  void RobotSliceDisplay::updatePlanningScene()
  {
    if (planning_scene_monitor_)
    {
      planning_scene_monitor_->requestPlanningSceneState(
          ros::names::append(getMoveGroupNS(),planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE));
      planning_scene_needs_render_ = true;
      resetInteractiveMarker();
    }
  }

  void RobotSliceDisplay::printCurrentPoseStamped()
  {
    if (!getRobotModel()) return;
    planning_scene_monitor::LockedPlanningSceneRO ps = getPlanningSceneRO();
    robot_state::RobotStatePtr rs = robot_state::RobotStatePtr(new robot_state::RobotState(ps->getCurrentState()));

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time(0);
    poseStamped.header.frame_id = getRobotModel()->getModelFrame();

    geometry_msgs::Pose gpose;
    Eigen::Affine3d epose = rs->getJointTransform(getRobotModel()->getRootJoint());
    tf::poseEigenToMsg(epose, gpose);
    poseStamped.pose = gpose;

    ROS_INFO_STREAM("" << poseStamped);
  }
}
// namespace gki_3dnav_rviz_plugin
