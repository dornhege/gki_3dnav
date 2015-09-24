/*
 * VisualPathPlannerNode.h
 *
 *  Created on: 2 Mar 2013
 *      Author: andreas
 */

#ifndef VISUALPATHPLANNERNODE_H_
#define VISUALPATHPLANNERNODE_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
//#include "gki_3dnav_planner/CheckCollisions.h"
//#include "gki_3dnav_planner/PlanPath.h"
#include <octomap_msgs/Octomap.h>
#include <tf/transform_listener.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>

namespace gki_3dnav_planner
{

class VisualPathPlannerNode
{
public:
    VisualPathPlannerNode();
    virtual ~VisualPathPlannerNode();

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

private:
    void loadMap();
    void updatePose(geometry_msgs::Pose pose);
    void attachBody();
    void detachBody();

    bool attached;
    EigenSTL::vector_Affine3d world_coke_poses;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> collisioinTestServer;
    visualization_msgs::InteractiveMarker start_control_marker;
    visualization_msgs::InteractiveMarker goal_control_marker;
    ros::ServiceClient planPathClient;
    ros::Subscriber octomapSubscriber;
    boost::shared_ptr<tf::TransformListener> tf_listener;

    planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor;
    planning_scene::PlanningScenePtr scene;
    ros::Publisher planning_scene_publisher;
};

} /* namespace gki_3dnav_planner */
#endif /* VISUALPATHPLANNERNODE_H_ */
