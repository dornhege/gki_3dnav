#ifndef ENVIRONMENT_NAVXYTHETALAT_MOVEIT_H
#define ENVIRONMENT_NAVXYTHETALAT_MOVEIT_H

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "freespace_mechanism_heuristic/freespace_mechanism_heuristic.h"

// TODO
// Later/Moveit: Needs new init/conversion functions, etc. as costmap is no longer out target
// check interaction with planning scene from up planner class
// Integrate these functions directly to work on octomap w/o coll map
// recheck all uses of the grid everywhere, e.g. heuristics, etc.
// - should be NULL (set to NULL) and dump everywhere then
// - maybe grid could be usefull for pessimistic, but should be noted as assumption-wise (e.g., only if robot
//      has pessimistic assumption and we can go "through" it, radius inscribed then must be
//      the inner pessimitic radius
// -> in that case costmap still usable, need to address construction/transformations between
// costmap, octomap, environment grid (.grid + heuristic) -> Def. what our world model is!

class EnvironmentNavXYThetaLatMoveit : public EnvironmentNAVXYTHETALAT
{
    public:
        EnvironmentNavXYThetaLatMoveit(ros::NodeHandle & nhPriv, double costmapOffsetX, double costmapOffsetY);
        virtual ~EnvironmentNavXYThetaLatMoveit() {}

        virtual int SetGoal(double x_m, double y_m, double theta_rad);
        virtual int SetStart(double x_m, double y_m, double theta_rad);
        virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta);
        virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta);

        bool gridToWorld(int X, int Y, int Theta, double & x, double & y, double & theta) const;
        geometry_msgs::Pose poseFromStateID(int stateID) const;

        virtual void clear_full_body_collision_infos();
        virtual void publish_expanded_states();

        /// Update the planning scene directly from the running MoveGroup instance.
        virtual void update_planning_scene();
        /// Update the planning scene to a custom one.
        virtual void update_planning_scene(planning_scene::PlanningSceneConstPtr scene);
        /// Publish the currently used planning scene instance.
        virtual void publish_planning_scene();
        /// Use this to access the PlanningScene. Never use internal data structures for that.
        virtual planning_scene::PlanningSceneConstPtr getPlanningScene();

        bool useFreespaceHeuristic(bool on) { useFreespaceHeuristic_ = on; }

        virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
        virtual int GetStartHeuristic(int stateID);
        virtual int GetGoalHeuristic(int stateID);

        moveit_msgs::DisplayTrajectory pathToDisplayTrajectory(const std::vector<geometry_msgs::PoseStamped> & path) const;

        int count;
        int past;
    protected:
        virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);

    protected:
        struct FullBodyCollisionInfo
        {
            bool initialized;
            bool collision;

            FullBodyCollisionInfo()
            {
                initialized = false;
                collision = true;
            }
        };
        std::vector<FullBodyCollisionInfo> full_body_collision_infos;
        freespace_mechanism_heuristic::HeuristicCostMap* freespace_heuristic_costmap;
        bool useFreespaceHeuristic_;

        planning_scene::PlanningScenePtr scene;
        planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor;
        std::string scene_update_name;  ///< Scene updates are queried by this service.
        ros::Publisher planning_scene_publisher;
        std::vector<std::string> allowed_collision_links;

        ros::Publisher pose_array_publisher;

        sbpl_xy_theta_pt_t discreteToContinuous(int x, int y, int theta) const;

        bool in_full_body_collision(EnvNAVXYTHETALATHashEntry_t* state);
        const FullBodyCollisionInfo& get_full_body_collision_info(EnvNAVXYTHETALATHashEntry_t* state);

        // offsets to convert costmap coordinates to world coordinates for 3d collision checks
        double costmapOffsetX;
        double costmapOffsetY;
};

#endif

