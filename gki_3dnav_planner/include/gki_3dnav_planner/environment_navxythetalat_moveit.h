#ifndef ENVIRONMENT_NAVXYTHETALAT_MOVEIT_H
#define ENVIRONMENT_NAVXYTHETALAT_MOVEIT_H

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// TODO
// FIXME: Do we need to re-set the CreateNewHashEntry_lookup pointer to member things in init or does polymorphism work here?
// Fix + debug this to be working
// -> exchancge planning scene to locked
// check who/what uses getActionCost and refactor checks into there if possible (one time check)
// check how full body caching works, or what we need as cache key now
//
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
        virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionV = NULL);
        virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
        virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

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


        planning_scene::PlanningScenePtr scene;
        planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor;
        std::string scene_update_name;  ///< Scene updates are queried by this service.
        ros::Publisher planning_scene_publisher;

        std::vector<std::string> allowed_collision_links;


        ros::Publisher pose_array_publisher;


        // TODO this really needed or can we use another disc2cont style fn?
        sbpl_xy_theta_pt_t discreteToContinuous(int x, int y, int theta);

        bool in_full_body_collision(EnvNAVXYTHETALATHashEntry_t* state);
        const FullBodyCollisionInfo& get_full_body_collision_info(EnvNAVXYTHETALATHashEntry_t* state);

        // offsets to convert costmap coordinates to world coordinates for 3d collision checks
        double costmapOffsetX;
        double costmapOffsetY;
};

#endif
