#ifndef ENVIRONMENT_NAVXYTHETALAT_FLOURISH_H
#define ENVIRONMENT_NAVXYTHETALAT_FLOURISH_H

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <navigation/mapping/metaMap/base/traversableMap.h>

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

class EnvironmentNavXYThetaLatFlourish : public EnvironmentNAVXYTHETALAT
{
 public:
  bool IsValidCell(int X, int Y);
  bool IsWithinMapCell(int X, int Y);
  bool IsValidConfiguration(int X, int Y, int Theta);
  void EnsureHeuristicsUpdated(bool bGoalHeuristics);
  bool UpdateCost(int x, int y, unsigned char newcost);
  bool IsObstacle(int x, int y);
  unsigned char GetMapCost(int x, int y);
  void ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath, std::vector<sbpl_xy_theta_pt_t>* xythetaPath);
  int SetG(double x_m, double y_m, double theta_rad);
  bool tryToReadPrims(FILE* file){ return ReadMotionPrimitives(file); }
  EnvironmentNavXYThetaLatFlourish(ros::NodeHandle & nhPriv, Ais3dTools::TraversableMap tMap);
  virtual ~EnvironmentNavXYThetaLatFlourish() {}

  virtual int SetGoal(double x_m, double y_m, double theta_rad);
  virtual int SetStart(double x_m, double y_m, double theta_rad);
  virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta);
  virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta);

  virtual void clear_full_body_collision_infos();
  virtual void publish_expanded_states();

  void publish_traversable_map();

  /// Update the planning scene directly from the running MoveGroup instance.
  virtual void update_planning_scene();
  /// Update the planning scene to a custom one.
  virtual void update_planning_scene(planning_scene::PlanningSceneConstPtr scene);
  /// Publish the currently used planning scene instance.
  virtual void publish_planning_scene();
  /// Use this to access the PlanningScene. Never use internal data structures for that.
  virtual planning_scene::PlanningSceneConstPtr getPlanningScene();

 protected:
  int GetCellCost(int X, int Y, int Theta);
  virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);
  virtual void SetConfiguration(int width, int height,
			const unsigned char* mapdata,
			int startx, int starty, int starttheta,
			int goalx, int goaly, int goaltheta,
			double cellsize_m, double nominalvel_mpersecs,
			double timetoturn45degsinplace_secs,
			const std::vector<sbpl_2Dpt_t> & robot_perimeterV);

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
  ros::Publisher traversable_map_publisher;
  std::vector<std::string> allowed_collision_links;

  ros::Publisher pose_array_publisher;


  sbpl_xy_theta_pt_t discreteToContinuous(int x, int y, int theta);

  bool in_full_body_collision(EnvNAVXYTHETALATHashEntry_t* state);
  const FullBodyCollisionInfo& get_full_body_collision_info(EnvNAVXYTHETALATHashEntry_t* state);

  // offsets to convert costmap coordinates to world coordinates for 3d collision checks
  double costmapOffsetX;
  double costmapOffsetY;
  Ais3dTools::TraversableMap tMap;
  Eigen::Isometry3f rightFrontWheelToBaseLink, leftFrontWheelToBaseLink, rightRearWheelToBaseLink, leftRearWheelToBaseLink;

  float robotHeight;
  float robotBodyHeight;
  float robotBodyWidth; 
  float robotBodyLength;
  float robotArmLength;
  float robotMinSafeDistance;
  float robotSafeHeight;


  std::string planningFrameID;
};

#endif // ENVIRONMENT_NAVXYTHETALAT_FLOURISH_H
