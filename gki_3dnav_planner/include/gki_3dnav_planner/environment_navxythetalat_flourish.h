#ifndef ENVIRONMENT_NAVXYTHETALAT_FLOURISH_H
#define ENVIRONMENT_NAVXYTHETALAT_FLOURISH_H

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <navigation/mapping/metaMap/base/traversableMap.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <sbpl/utils/key.h>
#include "timing/timing.h"

#include "freespace_mechanism_heuristic/freespace_mechanism_heuristic.h"

#include <interactive_markers/interactive_marker_server.h>

class EnvironmentNavXYThetaLatFlourish : public EnvironmentNAVXYTHETALAT
{
 public:
  EnvironmentNavXYThetaLatFlourish(ros::NodeHandle* nhPriv, Ais3dTools::TraversableMap tMap);
  virtual ~EnvironmentNavXYThetaLatFlourish();

  // check if the cell given by indices x, y is inside the map and the height is safe for the robot
  bool IsValidCell(int X, int Y);
  // check if the cell given by indices x, y is inside the map 
  bool IsWithinMapCell(int X, int Y);
  // check if all cells in the footprint given the robot is at pose x, y, theta are valid. Uses isValidCell.
  bool IsValidConfiguration(int X, int Y, int Theta);
  // check if the traversable map says cell x, y is traversable or not
  bool IsObstacle(int x, int y);
  bool IsObstacle(Eigen::Vector2i index);
  double getMapOffsetX() { return mapOffsetX; }
  double getMapOffsetY() { return mapOffsetY; }
  const Ais3dTools::TraversableMap& traversableMap() const { return tMap; }

  // if x, y, theta is a valid configuration:
  // sets the start state for the planning task to x_m, y_m, theta_rad
  // creates a hash entry with the corresponding state id, returns the state id
  // else: returns -1
  virtual int SetStart(double x_m, double y_m, double theta_rad);
  // if x, y, theta is a valid configuration:
  // sets the goal state for the planning task to x_m, y_m, theta_rad
  // creates a hash entry with the corresponding state id, returns the state id
  // else: returns -1
  virtual int SetGoal(double x_m, double y_m, double theta_rad);
  // sets the parameter on whether we want to use the freespace heuristic
  bool useFreespaceHeuristic(bool on) { useFreespaceHeuristic_ = on; }
  // create a new hash entry for the given state
  virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta);
  virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta);

  // delete all entries in the full_body_traversability_cost_infos
  virtual void clear_full_body_traversability_cost_infos();

  // get a geometry_msgs::Pose from a given state id
  geometry_msgs::Pose poseFromStateID(int stateID) const;
  
  // compute and publish the wheel cells relative to the current state 
  // TODO check
  void computeWheelPositions();

  moveit_msgs::DisplayTrajectory pathToDisplayTrajectory(const std::vector<geometry_msgs::PoseStamped>& path) const;
  void ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath, std::vector<sbpl_xy_theta_pt_t>* xythetaPath);
  void resetTimingStats();
  void printTimingStats();


  // get euclidean distance from state FromStateID to state ToStateID in m
  virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
  // get euclidean distance from start in m
  virtual int GetStartHeuristic(int stateID);
  // get euclidean distance to goal in m
  virtual int GetGoalHeuristic(int stateID);

  // not doing anything right now
  void EnsureHeuristicsUpdated(bool bGoalHeuristics);
  //bool UpdateCost(int x, int y, unsigned char newcost);
  //unsigned char GetMapCost(int x, int y);

  /// Update the planning scene directly from the running MoveGroup instance.
  virtual void update_planning_scene();
  /// Update the planning scene to a custom one.
  virtual void update_planning_scene(planning_scene::PlanningSceneConstPtr scene);
  /// Use this to access the PlanningScene. Never use internal data structures for that.
  virtual planning_scene::PlanningSceneConstPtr getPlanningScene();

  //int ComputeCosts(int SourceX, int SourceY, int SourceTheta);

  // DEBUGGING
  /// Publish the currently used planning scene instance.
  virtual void publish_planning_scene();
  //virtual void publish_expanded_states();
  void publish_wheel_cells(std::vector<Eigen::Vector2i> wheelCells);
  void publish_traversable_map();
  //void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  int count;
  int past;

 protected:
  struct FullBodyTraversabilityCost
  {
    int cost;
    bool initialized;

    FullBodyTraversabilityCost()
    {
      initialized = false;
      cost = INFINITECOST;
    }
  };

  
  // compute the cost of state x, y, theta 
  // if a wheel position is outside the map: infty
  // if the cell x, y is outside the map or too high: infty
  // else: cost = distance to goal
  int GetCellCost(int X, int Y, int Theta);
  virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);
  virtual void SetConfiguration(int width, int height,
			const unsigned char* mapdata,
			int startx, int starty, int starttheta,
			int goalx, int goaly, int goaltheta,
			double cellsize_m, double nominalvel_mpersecs,
			double timetoturn45degsinplace_secs,
			const std::vector<sbpl_2Dpt_t> & robot_perimeterV);


  //sbpl_xy_theta_pt_t discreteToContinuous(int x, int y, int theta) const;
  //void discreteToContinuous(int x_d, int y_d, int theta_d, double& x_c, double& y_c, double& theta_c) const;
  //void discreteXYToContinuous(int x_d, int y_d, double& x_c, double& y_c) const;
  //void continuousToDiscrete(sbpl_xy_theta_pt_t pose, int& x, int& y, int& theta) const;
  //void continuousToDiscrete(double x_c, double y_c, double theta_c, int& x, int& y, int& theta) const;
  //void continuousXYToDiscrete(double x_c, double y_c, int& x, int& y) const;
  //Eigen::Vector2i continuousXYToDiscrete(Eigen::Vector2f xy_c) const;

  //sbpl_xy_theta_pt_t discreteToContinuous(int x, int y, int theta) const;
  sbpl_xy_theta_pt_t gridToWorld(int x, int y, int theta) const;
  void gridToWorld(int x_d, int y_d, int theta_d, double& x_c, double& y_c, double& theta_c) const;
  void grid2dToWorld(int x_d, int y_d, double& x_c, double& y_c) const;
  void worldToGrid(sbpl_xy_theta_pt_t pose, int& x, int& y, int& theta) const;
  void worldToGrid(double x_c, double y_c, double theta_c, int& x, int& y, int& theta) const;
  void world2dToGrid(double x_c, double y_c, int& x, int& y) const;
  Eigen::Vector2i world2dToGrid(Eigen::Vector2f xy_c) const;

  bool in_full_body_collision(EnvNAVXYTHETALATHashEntry_t* state);
  const FullBodyTraversabilityCost& get_full_body_traversability_cost_info(EnvNAVXYTHETALATHashEntry_t* state);

  std::vector<FullBodyTraversabilityCost> full_body_traversability_cost_infos;
  freespace_mechanism_heuristic::HeuristicCostMap* freespace_heuristic_costmap;
  bool useFreespaceHeuristic_;

  planning_scene::PlanningScenePtr scene;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor;
  std::string scene_update_name;  ///< Scene updates are queried by this service.
  ros::Publisher planning_scene_publisher;
  ros::Publisher wheel_cells_publisher;
  ros::Publisher traversable_map_publisher;
  std::vector<std::string> allowed_collision_links;

  //ros::Publisher pose_array_publisher;
  //ros::Publisher nontravpose_array_publisher;
  //ros::Publisher nontravaction_array_publisher;
  //ros::Publisher action_array_publisher;
  //ros::Publisher endtheta_array_publisher;

  // offsets to convert costmap coordinates to world coordinates for 3d collision checks
  double mapOffsetX;
  double mapOffsetY;
  Ais3dTools::TraversableMap tMap;
  Eigen::Isometry3f rightFrontWheelToBaseLink, leftFrontWheelToBaseLink, rightRearWheelToBaseLink, leftRearWheelToBaseLink;

  Timing* timeActionCost;
  //Timing* timeActionCostParent;
  Timing* timeFullBodyCollision;
  Timing* timeConfigCollisionCheck;
  Timing* timeTrafoComputation;
  Timing* timeHeuristic;

  float robotHeight;
  float robotBodyHeight;
  float robotBodyWidth; 
  float robotBodyLength;
  float robotArmLength;
  float robotMinSafeDistance;
  float robotSafeHeight;

  tf::TransformListener* tfListener;

  std::string planningFrameID;
  interactive_markers::InteractiveMarkerServer* interserver;
};

#endif // ENVIRONMENT_NAVXYTHETALAT_FLOURISH_H
