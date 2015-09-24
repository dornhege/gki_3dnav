#ifndef SBPL_LATTICE_PLANNER_H
#define SBPL_LATTICE_PLANNER_H

#include <iostream>
#include <vector>

using namespace std;

/** ROS **/
#include <ros/ros.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

// sbpl headers
#include "gki_3dnav_planner/environment_xyt_3d_collisions.h"
#include <sbpl/headers.h>
#include <geometry_msgs/Pose2D.h>

//global representation
#include <nav_core/base_global_planner.h>

namespace gki_3dnav_planner
{

class GKI3dNavPlanner: public nav_core::BaseGlobalPlanner
{
public:

	/**
	 * @brief  Default constructor for the NavFnROS object
	 */
	GKI3dNavPlanner();

	/**
	 * @brief  Constructor for the SBPLLatticePlanner object
	 * @param  name The name of this planner
	 * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
	 */
	GKI3dNavPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

	/**
	 * @brief  Initialization function for the SBPLLatticePlanner object
	 * @param  name The name of this planner
	 * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
	 */
	virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

	/**
	 * @brief Given a goal pose in the world, compute a plan
	 * @param scene The initial scene scpecified as a moveit planning scene
	 * @param goal The goal pose
	 * @param plan The plan... filled by the planner
	 * @return True if a valid plan was found, false otherwise
	 */
	bool makePlan(planning_scene::PlanningSceneConstPtr scene, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

	/**
	 * @brief Given a goal pose in the world, compute a plan
	 * @param start The start pose
	 * @param goal The goal pose
	 * @param plan The plan... filled by the planner
	 * @return True if a valid plan was found, false otherwise
	 */
	virtual bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

	virtual ~GKI3dNavPlanner()
	{
	}
	;

private:
	bool makePlan_(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, std::vector<geometry_msgs::PoseStamped>& plan);
	bool transformPoseToPlanningFrame(geometry_msgs::PoseStamped& stamped);
	unsigned char costMapCostToSBPLCost(unsigned char newcost);
	void publishStats(int solution_cost, int solution_size, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);

	bool initialized_;

	SBPLPlanner* planner_;
	Environment_xyt_3d_collisions* env_;

	std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

	double allocated_time_; /**< amount of time allowed for search */
	double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

	std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */
	std::string cost_map_topic_; /** what topic is being used for the costmap topic */

	bool forward_search_; /** whether to use forward or backward search */
	std::string primitive_filename_; /** where to find the motion primitives for the current robot */
	int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */

	unsigned char lethal_obstacle_;
	unsigned char inscribed_inflated_obstacle_;
	unsigned char sbpl_cost_multiplier_;

	costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */

	ros::Publisher plan_pub_;
	ros::Publisher stats_publisher_;

	std::vector<geometry_msgs::Point> footprint_;

};
}
;

#endif

