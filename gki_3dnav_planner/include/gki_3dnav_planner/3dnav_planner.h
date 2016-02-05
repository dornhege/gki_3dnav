#ifndef SBPL_LATTICE_PLANNER_H
#define SBPL_LATTICE_PLANNER_H

#include <iostream>
#include <vector>

using namespace std;

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "gki_3dnav_planner/environment_navxythetalat_moveit.h"
#include <sbpl/headers.h>
#include <geometry_msgs/Pose2D.h>
#include <gki_3dnav_planner/SampleValidPoses.h>
#include <nav_core/base_global_planner.h>
#include "gki_3dnav_planner/sbpl_xytheta_planner.h"

namespace gki_3dnav_planner
{

class GKI3dNavPlanner: public sbpl_xytheta_planner::SBPLXYThetaPlanner
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

    virtual EnvironmentNavXYThetaLatGeneric* createEnvironment(ros::NodeHandle & nhPriv)
    {
        // TODO
    }

    virtual bool initializeEnvironment(const std::vector<sbpl_2Dpt_t> & footprint,
            double trans_vel, double timeToTurn45Degs, const std::string & motion_primitive_filename)
    {
        // TODO
    }

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

    virtual ~GKI3dNavPlanner() {
        delete private_nh_;
    }

    /// Returns the frame that all planning is happening in.
    /**
     * This is usually the planning frame of the MoveIt planning scene. All data has to be either in this
     * frame or has to be transformed to this.
     */
    std::string getPlanningFrame() const;

protected:
    bool sampleValidPoses(gki_3dnav_planner::SampleValidPoses::Request & req, gki_3dnav_planner::SampleValidPoses::Response & resp);

private:
    bool makePlan_(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, std::vector<geometry_msgs::PoseStamped>& plan);
    // TODO move to env
    bool transformPoseToPlanningFrame(geometry_msgs::PoseStamped& stamped);
    unsigned char costMapCostToSBPLCost(unsigned char newcost);
    void publishStats(int solution_cost, int solution_size, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);

    void publish_expansions();

private:
    bool initialized_;
    ros::NodeHandle* private_nh_;

    SBPLPlanner* planner_;
    EnvironmentNavXYThetaLatMoveit* env_;

    std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

    double allocated_time_; /**< amount of time allowed for search */
    double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

    bool forward_search_; /** whether to use forward or backward search */
    std::string primitive_filename_; /** where to find the motion primitives for the current robot */
    int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */
    bool use_freespace_heuristic_;

    unsigned char lethal_obstacle_;
    unsigned char inscribed_inflated_obstacle_;
    unsigned char sbpl_cost_multiplier_;

    costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */

    ros::Publisher plan_pub_;
    ros::Publisher traj_pub_;
    ros::Publisher stats_publisher_;
    ros::Publisher expansions_publisher_;

    ros::ServiceServer srv_sample_poses_;

    std::vector<geometry_msgs::Point> footprint_;

};

}

#endif

