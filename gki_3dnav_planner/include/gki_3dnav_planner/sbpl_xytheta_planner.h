#ifndef SBPL_XYTHETA_PLANNER_H
#define SBPL_XYTHETA_PLANNER_H

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>

#include <sbpl/headers.h>
#include <gki_3dnav_planner/SampleValidPoses.h>

#include "gki_3dnav_planner/environment_navxythetalat_generic.h"

namespace sbpl_xytheta_planner
{

class SBPLXYThetaPlanner: public nav_core::BaseGlobalPlanner
{
public:
    SBPLXYThetaPlanner();
    SBPLXYThetaPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /// Main query from move_base
    virtual bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    virtual ~SBPLXYThetaPlanner() {
        delete private_nh_;
    }

    /// Returns the frame that all planning is happening in.
    /**
     * This is usually the planning frame of the environment. All data has to be either in this
     * frame or has to be transformed to this.
     */
    virtual std::string getPlanningFrame() const;

protected:
    virtual bool sampleValidPoses(gki_3dnav_planner::SampleValidPoses::Request & req, gki_3dnav_planner::SampleValidPoses::Response & resp);

    /// Create a custom environment for this planner.
    virtual EnvironmentNavXYThetaLatGeneric* createEnvironment(ros::NodeHandle & nhPriv) = 0;
    virtual bool initializeEnvironment(const std::vector<sbpl_2Dpt_t> & footprint,
				       double trans_vel, double timeToTurn45Degs, const std::string & motion_primitive_filename) = 0;


private:
    virtual void publishStats(int solution_cost, int solution_size, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);
    virtual void publish_expansions();

private:
    bool initialized_;
    ros::NodeHandle* private_nh_;

    SBPLPlanner* planner_;
    EnvironmentNavXYThetaLatGeneric* env_;

    std::string planner_type_;  ///< ARAPlanner or ADPlanner
    double allocated_time_;
    double initial_epsilon_;

    bool forward_search_;       /// TODO check
    int force_scratch_limit_;   /// TODO check

    costmap_2d::Costmap2DROS* costmap_ros_;

    ros::Publisher plan_pub_;
    ros::Publisher traj_pub_;
    ros::Publisher stats_publisher_;
    ros::Publisher expansions_publisher_;

    ros::ServiceServer srv_sample_poses_;
};

}

#endif

