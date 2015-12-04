/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *
 * Author: Mike Phillips
 *********************************************************************/

#include <gki_3dnav_planner/flourish_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
//#include <sbpl_lattice_planner/SBPLLatticePlannerStats.h>

#include <costmap_2d/inflation_layer.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;
using namespace ros;

PLUGINLIB_EXPORT_CLASS(flourish_planner::FlourishPlanner, nav_core::BaseGlobalPlanner);

namespace flourish_planner
{

  class LatticeSCQ: public StateChangeQuery
  {
  public:
    LatticeSCQ(EnvironmentNavXYThetaLatFlourish* env, std::vector<nav2dcell_t> const & changedcellsV) :
      env_(env), changedcellsV_(changedcellsV)
    {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const
    {
      if (predsOfChangedCells_.empty() && !changedcellsV_.empty())
	env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const
    {
      if (succsOfChangedCells_.empty() && !changedcellsV_.empty())
	env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
      return &succsOfChangedCells_;
    }

    EnvironmentNavXYThetaLatFlourish * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
  };

  FlourishPlanner::FlourishPlanner() :
    initialized_(false),
    costmap_ros_(NULL),
    forward_search_(false),
    initial_epsilon_(0),
    env_(NULL),
    sbpl_cost_multiplier_(0),
    force_scratch_limit_(0),
    lethal_obstacle_(0),
    inscribed_inflated_obstacle_(0),
    planner_(NULL),
    allocated_time_(0)
  {
  }

  FlourishPlanner::FlourishPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
    initialized_(false), costmap_ros_(NULL)
  {
    initialize(name, costmap_ros);
  }

  void FlourishPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if (!initialized_)
      {
	ros::NodeHandle private_nh("~/" + name);
	ros::NodeHandle nh(name);

	ROS_INFO("Name is %s", name.c_str());

	private_nh.param("planner_type", planner_type_, string("ARAPlanner"));
	private_nh.param("allocated_time", allocated_time_, 10.0);
	private_nh.param("initial_epsilon", initial_epsilon_, 3.0);
	private_nh.param("environment_type", environment_type_, string("XYThetaLattice"));
	private_nh.param("forward_search", forward_search_, bool(false));
	private_nh.param("primitive_filename", primitive_filename_, string(""));
	private_nh.param("force_scratch_limit", force_scratch_limit_, 500);

	double nominalvel_mpersecs, timetoturn45degsinplace_secs;
	private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
	private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

	int lethal_obstacle;
	private_nh.param("lethal_obstacle", lethal_obstacle, 20);
	lethal_obstacle_ = (unsigned char) lethal_obstacle;
	inscribed_inflated_obstacle_ = lethal_obstacle_ - 1;
	sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE / inscribed_inflated_obstacle_ + 1);
	ROS_DEBUG("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz", lethal_obstacle, inscribed_inflated_obstacle_, sbpl_cost_multiplier_);

	costmap_ros_ = costmap_ros;

	std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

	env_ = new EnvironmentNavXYThetaLatFlourish(private_nh, costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getOriginY());

	// check if the costmap has an inflation layer
	// Warning: footprint updates after initialization are not supported here
	unsigned char cost_possibly_circumscribed_tresh = 0;
	for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::const_iterator layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin(); layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
	  {
	    boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer = boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
	    if (!inflation_layer)
	      continue;

	    cost_possibly_circumscribed_tresh = inflation_layer->computeCost(costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
	  }

	if (!env_->SetEnvParameter("cost_inscribed_thresh", costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
	  {
	    ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
	    exit(1);
	  }
	if (!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(cost_possibly_circumscribed_tresh)))
	  {
	    ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
	    exit(1);
	  }
	int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);
	vector<sbpl_2Dpt_t> perimeterptsV;
	perimeterptsV.reserve(footprint.size());
	for (size_t ii(0); ii < footprint.size(); ++ii)
	  {
	    sbpl_2Dpt_t pt;
	    pt.x = footprint[ii].x;
	    pt.y = footprint[ii].y;
	    perimeterptsV.push_back(pt);
	    std::cout << pt.x << "," << pt.y << " ";
	  }

	bool ret;
	std::cout << costmap_ros_->getCostmap()->getSizeInCellsX() << " " << // width
				      costmap_ros_->getCostmap()->getSizeInCellsY() << " " << // height
				       " " << costmap_ros_->getCostmap()->getResolution() << " " << nominalvel_mpersecs << " " << timetoturn45degsinplace_secs << " " << obst_cost_thresh << " " << primitive_filename_.c_str();
	try
	  {
	    FILE* bla = fopen(primitive_filename_.c_str(), "r");
	    if(bla != NULL){
	      if(env_->tryToReadPrims(bla) == false)
		{
		  SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
		  ROS_ERROR("sbpl exception");
		}else{
		  ROS_ERROR("sbpl non exception");
	      }
	      std::cout << "read file" << std::endl;
	    }else{
	      std::cout << "couldn't read file" << std::endl;
	    }

	    fclose(bla);
	    ret = env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(), // width
				      costmap_ros_->getCostmap()->getSizeInCellsY(), // height
				      0, // mapdata
				      0, 0, 0, // start (x, y, theta, t)
				      0, 0, 0, // goal (x, y, theta)
				      0, 0, 0, //goal tolerance
				      perimeterptsV, costmap_ros_->getCostmap()->getResolution(), nominalvel_mpersecs, timetoturn45degsinplace_secs, obst_cost_thresh, primitive_filename_.c_str());
	  } catch (SBPL_Exception* e)
	  {

	    ROS_ERROR("SBPL encountered a fatal exception! %s", e->what());
	    ret = false;
	  }
	if (!ret)
	  {
	    ROS_ERROR("SBPL initialization failed!");
	    exit(1);
	  }
	for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ++ix)
	  for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY(); ++iy)
	    env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy)));

	if ("ARAPlanner" == planner_type_)
	  {
	    ROS_INFO("Planning with ARA*");
	    planner_ = new ARAPlanner(env_, forward_search_);
	  }
	else if ("ADPlanner" == planner_type_)
	  {
	    ROS_INFO("Planning with AD*");
	    planner_ = new ADPlanner(env_, forward_search_);
	  }
	else
	  {
	    ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
	    exit(1);
	  }

	ROS_INFO("[gki_3dnav_planner] Initialized successfully");
	plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
	//stats_publisher_ = private_nh.advertise<sbpl_lattice_planner::SBPLLatticePlannerStats>("sbpl_lattice_planner_stats", 1);

	initialized_ = true;
      }
  }

  //Taken from Sachin's sbpl_cart_planner
  //This rescales the costmap according to a rosparam which sets the obstacle cost
  unsigned char FlourishPlanner::costMapCostToSBPLCost(unsigned char newcost)
  {
    if (newcost == costmap_2d::LETHAL_OBSTACLE)
      return lethal_obstacle_;
    else if (newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return inscribed_inflated_obstacle_;
    else if (newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
      return 0;
    else
      return (unsigned char) (newcost / sbpl_cost_multiplier_ + 0.5);
  }

  void FlourishPlanner::publishStats(int solution_cost, int solution_size, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
  {
    // Fill up statistics and publish
    //	sbpl_lattice_planner::SBPLLatticePlannerStats stats;
    //	stats.initial_epsilon = initial_epsilon_;
    //	stats.plan_to_first_solution = false;
    //	stats.final_number_of_expands = planner_->get_n_expands();
    //	stats.allocated_time = allocated_time_;
    //
    //	stats.time_to_first_solution = planner_->get_initial_eps_planning_time();
    //	stats.actual_time = planner_->get_final_eps_planning_time();
    //	stats.number_of_expands_initial_solution = planner_->get_n_expands_init_solution();
    //	stats.final_epsilon = planner_->get_final_epsilon();
    //
    //	stats.solution_cost = solution_cost;
    //	stats.path_size = solution_size;
    //	stats.start = start;
    //	stats.goal = goal;
    //	stats_publisher_.publish(stats);
  }

  bool FlourishPlanner::transformPoseToPlanningFrame(geometry_msgs::PoseStamped& stamped)
  {
    planning_scene::PlanningSceneConstPtr scene = env_->getPlanningScene();
    if (! scene->getTransforms().sameFrame(scene->getPlanningFrame(), stamped.header.frame_id))
      {
	if (! scene->getTransforms().canTransform(stamped.header.frame_id))
	  {
	    return false;
	  }
	Eigen::Affine3d original_goal;
	tf::poseMsgToEigen(stamped.pose, original_goal);
	Eigen::Affine3d map_goal;
	scene->getTransforms().transformPose(stamped.header.frame_id, original_goal, map_goal);
	tf::poseEigenToMsg(map_goal, stamped.pose);
	stamped.header.frame_id = scene->getPlanningFrame();
      }
    return true;
  }

  bool FlourishPlanner::makePlan(planning_scene::PlanningSceneConstPtr scene, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    const robot_state::RobotState& state = scene->getCurrentState();
    geometry_msgs::PoseStamped start;
    start.pose.position.x = state.getVariablePosition("world_joint/x");
    start.pose.position.y = state.getVariablePosition("world_joint/y");
    start.pose.position.z = 0;
    start.pose.orientation = tf::createQuaternionMsgFromYaw(state.getVariablePosition("world_joint/theta"));
    start.header.frame_id = scene->getPlanningFrame();
    env_->clear_full_body_collision_infos();
    //env_->update_planning_scene(scene);
    //env_->publish_planning_scene();
    return makePlan_(start, goal, plan);
  }
  bool FlourishPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    //env_->clear_full_body_collision_infos();
    //env_->update_planning_scene();
    //env_->publish_planning_scene();
    return makePlan_(start, goal, plan);
  }

  bool FlourishPlanner::makePlan_(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    if (!initialized_)
      {
	ROS_ERROR("Global planner is not initialized");
	return false;
      }
    /*if (! transformPoseToPlanningFrame(start))
      {
	ROS_ERROR("Unable to transform start pose into planning frame");
	return false;
      }
    if (! transformPoseToPlanningFrame(goal))
      {
	ROS_ERROR("Unable to transform goal pose into planning frame");
	return false;
	}*/

    ROS_INFO("[gki_3dnav_planner] getting start point (%g,%g) goal point (%g,%g)", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
    double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

    planner_->force_planning_from_scratch();
    try
      {
	int ret = env_->SetStart(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
	if (ret < 0 || planner_->set_start(ret) == 0)
	  {
	    ROS_ERROR("ERROR: failed to set start state\n");
	    return false;
	  }
      } catch (SBPL_Exception& e)
      {
	ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
	return false;
      }

    try
      {
	int ret = env_->SetGoal(goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_goal);
	if (ret < 0 || planner_->set_goal(ret) == 0)
	  {
	    ROS_ERROR("ERROR: failed to set goal state\n");
	    return false;
	  }
      } catch (SBPL_Exception& e)
      {
	ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
	return false;
      }

    int offOnCount = 0;
    int onOffCount = 0;
    int allCount = 0;
    vector<nav2dcell_t> changedcellsV;

    for (unsigned int ix = 0; ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ix++)
      {
	for (unsigned int iy = 0; iy < costmap_ros_->getCostmap()->getSizeInCellsY(); iy++)
	  {

	    unsigned char oldCost = env_->GetMapCost(ix, iy);
	    unsigned char newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy));

	    if (oldCost == newCost)
	      continue;

	    allCount++;

	    //first case - off cell goes on

	    if ((oldCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && oldCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
		&& (newCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || newCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
	      {
		offOnCount++;
	      }

	    if ((oldCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || oldCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
		&& (newCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && newCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
	      {
		onOffCount++;
	      }
	    env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy)));

	    nav2dcell_t nav2dcell;
	    nav2dcell.x = ix;
	    nav2dcell.y = iy;
	    changedcellsV.push_back(nav2dcell);
	  }
      }

    try
      {
	if (!changedcellsV.empty())
	  {
	    StateChangeQuery* scq = new LatticeSCQ(env_, changedcellsV);
	    planner_->costs_changed(*scq);
	    delete scq;
	  }

	if (allCount > force_scratch_limit_)
	  planner_->force_planning_from_scratch();
      } catch (SBPL_Exception& e)
      {
	ROS_ERROR("SBPL failed to update the costmap");
	return false;
      }

    //setting planner parameters
    ROS_DEBUG("allocated:%f, init eps:%f\n", allocated_time_, initial_epsilon_);
    planner_->set_initialsolution_eps(initial_epsilon_);
    planner_->set_search_mode(false);

    ROS_DEBUG("[gki_3dnav_planner] run planner");
    vector<int> solution_stateIDs;
    int solution_cost;
    try
      {
	int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
	if (ret)
	  ROS_DEBUG("Solution is found\n");
	else
	  {
	    ROS_INFO("Solution not found\n");
	    publishStats(solution_cost, 0, start, goal);
	    return false;
	  }
      } catch (SBPL_Exception& e)
      {
	ROS_ERROR("SBPL encountered a fatal exception while planning");
	return false;
      }

    ROS_DEBUG("size of solution=%d", (int )solution_stateIDs.size());

    vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
    try
      {
	env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
      } catch (SBPL_Exception& e)
      {
	ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
	return false;
      }
    // if the plan has zero points, add a single point to make move_base happy
    if (sbpl_path.size() == 0)
      {
	EnvNAVXYTHETALAT3Dpt_t s(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
	sbpl_path.push_back(s);
      }

    ROS_DEBUG("Plan has %d points.\n", (int )sbpl_path.size());
    ros::Time plan_time = ros::Time::now();

    plan.clear();
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(sbpl_path.size());
    gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    gui_path.header.stamp = plan_time;
    for (unsigned int i = 0; i < sbpl_path.size(); i++)
      {
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = plan_time;
	pose.header.frame_id = costmap_ros_->getGlobalFrameID();

	pose.pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
	pose.pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
	pose.pose.position.z = start.pose.position.z;

	tf::Quaternion temp;
	temp.setRPY(0, 0, sbpl_path[i].theta);
	pose.pose.orientation.x = temp.getX();
	pose.pose.orientation.y = temp.getY();
	pose.pose.orientation.z = temp.getZ();
	pose.pose.orientation.w = temp.getW();

	plan.push_back(pose);

	gui_path.poses[i] = plan[i];
      }
    plan_pub_.publish(gui_path);
    publishStats(solution_cost, sbpl_path.size(), start, goal);

    // DeBUG
    env_->publish_expanded_states();
    return true;
  }
}
;
