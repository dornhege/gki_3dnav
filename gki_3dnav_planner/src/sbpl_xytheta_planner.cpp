#include <gki_3dnav_planner/sbpl_xytheta_planner.h>
#include <nav_msgs/Path.h>
#include <gki_3dnav_planner/PlannerStats.h>
#include <sbpl/planners/planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <iomanip>
#include <angles/angles.h>
#include "color_tools/color_tools.h"

namespace sbpl_xytheta_planner
{

SBPLXYThetaPlanner::SBPLXYThetaPlanner() :
        initialized_(false), costmap_ros_(NULL), forward_search_(false), initial_epsilon_(0),
        env_(NULL), force_scratch_limit_(0), planner_(NULL), allocated_time_(0)
{
}

SBPLXYThetaPlanner::SBPLXYThetaPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
        initialized_(false), costmap_ros_(NULL)
{
    initialize(name, costmap_ros);
}

void SBPLXYThetaPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(initialized_)
        return;

    private_nh_ = new ros::NodeHandle("~/" + name);
    ros::NodeHandle nh(name);

    costmap_ros_ = costmap_ros;

    ROS_INFO("Planner Name is %s", name.c_str());

    private_nh_->param("planner_type", planner_type_, std::string("ARAPlanner"));
    private_nh_->param("allocated_time", allocated_time_, 10.0);
    private_nh_->param("initial_epsilon", initial_epsilon_, 3.0);
    private_nh_->param("forward_search", forward_search_, bool(false));
    std::string motion_primitive_filename;
    
    private_nh_->param("motion_primitive_filename", motion_primitive_filename, std::string(""));
    private_nh_->param("force_scratch_limit", force_scratch_limit_, 500);

    double trans_vel, rot_vel;
    private_nh_->param("trans_vel", trans_vel, 0.4);
    private_nh_->param("rot_vel", rot_vel, 1.3);
    bool track_expansions;
    private_nh_->param("track_expansions", track_expansions, false);

    // Environment creation and initialization
    env_ = createEnvironment(*private_nh_);
    if(env_ == NULL) {
        ROS_FATAL("Failed to create environment.");
        exit(1);
    }

    std::vector<sbpl_2Dpt_t> perimeterptsV;
    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    perimeterptsV.reserve(footprint.size());
    for(size_t ii(0); ii < footprint.size(); ++ii) {
        sbpl_2Dpt_t pt;
        pt.x = footprint[ii].x;
        pt.y = footprint[ii].y;
        perimeterptsV.push_back(pt);
    }

    bool ret = true;
    try {
        double timeToTurn45Degs = M_PI/4.0/rot_vel;
        ret = initializeEnvironment(perimeterptsV, trans_vel, timeToTurn45Degs, motion_primitive_filename);
    } catch(SBPL_Exception& e) {
        ROS_ERROR("SBPL encountered a fatal exception initializing the environment!");
        ret = false;
    }
    if(!ret) {
        ROS_ERROR("SBPL initialization failed!");
        exit(1);
    }

    // Search planner creation
    if ("ARAPlanner" == planner_type_) {
        ROS_INFO("Planning with ARA*");
        planner_ = new ARAPlanner(env_, forward_search_);
        dynamic_cast<ARAPlanner*>(planner_)->set_track_expansions(track_expansions);
    } else if ("ADPlanner" == planner_type_) {
        ROS_INFO("Planning with AD*");
        planner_ = new ADPlanner(env_, forward_search_);
    } else {
        ROS_FATAL("Unknown planner type: %s (supported: ARAPlanner or ADPlanner)", planner_type_.c_str());
        exit(1);
    }

    ROS_INFO("sbpl_xytheta_planner: Initialized successfully");
    plan_pub_ = private_nh_->advertise<nav_msgs::Path>("plan", 1);
    stats_publisher_ = private_nh_->advertise<gki_3dnav_planner::PlannerStats>("planner_stats", 10);
    traj_pub_ = private_nh_->advertise<moveit_msgs::DisplayTrajectory>("trajectory", 5);
    expansions_publisher_ = private_nh_->advertise<visualization_msgs::MarkerArray>("expansions", 3, true);

    srv_sample_poses_ = private_nh_->advertiseService("sample_valid_poses",
            &SBPLXYThetaPlanner::sampleValidPoses, this);

    srand48(time(NULL));
    initialized_ = true;
}

std::string SBPLXYThetaPlanner::getPlanningFrame() const
{
    return env_->getPlanningFrame();
}

void SBPLXYThetaPlanner::publishStats(int solution_cost, int solution_size, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
{
    gki_3dnav_planner::PlannerStats stats;
    std::vector< ::PlannerStats > planner_stats;
    planner_->get_search_stats(&planner_stats);
    for(int i = 0; i < planner_stats.size(); ++i) {
        gki_3dnav_planner::PlannerStat stat;
        stat.eps = planner_stats[i].eps;
        stat.suboptimality = planner_stats[i].suboptimality;
        stat.time = planner_stats[i].time;
        stat.g = planner_stats[i].g;
        stat.cost = planner_stats[i].cost;
        stat.expands = planner_stats[i].expands;
        stats.stats.push_back(stat);
    }
    stats_publisher_.publish(stats);
}

bool SBPLXYThetaPlanner::sampleValidPoses(gki_3dnav_planner::SampleValidPoses::Request & req,
        gki_3dnav_planner::SampleValidPoses::Response & resp)
{
    geometry_msgs::Point min, max;
    env_->getExtents(min.x, max.x, min.y, max.y);

    geometry_msgs::Pose pose;
    resp.poses.header.frame_id = getPlanningFrame();
    int numTries = 0;
    while(numTries < req.max_tries) {
        // sample pose and add to resp
        pose.position.x = min.x + (max.x - min.x) * drand48();
        pose.position.y = min.y + (max.y - min.y) * drand48();
        double theta = -M_PI + 2 * M_PI * drand48();
        pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        numTries++;
        try {
            int ret = env_->SetStart(pose.position.x, pose.position.y, theta);
            if(ret < 0)
                continue;
        } catch (SBPL_Exception& e) {
            continue;
        }

        resp.poses.poses.push_back(pose);
        if(resp.poses.poses.size() >= req.n)
            break;
    }

    return resp.poses.poses.size() >= req.n;
}

bool SBPLXYThetaPlanner::makePlan(const geometry_msgs::PoseStamped& startPose,
				  const geometry_msgs::PoseStamped& goalPose,
				  std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(!initialized_) {
        ROS_ERROR("Global planner is not initialized");
        return false;
    }
    env_->updateForPlanRequest();
    ROS_INFO("Planning frame is %s", getPlanningFrame().c_str());

    geometry_msgs::PoseStamped start = startPose;
    if(!env_->transformPoseToPlanningFrame(start)) {
        ROS_ERROR("Unable to transform start pose into planning frame");
        return false;
    }

    geometry_msgs::PoseStamped goal = goalPose;
    if(!env_->transformPoseToPlanningFrame(goal)) {
        ROS_ERROR("Unable to transform goal pose into planning frame");
        return false;
    }

    double theta_start = tf::getYaw(start.pose.orientation);
    double theta_goal = tf::getYaw(goal.pose.orientation);
    ROS_INFO("sbpl_xytheta_planner: setting start (%.2f, %.2f, %.2f deg), goal (%.2f, %.2f, %.2f deg)",
            start.pose.position.x, start.pose.position.y, angles::to_degrees(theta_start),
            goal.pose.position.x, goal.pose.position.y, angles::to_degrees(theta_goal));

    int startId = 0;

    planner_->force_planning_from_scratch();
    try {
        int ret = env_->SetStart(start.pose.position.x, start.pose.position.y, theta_start);
        startId = ret;
        if(ret < 0 || planner_->set_start(ret) == 0) {
            ROS_ERROR("ERROR: failed to set start state\n");
            return false;
        }
    } catch (SBPL_Exception& e) {
        ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
        return false;
    }

    try {
        int ret = env_->SetGoal(goal.pose.position.x, goal.pose.position.y, theta_goal);
        if(ret < 0 || planner_->set_goal(ret) == 0) {
            ROS_ERROR("ERROR: failed to set goal state\n");
            return false;
        }
    } catch (SBPL_Exception& e) {
        ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
        return false;
    }

    planner_->force_planning_from_scratch();
    // TODO also/instead update planner with changed world/states

    ROS_INFO("Start state Heur: %d", env_->GetGoalHeuristic(startId));

    ROS_DEBUG("allocated time: %.1f, initial eps: %.2f\n", allocated_time_, initial_epsilon_);
    planner_->set_initialsolution_eps(initial_epsilon_);
    planner_->set_search_mode(false);   // TODO

    ROS_DEBUG("Running planner");
    std::vector<int> solution_stateIDs;
    int solution_cost;
    try {
        env_->resetTimingStats();
        int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
        env_->printTimingStats();
        if(ret) {
            ROS_DEBUG("Solution is found\n");
        } else {
            ROS_INFO("Solution not found\n");
            publish_expansions();
            publishStats(solution_cost, 0, start, goal);
            return false;
        }
    } catch (SBPL_Exception& e) {
        ROS_ERROR("SBPL encountered a fatal exception while planning");
        return false;
    }

    ROS_DEBUG("solution length %zu", solution_stateIDs.size());

    std::vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
    try {
        env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
    } catch (SBPL_Exception& e) {
        ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
        return false;
    }

    // TODO check if the plan has zero points, add a single point to make move_base happy
    if(sbpl_path.size() == 0) {
        EnvNAVXYTHETALAT3Dpt_t s(start.pose.position.x, start.pose.position.y, theta_start);
        sbpl_path.push_back(s);
    }

    ROS_DEBUG("Plan has %zu steps.", sbpl_path.size());

    ros::Time plan_time = ros::Time::now();
    plan.clear();
    nav_msgs::Path gui_path;
    gui_path.poses.resize(sbpl_path.size());
    gui_path.header.frame_id = getPlanningFrame();
    gui_path.header.stamp = plan_time;
    for(unsigned int i = 0; i < sbpl_path.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = getPlanningFrame();

        pose.pose.position.x = sbpl_path[i].x;
        pose.pose.position.y = sbpl_path[i].y;
        pose.pose.position.z = start.pose.position.z;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(sbpl_path[i].theta);
        plan.push_back(pose);
        gui_path.poses[i] = plan[i];
    }

    plan_pub_.publish(gui_path);
    publishStats(solution_cost, sbpl_path.size(), start, goal);
    moveit_msgs::DisplayTrajectory traj = env_->pathToDisplayTrajectory(plan);
    traj_pub_.publish(traj);

    publish_expansions();
    return true;
}

void SBPLXYThetaPlanner::publish_expansions()
{
    ARAPlanner* pl = dynamic_cast<ARAPlanner*>(planner_);
    if(!pl)
        return;

    const std::vector< std::vector<int> > & gen_states = pl->get_generated_states();
    const std::vector< std::vector<int> > & exp_states = pl->get_expanded_states();

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker mark;
    mark.type = visualization_msgs::Marker::ARROW;
    mark.scale.x = 0.1;
    mark.scale.y = 0.01;
    mark.scale.z = 0.01;
    mark.color.a = 0.1;
    mark.header.frame_id = getPlanningFrame();
    color_tools::HSV hsv;
    hsv.s = 1.0;
    hsv.v = 1.0;
    for(int iteration = 0; iteration < exp_states.size(); iteration++) {
        std::stringstream ss;
        ss << "expansions_" << std::setfill('0') << std::setw(2) << iteration;
        mark.ns = ss.str();
        hsv.h = 300.0 * (1.0 - 1.0*iteration/exp_states.size());
        color_tools::convert(hsv, mark.color);
        int state = 0;
        mark.action = visualization_msgs::Marker::ADD;
        for(; state < exp_states[iteration].size(); state++) {
            mark.id = state;
            mark.pose = env_->poseFromStateID(exp_states[iteration][state]);
            mark.pose.position.z += 0.3 * (1.0 - 1.0*iteration/exp_states.size());
            ma.markers.push_back(mark);
        }
        mark.action = visualization_msgs::Marker::DELETE;
        for(; state < 1000; state++) {
            mark.id = state;
            ma.markers.push_back(mark);
        }
    }
    for(int iteration = 0; iteration < gen_states.size(); iteration++) {
        std::stringstream ss;
        ss << "generated_" << std::setfill('0') << std::setw(2) << iteration;
        mark.ns = ss.str();
        hsv.h = 300.0 * (1.0 - 1.0*iteration/gen_states.size());
        color_tools::convert(hsv, mark.color);
        int state = 0;
        mark.action = visualization_msgs::Marker::ADD;
        for(; state < gen_states[iteration].size(); state++) {
            mark.id = state;
            mark.pose = env_->poseFromStateID(gen_states[iteration][state]);
            mark.pose.position.z += 0.3 * (1.0 - 1.0*iteration/gen_states.size());
            ma.markers.push_back(mark);
        }
        mark.action = visualization_msgs::Marker::DELETE;
        for(; state < 1000; state++) {
            mark.id = state;
            ma.markers.push_back(mark);
        }
    }

    expansions_publisher_.publish(ma);
}

}

