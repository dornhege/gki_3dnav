#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <gki_3dnav_planner/PlannerStats.h>
#include <yaml-cpp/emitter.h>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

std::vector< std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> > poseQueries;
std::vector<gki_3dnav_planner::PlannerStats> g_Stats;
bool g_StatsReceived;

bool g_ReverseQueries = false;

void posesCallback(const geometry_msgs::PoseArray & pa)
{
    ROS_INFO("Got PA size: %zu", pa.poses.size());

    geometry_msgs::PoseStamped psStart;
    geometry_msgs::PoseStamped psGoal;
    psStart.header = pa.header;
    psGoal.header = pa.header;
    for(int i = 0; i < pa.poses.size(); ++i) {
        // no reverse queries?
        int j = i + 1;
        if(g_ReverseQueries)
            j = 0;
        for(; j < pa.poses.size(); ++j) {
            if(i == j)
                continue;
            psStart.pose = pa.poses[i];
            psGoal.pose = pa.poses[j];
            if(hypot(psStart.pose.position.x - psGoal.pose.position.x,
                        psStart.pose.position.y - psGoal.pose.position.y) < 0.5)
                continue;
            poseQueries.push_back(std::make_pair(psStart, psGoal));
        }
    }
}

void statsCallback(const gki_3dnav_planner::PlannerStats & stats)
{
    g_Stats.push_back(stats);
    g_StatsReceived = true;
}

void collectData()
{
    ROS_INFO("Got %zu queries", poseQueries.size());
    g_Stats.clear();
    for(int i = 0; i < poseQueries.size(); ++i) {
        if(!ros::ok())
            break;
        g_StatsReceived = false;
        nav_msgs::GetPlan srv;
        srv.request.start = poseQueries[i].first;
        srv.request.goal = poseQueries[i].second;
        bool err = false;
        ROS_INFO("Calling plan for query: %d", i);
        if(!ros::service::call("/move_base_node/make_plan", srv) || srv.response.plan.poses.empty()) {
            ROS_ERROR("Could not plan for %d", i);
            // FIXME also no plan found
            err = true;
        }
        int count = 0;
        while(!g_StatsReceived) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
            count++;
            if(count > 10) {
                if(err) {
                    ROS_INFO("Inserting empty stats");
                    g_Stats.push_back(gki_3dnav_planner::PlannerStats());
                    break;
                }
            }
        }
    }
}

template <typename T>
void writeKeyVal(YAML::Emitter & em, const std::string & key, const T& val)
{
    em << YAML::Key;
    em << key;
    em << YAML::Value;
    em << val;
}

void yamlPose(YAML::Emitter & emitter, const geometry_msgs::PoseStamped & ps)
{
    emitter << YAML::BeginMap;
    writeKeyVal(emitter, "frame_id", ps.header.frame_id);
    writeKeyVal(emitter, "x", ps.pose.position.x);
    writeKeyVal(emitter, "y", ps.pose.position.y);
    writeKeyVal(emitter, "z", ps.pose.position.z);
    writeKeyVal(emitter, "qx", ps.pose.orientation.x);
    writeKeyVal(emitter, "qy", ps.pose.orientation.y);
    writeKeyVal(emitter, "qz", ps.pose.orientation.z);
    writeKeyVal(emitter, "qw", ps.pose.orientation.w);
    emitter << YAML::EndMap;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "evaluate_pose_queries");
    ros::NodeHandle nh;

    ros::Subscriber subPoses = nh.subscribe("valid_poses", 3, posesCallback);
    ros::Subscriber subStats = nh.subscribe("/move_base_node/GKI3dNavPlanner/planner_stats", 10, statsCallback);

    ros::Rate rate(10.0);
    while(ros::ok() && poseQueries.empty()) {
        ros::spinOnce();
        rate.sleep();
    }

    std::vector<gki_3dnav_planner::PlannerStats> freespace_stats;
    ros::param::set("/move_base_node/GKI3dNavPlanner/use_freespace_heuristic", true);
    collectData();
    freespace_stats = g_Stats;

    std::vector<gki_3dnav_planner::PlannerStats> no_freespace_stats;
    ros::param::set("/move_base_node/GKI3dNavPlanner/use_freespace_heuristic", false);
    collectData();
    no_freespace_stats = g_Stats;

    if(poseQueries.size() != no_freespace_stats.size()) {
        ROS_ERROR("poseQueries size %zu != no_freespace_stats size %zu", poseQueries.size(), no_freespace_stats.size());
    }
    if(poseQueries.size() != freespace_stats.size()) {
        ROS_ERROR("poseQueries size %zu != no_freespace_stats %zu", poseQueries.size(), freespace_stats.size());
    }

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key;
    emitter << "no_freespace_stats";
    emitter << YAML::Value;
    emitter << YAML::BeginSeq;
    int stat_index = 0;
    forEach(const gki_3dnav_planner::PlannerStats & ps, no_freespace_stats) {
        emitter << YAML::BeginMap;
        if(stat_index < poseQueries.size()) {
            emitter << YAML::Key;
            emitter << "start_pose";
            emitter << YAML::Value;
            yamlPose(emitter, poseQueries[stat_index].first);
            emitter << YAML::Key;
            emitter << "goal_pose";
            emitter << YAML::Value;
            yamlPose(emitter, poseQueries[stat_index].second);
        }
        emitter << YAML::Key;
        emitter << "planner_stats";
        emitter << YAML::Value;
        emitter << YAML::BeginSeq;
        forEach(const gki_3dnav_planner::PlannerStat & pss, ps.stats) {
            emitter << YAML::BeginMap;
            writeKeyVal(emitter, "eps", pss.eps);
            writeKeyVal(emitter, "suboptimality", pss.suboptimality);
            writeKeyVal(emitter, "g", pss.g);
            writeKeyVal(emitter, "cost", pss.cost);
            writeKeyVal(emitter, "time", pss.time);
            writeKeyVal(emitter, "expands", pss.expands);
            emitter << YAML::EndMap;
        }
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;
        stat_index++;
    }
    emitter << YAML::EndSeq;

    emitter << YAML::Key;
    emitter << "freespace_stats";
    emitter << YAML::Value;
    emitter << YAML::BeginSeq;
    forEach(const gki_3dnav_planner::PlannerStats & ps, freespace_stats) {
        emitter << YAML::BeginMap;
        if(stat_index < poseQueries.size()) {
            emitter << YAML::Key;
            emitter << "start_pose";
            emitter << YAML::Value;
            yamlPose(emitter, poseQueries[stat_index].first);
            emitter << YAML::Key;
            emitter << "goal_pose";
            emitter << YAML::Value;
            yamlPose(emitter, poseQueries[stat_index].second);
        }
        emitter << YAML::Key;
        emitter << "planner_stats";
        emitter << YAML::Value;
        emitter << YAML::BeginSeq;
        forEach(const gki_3dnav_planner::PlannerStat & pss, ps.stats) {
            emitter << YAML::BeginMap;
            writeKeyVal(emitter, "eps", pss.eps);
            writeKeyVal(emitter, "suboptimality", pss.suboptimality);
            writeKeyVal(emitter, "g", pss.g);
            writeKeyVal(emitter, "cost", pss.cost);
            writeKeyVal(emitter, "time", pss.time);
            writeKeyVal(emitter, "expands", pss.expands);
            emitter << YAML::EndMap;
        }
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;

    FILE* f = fopen("evaluate_pose_queries.yaml", "w");
    if(!f)
        return 1;
    fprintf(f, "%s", emitter.c_str());
    fclose(f);
}

