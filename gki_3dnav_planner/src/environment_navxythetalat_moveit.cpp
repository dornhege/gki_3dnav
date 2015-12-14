#include <gki_3dnav_planner/environment_navxythetalat_moveit.h>
#include <geometry_msgs/PoseArray.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/key.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/2Dgridsearch.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>
#include <tf/transform_datatypes.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

EnvironmentNavXYThetaLatMoveit::EnvironmentNavXYThetaLatMoveit(ros::NodeHandle & nhPriv,
        double costmapOffsetX, double costmapOffsetY) :
    costmapOffsetX(costmapOffsetX), costmapOffsetY(costmapOffsetY)
{
    nhPriv.param("scene_update_name", scene_update_name, move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    nhPriv.getParam("allowed_collision_links", allowed_collision_links);

    std::string freespace_heuristic_costmap_file;
    nhPriv.getParam("freespace_heuristic_costmap", freespace_heuristic_costmap_file);
    if(freespace_heuristic_costmap_file.empty()) {
        freespace_heuristic_costmap = NULL;
    } else {
        freespace_heuristic_costmap = new freespace_mechanism_heuristic::HeuristicCostMap(
                freespace_heuristic_costmap_file, freespace_mechanism_heuristic::HeuristicCostMap::OutOfMapMaxCost);
    }

    update_planning_scene();

    planning_scene_publisher = nhPriv.advertise<moveit_msgs::PlanningScene>("planning_scene_3dnav", 1, true);
    pose_array_publisher = nhPriv.advertise<geometry_msgs::PoseArray>("expanded_states", 1, true);
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNavXYThetaLatMoveit::SetGoal(double x_m, double y_m, double theta_rad)
{
    if(EnvironmentNAVXYTHETALAT::SetGoal(x_m, y_m, theta_rad) == -1)
        return -1;

    int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);
    EnvNAVXYTHETALATHashEntry_t* OutHashEntry = (this->*GetHashEntry)(x, y, theta);
    ROS_ASSERT(OutHashEntry != NULL);   // should have been created by parent
    if(in_full_body_collision(OutHashEntry)) {
        SBPL_ERROR("ERROR: goal state in collision\n", x, y);
        EnvNAVXYTHETALAT.goalstateid = -1;
        return -1;
    }

    return EnvNAVXYTHETALAT.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNavXYThetaLatMoveit::SetStart(double x_m, double y_m, double theta_rad)
{
    if(EnvironmentNAVXYTHETALAT::SetStart(x_m, y_m, theta_rad) == -1)
        return -1;

    int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);
    EnvNAVXYTHETALATHashEntry_t* OutHashEntry = (this->*GetHashEntry)(x, y, theta);
    ROS_ASSERT(OutHashEntry != NULL);
    if (in_full_body_collision(OutHashEntry)) {
        SBPL_ERROR("ERROR: start state in collision\n", x, y);
        EnvNAVXYTHETALAT.startstateid = -1;
        return -1;
    }

    return EnvNAVXYTHETALAT.startstateid;
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNavXYThetaLatMoveit::CreateNewHashEntry_lookup(int X, int Y, int Theta)
{
    // the CreateNewHashEntry... functions always create a new entry, so we can assume that happens and
    // add the full_body_collision_infos entry now.
    // Do this before, instead of after the call as exceptions will appear only after a StateID2CoordTable
    // entry was created, so that at least StateID2CoordTable and full_body_collision_infos are 
    // consistent.
    full_body_collision_infos.push_back(FullBodyCollisionInfo());
    EnvNAVXYTHETALATHashEntry_t* he = EnvironmentNAVXYTHETALAT::CreateNewHashEntry_lookup(X, Y, Theta);

    return he;
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNavXYThetaLatMoveit::CreateNewHashEntry_hash(int X, int Y, int Theta)
{
    full_body_collision_infos.push_back(FullBodyCollisionInfo());
    EnvNAVXYTHETALATHashEntry_t* he = EnvironmentNAVXYTHETALAT::CreateNewHashEntry_hash(X, Y, Theta);

    return he;
}

int EnvironmentNavXYThetaLatMoveit::GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action)
{
    int cost = EnvironmentNAVXYTHETALAT::GetActionCost(SourceX, SourceY, SourceTheta, action);
    if(cost >= INFINITECOST)
        return cost;

    // full body check to maybe disregard state
    int endX = SourceX + action->dX;
    int endY = SourceY + action->dY;
    int endTheta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if((OutHashEntry = (this->*GetHashEntry)(endX, endY, endTheta)) == NULL) {
        // might have to create a end entry
        OutHashEntry = (this->*CreateNewHashEntry)(endX, endY, endTheta);
    }
    if(in_full_body_collision(OutHashEntry))
        return INFINITECOST;

    return cost;
}


// PlanningScene handling

void EnvironmentNavXYThetaLatMoveit::update_planning_scene()
{
    scene_monitor->requestPlanningSceneState(scene_update_name);
    planning_scene_monitor::LockedPlanningSceneRO ps(scene_monitor);
    scene = planning_scene::PlanningScene::clone(ps);
    // FIXME for now clone the whole scene instead of keeping the LockedPlanningSceneRO
    // and returning that in getPlanningScene() dependent on a manual one being set
    // in update_planning_scene overload or not.

    collision_detection::AllowedCollisionMatrix & allowed_collisions = scene->getAllowedCollisionMatrixNonConst();
    forEach(const std::string & cl, allowed_collision_links)
        allowed_collisions.setDefaultEntry(cl, true);
}

void EnvironmentNavXYThetaLatMoveit::update_planning_scene(planning_scene::PlanningSceneConstPtr scn)
{
    this->scene = planning_scene::PlanningScene::clone(scn);

    collision_detection::AllowedCollisionMatrix & allowed_collisions = scene->getAllowedCollisionMatrixNonConst();
    forEach(const std::string & cl, allowed_collision_links)
        allowed_collisions.setDefaultEntry(cl, true);
}

planning_scene::PlanningSceneConstPtr EnvironmentNavXYThetaLatMoveit::getPlanningScene()
{
    return scene;
}

void EnvironmentNavXYThetaLatMoveit::publish_planning_scene()
{
    moveit_msgs::PlanningScene msg;
    getPlanningScene()->getPlanningSceneMsg(msg);
    planning_scene_publisher.publish(msg);
}


void EnvironmentNavXYThetaLatMoveit::clear_full_body_collision_infos()
{
    for (size_t i = 0; i < full_body_collision_infos.size(); i++)
    {
        full_body_collision_infos[i].initialized = false;
    }
}


void EnvironmentNavXYThetaLatMoveit::publish_expanded_states()
{
    geometry_msgs::PoseArray msg;
    msg.header.frame_id = getPlanningScene()->getPlanningFrame();
    for (size_t id = 0; id < full_body_collision_infos.size(); id++)
    {
        if (full_body_collision_infos[id].initialized && ! full_body_collision_infos[id].collision)
        {
            EnvNAVXYTHETALATHashEntry_t* state = StateID2CoordTable[id];
            sbpl_xy_theta_pt_t coords = discreteToContinuous(state->X, state->Y, state->Theta);
            msg.poses.push_back(geometry_msgs::Pose());
            geometry_msgs::Pose& pose = msg.poses.back();
            pose.position.x = coords.x + costmapOffsetX;
            pose.position.y = coords.y + costmapOffsetY;
            pose.position.z = 0;
            pose.orientation = tf::createQuaternionMsgFromYaw(coords.theta);
        }
    }
    pose_array_publisher.publish(msg);
}

sbpl_xy_theta_pt_t EnvironmentNavXYThetaLatMoveit::discreteToContinuous(int x, int y, int theta)
{
    sbpl_xy_theta_pt_t pose;
    pose.x = DISCXY2CONT(x, EnvNAVXYTHETALATCfg.cellsize_m);
    pose.y = DISCXY2CONT(y, EnvNAVXYTHETALATCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(theta, EnvNAVXYTHETALATCfg.NumThetaDirs);
    return pose;
}

bool EnvironmentNavXYThetaLatMoveit::in_full_body_collision(EnvNAVXYTHETALATHashEntry_t* state)
{
    return get_full_body_collision_info(state).collision;
}

const EnvironmentNavXYThetaLatMoveit::FullBodyCollisionInfo& EnvironmentNavXYThetaLatMoveit::get_full_body_collision_info(EnvNAVXYTHETALATHashEntry_t* state)
{
    ROS_ASSERT_MSG(full_body_collision_infos.size() > state->stateID, "full_body_collision: state_id mismatch!");
    if (! full_body_collision_infos[state->stateID].initialized)
    {
        sbpl_xy_theta_pt_t pose = discreteToContinuous(state->X, state->Y, state->Theta);
        // Static uses this to get an initialized state from the scene
        // that we can subsequently change
        static robot_state::RobotState robot_state = scene->getCurrentState();
        robot_state.setVariablePosition("world_joint/x", pose.x + costmapOffsetX);
        robot_state.setVariablePosition("world_joint/y", pose.y + costmapOffsetY);
        robot_state.setVariablePosition("world_joint/theta", pose.theta);
        robot_state.update();
        full_body_collision_infos[state->stateID].initialized = true;
        full_body_collision_infos[state->stateID].collision = getPlanningScene()->isStateColliding(robot_state);
        //ROS_INFO("get_full_body_collision_info for (%.2f, %.2f, %.2f) = %d",
        //        pose.x + costmapOffsetX, pose.y + costmapOffsetY,
        //        pose.theta, full_body_collision_infos[state->stateID].collision);
    }
    return full_body_collision_infos[state->stateID];
}

int EnvironmentNavXYThetaLatMoveit::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    int heur = EnvironmentNAVXYTHETALAT::GetFromToHeuristic(FromStateID, ToStateID);

    EnvNAVXYTHETALATHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
    EnvNAVXYTHETALATHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];

    int dx = ToHashEntry->X - FromHashEntry->X;
    int dy = ToHashEntry->Y - FromHashEntry->Y;

    int hfs = 0;
    if(useFreespaceHeuristic_ && freespace_heuristic_costmap)
        hfs = freespace_heuristic_costmap->getCost(dx, dy, FromHashEntry->Theta, ToHashEntry->Theta);

    count++;
    if(count % 1000 == 0) {
        printf("Cur: %d Old: %d, Total Heur: %d, Old used: %d, New used: %d, impr %.2f\n", 
                hfs, heur,
                count, past, count - past, (double)(count-past)/count*100.0);
    }
    if(hfs > heur)
        return hfs;
    past++;
    return heur;
}

int EnvironmentNavXYThetaLatMoveit::GetStartHeuristic(int stateID)
{
    int heur = EnvironmentNAVXYTHETALAT::GetStartHeuristic(stateID);

    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    int dx = EnvNAVXYTHETALATCfg.StartX_c - HashEntry->X;
    int dy = EnvNAVXYTHETALATCfg.StartY_c - HashEntry->Y;
    dx = -dx;   // FIXME this is supposed to be FROM start TO stateID, not vice versa
    dy = -dy;
    int endTh = HashEntry->Theta;

    int hfs = 0;
    if(useFreespaceHeuristic_ && freespace_heuristic_costmap)
        hfs = freespace_heuristic_costmap->getCost(dx, dy, EnvNAVXYTHETALATCfg.StartTheta, endTh);

    count++;
    if(count % 1000 == 0) {
        printf("Cur: %d Old: %d, Total Heur: %d, Old used: %d, New used: %d, impr %.2f\n", 
                hfs, heur,
                count, past, count - past, (double)(count-past)/count*100.0);
    }
    if(hfs > heur)
        return hfs;
    past++;
    return heur;
}

int EnvironmentNavXYThetaLatMoveit::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    //computes distances from start state that is grid2D, so it is EndX_c EndY_c
    int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); 
    int hEuclid = (int)(NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(HashEntry->X, HashEntry->Y,
                EnvNAVXYTHETALATCfg.EndX_c,
                EnvNAVXYTHETALATCfg.EndY_c));
    //define this function if it is used in the planner (heuristic backward search would use it)
    int heur = (int)(((double)__max(h2D, hEuclid)) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);

    int dx = EnvNAVXYTHETALATCfg.EndX_c - HashEntry->X;
    int dy = EnvNAVXYTHETALATCfg.EndY_c - HashEntry->Y;
    int startTh = HashEntry->Theta;

    int hfs = 0;
    if(useFreespaceHeuristic_ && freespace_heuristic_costmap)
        hfs = freespace_heuristic_costmap->getCost(dx, dy, startTh, EnvNAVXYTHETALATCfg.EndTheta);

    count++;
    if(count % 1000 == 0) {
        printf("Cur: %d Old: %d, Total Heur: %d, Old used: %d, New used: %d, impr %.2f\n", 
                hfs, heur,
                count, past, count - past, (double)(count-past)/count*100.0);
    }
    if(hfs > heur)
        return hfs;
    past++;
    return heur;
}

moveit_msgs::DisplayTrajectory EnvironmentNavXYThetaLatMoveit::pathToDisplayTrajectory(const std::vector<geometry_msgs::PoseStamped> & path) const
{
    moveit_msgs::DisplayTrajectory dtraj;
    dtraj.model_id = "robot";    // FIXME this is just for matching?
    if(path.empty())
        return dtraj;

    // Do NOT update the scene. This should be the path that we planned with before.
    moveit::core::robotStateToRobotStateMsg(scene->getCurrentState(), dtraj.trajectory_start);
    moveit_msgs::RobotTrajectory traj;
    traj.multi_dof_joint_trajectory.header = path.front().header;   // all poses in path should be in the same frame
    traj.multi_dof_joint_trajectory.joint_names.push_back("world_joint");
    tf::Transform lastPose;
    for(unsigned int i = 0; i < path.size(); ++i) {
        trajectory_msgs::MultiDOFJointTrajectoryPoint pt;
        geometry_msgs::Transform tf;
        tf.translation.x = path[i].pose.position.x;
        tf.translation.y = path[i].pose.position.y;
        tf.translation.z = path[i].pose.position.z;
        tf.rotation = path[i].pose.orientation;

        if(i > 0) {
            tf::Transform curPose;
            tf::transformMsgToTF(tf, curPose);
            tf::Transform delta = lastPose.inverseTimes(curPose);
            if(hypot(delta.getOrigin().x(), delta.getOrigin().y()) < 0.05 &&
                    tf::getYaw(delta.getRotation()) < 0.2)
                continue;
        }

        pt.transforms.push_back(tf);
        tf::transformMsgToTF(tf, lastPose);

        traj.multi_dof_joint_trajectory.points.push_back(pt);
    }

    dtraj.trajectory.push_back(traj);
    return dtraj;
}

