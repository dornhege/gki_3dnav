#include <gki_3dnav_planner/environment_navxythetalat_generic.h>
#include <geometry_msgs/PoseArray.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/key.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/2Dgridsearch.h>
#include <tf/transform_datatypes.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "timing/timing.h"

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

struct ScopeExit
{
    ScopeExit(boost::function<void ()> fn) : function_(fn) { }
    ~ScopeExit() { function_(); }

    boost::function<void()> function_;
};

EnvironmentNavXYThetaLatGeneric::EnvironmentNavXYThetaLatGeneric(ros::NodeHandle & nhPriv)
{
    std::string freespace_heuristic_costmap_file;
    nhPriv.getParam("freespace_heuristic_costmap", freespace_heuristic_costmap_file);
    if(freespace_heuristic_costmap_file.empty()) {
        freespace_heuristic_costmap = NULL;
    } else {
        freespace_heuristic_costmap = new freespace_mechanism_heuristic::HeuristicCostMap(
                freespace_heuristic_costmap_file,
                freespace_mechanism_heuristic::HeuristicCostMap::OutOfMapExpandEuclideanAppend);
    }

    timeActionCost = new Timing("action_cost", true, Timing::SP_STATS, false);
    timeActionCostParent = new Timing("action_cost_parent", true, Timing::SP_STATS, false);
    timeFreespace = new Timing("freespace_heuristic", true, Timing::SP_STATS, false);
    timeFullBodyCollision = new Timing("full_body_collision", true, Timing::SP_STATS, false);
    time3dCheck = new Timing("3d_check", true, Timing::SP_STATS, false);
    timeHeuristic = new Timing("heuristic", true, Timing::SP_STATS, false);
}

EnvironmentNavXYThetaLatGeneric::~EnvironmentNavXYThetaLatGeneric()
{
    delete timeActionCost;
    delete timeActionCostParent;
    delete timeFreespace;
    delete timeFullBodyCollision;
    delete time3dCheck;
    delete timeHeuristic;
}

int EnvironmentNavXYThetaLatGeneric::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    timeHeuristic->start();
    ScopeExit se(boost::bind(&Timing::end, timeHeuristic));

    int heur = EnvironmentNAVXYTHETALAT::GetFromToHeuristic(FromStateID, ToStateID);

    int fromX, fromY, fromTheta;
    int toX, toY, toTheta;
    GetCoordFromState(FromStateID, fromX, fromY, fromTheta);
    GetCoordFromState(ToStateID, toX, toY, toTheta);

    int dx = toX - fromX;
    int dy = toY - fromY;

    int hfs = getFreespaceCost(dx, dy, fromTheta, toTheta);
    if(hfs > heur)
        return hfs;
    return heur;
}

int EnvironmentNavXYThetaLatGeneric::GetStartHeuristic(int stateID)
{
    timeHeuristic->start();
    ScopeExit se(boost::bind(&Timing::end, timeHeuristic));

    int heur = EnvironmentNAVXYTHETALAT::GetStartHeuristic(stateID);

    int x, y, theta;
    GetCoordFromState(stateID, x, y, theta);
    int dx = EnvNAVXYTHETALATCfg.StartX_c - x;
    int dy = EnvNAVXYTHETALATCfg.StartY_c - y;
    dx = -dx;   // FIXME this is supposed to be FROM start TO stateID, not vice versa
    dy = -dy;
    int endTh = theta;

    int hfs = getFreespaceCost(dx, dy, EnvNAVXYTHETALATCfg.StartTheta, endTh);
    if(hfs > heur)
        return hfs;
    return heur;
}

int EnvironmentNavXYThetaLatGeneric::GetGoalHeuristic(int stateID)
{
    timeHeuristic->start();
    ScopeExit se(boost::bind(&Timing::end, timeHeuristic));

#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    int x, y, theta;
    GetCoordFromState(stateID, x, y, theta);
    //computes distances from start state that is grid2D, so it is EndX_c EndY_c
    int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(x, y);
    int hEuclid = (int)(NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(x, y,
                EnvNAVXYTHETALATCfg.EndX_c,
                EnvNAVXYTHETALATCfg.EndY_c));
    //define this function if it is used in the planner (heuristic backward search would use it)
    int heur = (int)(((double)__max(h2D, hEuclid)) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);

    int dx = EnvNAVXYTHETALATCfg.EndX_c - x;
    int dy = EnvNAVXYTHETALATCfg.EndY_c - y;
    int startTh = theta;

    int hfs = getFreespaceCost(dx, dy, startTh, EnvNAVXYTHETALATCfg.EndTheta);
    if(hfs > heur)
        return hfs;
    return heur;
}

int EnvironmentNavXYThetaLatGeneric::getFreespaceCost(int deltaX, int deltaY, int theta_start, int theta_end, int depth)
{
    if(depth == 0)
        timeFreespace->start();

    if(depth >= 6) {
        ROS_WARN_THROTTLE(1.0, "Freespace Heuristic depth: %d", depth);
    }
    int hfs = 0;
    if(useFreespaceHeuristic_ && freespace_heuristic_costmap){
        hfs = freespace_heuristic_costmap->getCost(deltaX, deltaY, theta_start, theta_end);
        //std::cout << "got first cost " << hfs << std::endl;
        if(hfs == INFINITECOST){
            // TODO move to freespace_heuristic_costmap
            // figure out cell in freespace_heuristic_costmap in the direction of deltaX, deltaY
            int maxdx = freespace_heuristic_costmap->getWidth()/2-2;
            int maxdy = freespace_heuristic_costmap->getHeight()/2-2;
            // y coordinate corresponding to maxdx in direction of deltaX, deltaY, and vice versa
            int yForMaxdx = deltaX==0? (deltaY>0?maxdy:-maxdy) : (maxdx*deltaY)/abs(deltaX);
            int xForMaxdy = deltaY==0? (deltaX>0?maxdx:-maxdx) : (maxdy*deltaX)/abs(deltaY);
            int deltaXInMap = deltaX >= 0? maxdx : -maxdx;
            int deltaYInMap = yForMaxdx;
            //std::cout << "step 1: maxdx " << maxdx << " maxdy " << maxdy << " yForMaxdx " << yForMaxdx << " xForMaxdy " << xForMaxdy << " deltaXInMap " << deltaXInMap << " deltaYInMap " << deltaYInMap << std::endl;
            if(abs(deltaYInMap) > maxdy){
                deltaXInMap = xForMaxdy;
                deltaYInMap = deltaY >= 0? maxdy : -maxdy;
                ROS_ASSERT(abs(xForMaxdy) <= maxdx);
            }
            //std::cout << "step 2: maxdx " << maxdx << " maxdy " << maxdy << " yForMaxdx " << yForMaxdx << " xForMaxdy " << xForMaxdy << " deltaXInMap " << deltaXInMap << " deltaYInMap " << deltaYInMap << std::endl;
            int hdelta = freespace_heuristic_costmap->getCost(deltaXInMap, deltaYInMap, theta_start, 0)
                + getFreespaceCost(deltaX-deltaXInMap, deltaY-deltaYInMap, 0, theta_end, depth + 1);
            for(size_t i = 1; i < EnvNAVXYTHETALATCfg.NumThetaDirs; ++i){
                int hnewdelta = freespace_heuristic_costmap->getCost(deltaXInMap, deltaYInMap, theta_start, i)
                    + getFreespaceCost(deltaX-deltaXInMap, deltaY-deltaYInMap, i, theta_end, depth + 1);
                if(hnewdelta < hdelta){
                    hdelta = hnewdelta;
                }
            }
            hfs = hdelta;
        }
    }

    if(depth == 0)
        timeFreespace->end();
    return hfs;
}


void EnvironmentNavXYThetaLatGeneric::resetTimingStats()
{
    timeActionCost->getStats().reset();
    timeActionCostParent->getStats().reset();
    timeFreespace->getStats().reset();
    timeFullBodyCollision->getStats().reset();
    time3dCheck->getStats().reset();
    timeHeuristic->getStats().reset();
}

void EnvironmentNavXYThetaLatGeneric::printTimingStats()
{
    timeActionCost->printStats(true);
    timeActionCostParent->printStats(true);
    timeFreespace->printStats(true);
    timeFullBodyCollision->printStats(true);
    time3dCheck->printStats(true);
    timeHeuristic->printStats(true);
}

moveit_msgs::DisplayTrajectory EnvironmentNavXYThetaLatGeneric::pathToDisplayTrajectory(const std::vector<geometry_msgs::PoseStamped> & path) const
{
    moveit_msgs::DisplayTrajectory dtraj;
    dtraj.model_id = "pr2";
    if(path.empty())
        return dtraj;

    // Do NOT update the scene. This should be the path that we planned with before.
    // TODO moveit::core::robotStateToRobotStateMsg(scene->getCurrentState(), dtraj.trajectory_start);
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

