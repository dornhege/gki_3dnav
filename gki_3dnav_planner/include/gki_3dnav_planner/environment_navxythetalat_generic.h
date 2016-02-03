#ifndef ENVIRONMENT_NAVXYTHETALAT_GENERIC_H
#define ENVIRONMENT_NAVXYTHETALAT_GENERIC_H

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "freespace_mechanism_heuristic/freespace_mechanism_heuristic.h"
#include "timing/timing.h"

/// Generic x, y, theta environment that does not assume a specific grid.
/**
 * Child classes should make this behave like a discretized x, y, theta lattice.
 *
 * All poses are assumed to be in one frame given by getPlanningFrame().
 */
class EnvironmentNavXYThetaLatGeneric : public EnvironmentNAVXYTHETALAT
{
    public:
        EnvironmentNavXYThetaLatGeneric(ros::NodeHandle & nhPriv);
        virtual ~EnvironmentNavXYThetaLatGeneric();

        /// Returns the tf frame that this env assumes all poses to be in.
        virtual std::string getPlanningFrame() const = 0;

        /// Transform a pose in any frame by a suitable method to the planning frame.
        // TODO default = tf?
        virtual bool transformPoseToPlanningFrame(geometry_msgs::PoseStamped & pose) = 0;

        /// Return a pose in the planning frame for stateID.
        virtual geometry_msgs::Pose poseFromStateID(int stateID) const = 0;

        /// Get the x/y dimensions of the underlying environment.
        virtual void getExtents(double minX, double maxX, double minY, double maxY) const = 0;

        /// Update the internal representation to be current for a plan request.
        virtual void updateForPlanRequest() { }

        // TODO heuristic stuff in here?
        bool useFreespaceHeuristic(bool on) { useFreespaceHeuristic_ = on; }

        virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
        virtual int GetStartHeuristic(int stateID);
        virtual int GetGoalHeuristic(int stateID);
        virtual int getFreespaceCost(int deltaX, int deltaY, int theta_start, int theta_end, int depth = 0);

        /// Convert a path into a DisplayTrajectory.
        // TODO needs moveit? can be moved to planner as it has poses already?
        // -> Needs moveit, if not avail. return empty --> planner should handle gracefully and warn once!
        virtual moveit_msgs::DisplayTrajectory pathToDisplayTrajectory(
                const std::vector<geometry_msgs::PoseStamped> & path) const;

        virtual void resetTimingStats();
        virtual void printTimingStats();

    protected:
        freespace_mechanism_heuristic::HeuristicCostMap* freespace_heuristic_costmap;
        bool useFreespaceHeuristic_;

        // TODO which of these - would need to be wrapped at outside of functions
        // not in the parent here!
        Timing* timeActionCost;
        Timing* timeActionCostParent;
        Timing* timeFreespace;
        Timing* timeFullBodyCollision;
        Timing* time3dCheck;
        Timing* timeHeuristic;
};

#endif

