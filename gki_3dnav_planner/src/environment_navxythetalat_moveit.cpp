#include <gki_3dnav_planner/environment_navxythetalat_moveit.h>
#include <geometry_msgs/PoseArray.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/key.h>
#include <sbpl/planners/planner.h>
#include <moveit/move_group/capability_names.h>

// TODO ugly copied + macro
#define XYTHETA2INDEX(X,Y,THETA) (THETA + X*EnvNAVXYTHETALATCfg.NumThetaDirs + \
                                  Y*EnvNAVXYTHETALATCfg.EnvWidth_c*EnvNAVXYTHETALATCfg.NumThetaDirs)

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

EnvironmentNavXYThetaLatMoveit::EnvironmentNavXYThetaLatMoveit(ros::NodeHandle & nhPriv,
        double costmapOffsetX, double costmapOffsetY) :
    costmapOffsetX(costmapOffsetX), costmapOffsetY(costmapOffsetY)
{
    // TODO can we remove these inits, should be done in parent constructor
    GetHashEntry = NULL;
    CreateNewHashEntry = NULL;
    HashTableSize = 0;
    Coord2StateIDHashTable = NULL;
    Coord2StateIDHashTable_lookup = NULL;

    nhPriv.param("scene_update_name", scene_update_name, move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    nhPriv.getParam("allowed_collision_links", allowed_collision_links);

    update_planning_scene();

    planning_scene_publisher = nhPriv.advertise<moveit_msgs::PlanningScene>("planning_scene_3dnav", 1, true);
    pose_array_publisher = nhPriv.advertise<geometry_msgs::PoseArray>("expanded_states", 1, true);
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNavXYThetaLatMoveit::SetGoal(double x_m, double y_m, double theta_rad)
{
    int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);

    SBPL_PRINTF("env: setting goal to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

    if (!IsWithinMapCell(x, y))
    {
        SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    if (!IsValidConfiguration(x, y, theta))
    {
        SBPL_PRINTF("WARNING: goal configuration is invalid\n");
    }

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL)
    {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }
    if (in_full_body_collision(OutHashEntry))
    {
        SBPL_ERROR("ERROR: goal state in collision\n", x, y);
    }

    //need to recompute start heuristics?
    if (EnvNAVXYTHETALAT.goalstateid != OutHashEntry->stateID)
    {
        bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
        bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
    }

    EnvNAVXYTHETALAT.goalstateid = OutHashEntry->stateID;

    EnvNAVXYTHETALATCfg.EndX_c = x;
    EnvNAVXYTHETALATCfg.EndY_c = y;
    EnvNAVXYTHETALATCfg.EndTheta = theta;

    return EnvNAVXYTHETALAT.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNavXYThetaLatMoveit::SetStart(double x_m, double y_m, double theta_rad)
{
    int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);

    if (!IsWithinMapCell(x, y))
    {
        SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    SBPL_PRINTF("env: setting start to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

    if (!IsValidConfiguration(x, y, theta))
    {
        SBPL_PRINTF("WARNING: start configuration %d %d %d is invalid\n", x, y, theta);
    }

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL)
    {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }
    if (in_full_body_collision(OutHashEntry))
    {
        SBPL_ERROR("ERROR: start state in collision\n", x, y);
    }

    //need to recompute start heuristics?
    if (EnvNAVXYTHETALAT.startstateid != OutHashEntry->stateID)
    {
        bNeedtoRecomputeStartHeuristics = true;
        //because termination condition can be not all states TODO - make it dependent on term. condition
        bNeedtoRecomputeGoalHeuristics = true;
    }

    //set start
    EnvNAVXYTHETALAT.startstateid = OutHashEntry->stateID;
    EnvNAVXYTHETALATCfg.StartX_c = x;
    EnvNAVXYTHETALATCfg.StartY_c = y;
    EnvNAVXYTHETALATCfg.StartTheta = theta;

    return EnvNAVXYTHETALAT.startstateid;
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNavXYThetaLatMoveit::CreateNewHashEntry_lookup(int X, int Y, int Theta)
{
    int i;

#if TIME_DEBUG	
    clock_t currenttime = clock();
#endif

    EnvNAVXYTHETALATHashEntry_t* HashEntry = new EnvNAVXYTHETALATHashEntry_t;

    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;

    HashEntry->stateID = StateID2CoordTable.size();

    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);
    full_body_collision_infos.push_back(FullBodyCollisionInfo());

    int index = XYTHETA2INDEX(X, Y, Theta);

#if DEBUG
    if(Coord2StateIDHashTable_lookup[index] != NULL)
    {
        SBPL_ERROR("ERROR: creating hash entry for non-NULL hashentry\n");
        throw new SBPL_Exception();
    }
#endif

    Coord2StateIDHashTable_lookup[index] = HashEntry;

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++)
    {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int) StateID2IndexMapping.size() - 1)
    {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNavXYThetaLatMoveit::CreateNewHashEntry_hash(int X, int Y, int Theta)
{
    int i;

#if TIME_DEBUG	
    clock_t currenttime = clock();
#endif

    EnvNAVXYTHETALATHashEntry_t* HashEntry = new EnvNAVXYTHETALATHashEntry_t;

    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;

    HashEntry->stateID = StateID2CoordTable.size();

    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);
    full_body_collision_infos.push_back(FullBodyCollisionInfo());

    //get the hash table bin
    i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta);

    //insert the entry into the bin
    Coord2StateIDHashTable[i].push_back(HashEntry);

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++)
    {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int) StateID2IndexMapping.size() - 1)
    {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;
}

void EnvironmentNavXYThetaLatMoveit::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/)
{
    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    CostV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    if (actionV != NULL)
    {
        actionV->clear();
        actionV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    }

    //goal state should be absorbing
    if (SourceStateID == EnvNAVXYTHETALAT.goalstateid)
        return;

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    //iterate through actions
    for (aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
    {
        EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int) HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY))
            continue;

        //get cost
        int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST)
            continue;

        EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
        {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }
        // 3d collision check
        if (in_full_body_collision(OutHashEntry))
            continue;
        //		cost += get_full_body_cost_penalty(OutHashEntry);
        //		if (cost >= INFINITECOST)
        //			continue;

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        if (actionV != NULL)
            actionV->push_back(nav3daction);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

void EnvironmentNavXYThetaLatMoveit::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV)
{
    //TODO- to support tolerance, need:
    // a) generate preds for goal state based on all possible goal state variable settings,
    // b) change goal check condition in gethashentry c) change
    //    getpredsofchangedcells and getsuccsofchangedcells functions

    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int) HashEntry->Theta].size());
    CostV->reserve(EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int) HashEntry->Theta].size());

    //iterate through actions
    std::vector<EnvNAVXYTHETALATAction_t*>* actionsV = &EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int) HashEntry->Theta];
    for (aind = 0; aind < (int) EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int) HashEntry->Theta].size(); aind++)
    {

        EnvNAVXYTHETALATAction_t* nav3daction = actionsV->at(aind);

        int predX = HashEntry->X - nav3daction->dX;
        int predY = HashEntry->Y - nav3daction->dY;
        int predTheta = nav3daction->starttheta;

        //skip the invalid cells
        if (!IsValidCell(predX, predY))
            continue;

        //get cost
        int cost = GetActionCost(predX, predY, predTheta, nav3daction);
        if (cost >= INFINITECOST)
            continue;

        EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(predX, predY, predTheta)) == NULL)
        {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(predX, predY, predTheta);
        }
        // 3d collision check
        if (in_full_body_collision(OutHashEntry))
            continue;
        //		cost += get_full_body_cost_penalty(OutHashEntry);
        //		if (cost >= INFINITECOST)
        //			continue;

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

void EnvironmentNavXYThetaLatMoveit::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    int cost;

#if DEBUG
    if(state->StateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in Env... function: stateID illegal\n");
        throw new SBPL_Exception();
    }

    if((int)state->Actions.size() != 0)
    {
        SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
        throw new SBPL_Exception();
    }
#endif

    //goal state should be absorbing
    if (state->StateID == EnvNAVXYTHETALAT.goalstateid)
        return;

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[state->StateID];

    //iterate through actions
    for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
    {
        EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int) HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY))
            continue;

        //get cost
        cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST)
            continue;

#if TIME_DEBUG
        clock_t currenttime = clock();
#endif

        EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
        {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }
        // 3d collision check
        if (in_full_body_collision(OutHashEntry))
            continue;
        //		cost += get_full_body_cost_penalty(OutHashEntry);
        //		if (cost >= INFINITECOST)
        //			continue;

        //add the action
        CMDPACTION* action = state->AddAction(aind);
        action->AddOutcome(OutHashEntry->stateID, cost, 1.0);

#if TIME_DEBUG
        time3_addallout += clock()-currenttime;
#endif
    }
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
        collision_detection::CollisionRequest request;
        //request.contacts = true;
        collision_detection::CollisionResult result;
        sbpl_xy_theta_pt_t pose = discreteToContinuous(state->X, state->Y, state->Theta);
        // TODO
        robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
        robot_state.setVariablePosition("world_joint/x", pose.x + costmapOffsetX);
        robot_state.setVariablePosition("world_joint/y", pose.y + costmapOffsetY);
        robot_state.setVariablePosition("world_joint/theta", pose.theta);
        robot_state.update();
        getPlanningScene()->checkCollision(request, result);
        full_body_collision_infos[state->stateID].initialized = true;
        full_body_collision_infos[state->stateID].collision = result.collision;
        //ROS_INFO("get_full_body_collision_info for (%.2f, %.2f, %.2f) = %d",
        //        pose.x + costmapOffsetX, pose.y + costmapOffsetY,
        //        pose.theta, result.collision);
        //forEach(const collision_detection::CollisionResult::ContactMap::value_type & vt, result.contacts) {
        //    printf("%s - %s, Contacts\n", vt.first.first.c_str(), vt.first.second.c_str());
        //    forEach(const collision_detection::Contact & ct, vt.second) {
        //        printf("   %s - %s (%f %f %f)\n", ct.body_name_1.c_str(), ct.body_name_2.c_str(),
        //                ct.pos.x(), ct.pos.y(), ct.pos.z());
        //    }
        //}

    }
    return full_body_collision_infos[state->stateID];
}

