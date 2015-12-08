#include <gki_3dnav_planner/environment_navxythetalat_flourish.h>
#include <geometry_msgs/PoseArray.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/key.h>
#include <sbpl/planners/planner.h>
#include <moveit/move_group/capability_names.h>

#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <basics/transformationRepresentation.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

Eigen::Isometry3f stampedTfToIsometry(const tf::StampedTransform& trafo){
  tf::Vector3 trans = trafo.getOrigin();
  tf::Quaternion quat = trafo.getRotation();
  Eigen::Vector3f translation(trans.x(), trans.y(), trans.z());
  Eigen::Quaternionf quaternion(quat.w(), quat.x(), quat.y(), quat.z());
  quaternion.normalize();
  Eigen::Isometry3f result(Eigen::Isometry3f::Identity());
  result.linear() = quaternion.toRotationMatrix();
  result.translation() = translation;
  return result;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNavXYThetaLatFlourish::SetG(double x_m, double y_m, double theta_rad){
  int x = CONTXY2DISC(x_m-costmapOffsetX, tMap.getResolution());
  int y = CONTXY2DISC(y_m-costmapOffsetY, tMap.getResolution());
  int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);
  ROS_INFO("env: setting goal to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);
  ROS_INFO("map size: (%d %d)\n", tMap.size()(0), tMap.size()(1));

  Eigen::Vector2i index(x,y);
  if(!tMap.isInside(index)){
    ROS_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x,y);
    return -1;
  }

  if(!IsValidConfiguration(x,y,theta)){
    ROS_INFO("WARNING: goal configuration is invalid\n");
  }

  EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
  if((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL){
    //have to create a new entry
    OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
  }

  //need to recompute start heuristics?
  if(EnvNAVXYTHETALAT.goalstateid != OutHashEntry->stateID){
    bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
    bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
  }

  EnvNAVXYTHETALAT.goalstateid = OutHashEntry->stateID;

  EnvNAVXYTHETALATCfg.EndX_c = x;
  EnvNAVXYTHETALATCfg.EndY_c = y;
  EnvNAVXYTHETALATCfg.EndTheta = theta;

  return EnvNAVXYTHETALAT.goalstateid;    
}

bool EnvironmentNavXYThetaLatFlourish::IsValidCell(int X, int Y){
  Eigen::Vector2i index(X,Y);
  return (tMap.isInside(index) &&
	  tMap.cell(X,Y).getElevation() < robotSafeHeight);
}

bool EnvironmentNavXYThetaLatFlourish::IsWithinMapCell(int X, int Y){
  Eigen::Vector2i index(X,Y);
  return (tMap.isInside(index));
}

bool EnvironmentNavXYThetaLatFlourish::IsValidConfiguration(int X, int Y, int Theta){
  std::vector<sbpl_2Dcell_t> footprint;
  sbpl_xy_theta_pt_t pose;

  //compute continuous pose
  pose.x = DISCXY2CONT(X, tMap.getResolution());
  pose.y = DISCXY2CONT(Y, tMap.getResolution());
  pose.theta = DiscTheta2Cont(Theta, EnvNAVXYTHETALATCfg.NumThetaDirs);

  //compute footprint cells
  get_2d_footprint_cells(EnvNAVXYTHETALATCfg.FootprintPolygon, &footprint, pose, tMap.getResolution());

  //iterate over all footprint cells
  for(int find = 0; find < (int)footprint.size(); find++){
    int x = footprint.at(find).x;
    int y = footprint.at(find).y;

    if(!IsValidCell(x,y)){
      return false;
    }
  }

  return true;
}

void EnvironmentNavXYThetaLatFlourish::EnsureHeuristicsUpdated(bool bGoalHeuristics){
  std::cerr << "attempting to ensure heuristics are updated" << std::endl;

  /*if(bNeedtoRecomputeStartHeuristics && !bGoalHeuristics){
    grid2Dsearchfromstart->search(EnvNAVXYTHETALATCfg.Grid2D, EnvNAVXYTHETALATCfg.cost_inscribed_thresh, 
				  EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, 
				  SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH); 
    bNeedtoRecomputeStartHeuristics = false;
    SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c)
						    /EnvNAVXYTHETALATCfg.nominalvel_mpersecs));

  }


  if(bNeedtoRecomputeGoalHeuristics && bGoalHeuristics){
    grid2Dsearchfromgoal->search(EnvNAVXYTHETALATCfg.Grid2D, EnvNAVXYTHETALATCfg.cost_inscribed_thresh, 
				 EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c,  
				 SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH); 
    bNeedtoRecomputeGoalHeuristics = false;
    SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c)
						    /EnvNAVXYTHETALATCfg.nominalvel_mpersecs));

						    }*/
}

bool EnvironmentNavXYThetaLatFlourish::UpdateCost(int x, int y, unsigned char newcost)
{
#if DEBUG
  //SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,EnvNAVXYTHETALATCfg.Grid2D[x][y], newcost);
#endif

  std::cerr << "attempting to update cost" << std::endl;

  //EnvNAVXYTHETALATCfg.Grid2D[x][y] = newcost;
  //
  //bNeedtoRecomputeStartHeuristics = true;
  //bNeedtoRecomputeGoalHeuristics = true;

  return true;
}

bool EnvironmentNavXYThetaLatFlourish::IsObstacle(int x, int y){
#if DEBUG
  //SBPL_FPRINTF(fDeb, "Status of cell %d %d is queried. Its cost=%d\n", x,y,EnvNAVXYTHETALATCfg.Grid2D[x][y]);
#endif
  return tMap.cell(x,y).getTraversable(); 
}

unsigned char EnvironmentNavXYThetaLatFlourish::GetMapCost(int x, int y){
  std::cerr << "attempting to get map cost" << std::endl;
  return 0;
}

void EnvironmentNavXYThetaLatFlourish::ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath, std::vector<sbpl_xy_theta_pt_t>* xythetaPath){
  std::vector<EnvNAVXYTHETALATAction_t*> actionV;
  std::vector<int> CostV;
  std::vector<int> SuccIDV;
  int targetx_c, targety_c, targettheta_c;
  int sourcex_c, sourcey_c, sourcetheta_c;

  SBPL_PRINTF("checks=%ld\n", checks);

  xythetaPath->clear();

#if DEBUG
  SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
#endif

  for(int pind = 0; pind < (int)(stateIDPath->size())-1; pind++){
    int sourceID = stateIDPath->at(pind);
    int targetID = stateIDPath->at(pind+1);

#if DEBUG
    GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
#endif


    //get successors and pick the target via the cheapest action
    SuccIDV.clear();
    CostV.clear();
    actionV.clear();
    GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);
		
    int bestcost = INFINITECOST;
    int bestsind = -1;

#if DEBUG
    GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
    GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
    SBPL_FPRINTF(fDeb, "looking for %d %d %d -> %d %d %d (numofsuccs=%d)\n", sourcex_c, sourcey_c, sourcetheta_c,
		 targetx_c, targety_c, targettheta_c, (int)SuccIDV.size());

#endif

    for(int sind = 0; sind < (int)SuccIDV.size(); sind++){

#if DEBUG
      int x_c, y_c, theta_c;
      GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c);
      SBPL_FPRINTF(fDeb, "succ: %d %d %d\n", x_c, y_c, theta_c); 
#endif

      if(SuccIDV[sind] == targetID && CostV[sind] <= bestcost){
	bestcost = CostV[sind];
	bestsind = sind;
      }
    }
    if(bestsind == -1){
      SBPL_ERROR("ERROR: successor not found for transition:\n");
      GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
      GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
      SBPL_PRINTF("%d %d %d -> %d %d %d\n", sourcex_c, sourcey_c, sourcetheta_c,
		  targetx_c, targety_c, targettheta_c); 
      throw new SBPL_Exception();
    }

    //now push in the actual path
    int sourcex_c, sourcey_c, sourcetheta_c;
    GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
    double sourcex, sourcey;
    sourcex = DISCXY2CONT(sourcex_c, tMap.getResolution());
    sourcey = DISCXY2CONT(sourcey_c, tMap.getResolution());
      //TODO - when there are no motion primitives we should still print source state
    for(int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size())-1; ipind++){
      //translate appropriately
      sbpl_xy_theta_pt_t intermpt = actionV[bestsind]->intermptV[ipind];
      intermpt.x += sourcex;
      intermpt.y += sourcey;

#if DEBUG
      int nx = CONTXY2DISC(intermpt.x, tMap.getResolution());
      int ny = CONTXY2DISC(intermpt.y, tMap.getResolution());
      SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d) ", 
		   intermpt.x, intermpt.y, intermpt.theta, 
		   nx, ny, 
		   ContTheta2Disc(intermpt.theta, EnvNAVXYTHETALATCfg.NumThetaDirs), GetCellCost(nx, ny, intermpt.theta));//TODO
	  if(ipind == 0) SBPL_FPRINTF(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
	  else SBPL_FPRINTF(fDeb, "\n");
#endif

	  //store
	  xythetaPath->push_back(intermpt);
	}
    }
}

EnvironmentNavXYThetaLatFlourish::EnvironmentNavXYThetaLatFlourish(ros::NodeHandle & nhPriv, Ais3dTools::TraversableMap tMap):
  tMap(tMap)
{

  robotHeight = 1.2;
  robotBodyHeight = 0.2; 
  robotBodyWidth = 2.0; 
  robotBodyLength = 2.0;
  robotArmLength = 0.6;
  robotMinSafeDistance = 0.1;
  robotSafeHeight = robotHeight - robotMinSafeDistance;

  Eigen::Vector2f offset;
  tMap.getOffset(offset);
  costmapOffsetX = offset.x();
  costmapOffsetY = offset.y();
  

  tf::TransformListener tfListener;
  tf::StampedTransform rightFrontWheelToBaseLinkTransform, leftFrontWheelToBaseLinkTransform, rightRearWheelToBaseLinkTransform, leftRearWheelToBaseLinkTransform;

  //ros::Time now = ros::Time::now();

  try{
    tfListener.waitForTransform("/wheel_fr_link",  "/base_link", 
    				 ros::Time(0), ros::Duration(0.5));
    tfListener.lookupTransform("/wheel_fr_link",  "/base_link", 
    				ros::Time(0), rightFrontWheelToBaseLinkTransform);
    tfListener.waitForTransform("/wheel_fl_link",  "/base_link", 
    				 ros::Time(0), ros::Duration(0.5));
    tfListener.lookupTransform("/wheel_fl_link",  "/base_link", 
    				ros::Time(0), leftFrontWheelToBaseLinkTransform);
    tfListener.waitForTransform("/wheel_rr_link",  "/base_link", 
    				 ros::Time(0), ros::Duration(0.5));
    tfListener.lookupTransform("/wheel_rr_link",  "/base_link", 
    				ros::Time(0), rightRearWheelToBaseLinkTransform);
    tfListener.waitForTransform("/wheel_rl_link",  "/base_link", 
    				 ros::Time(0), ros::Duration(0.5));
    tfListener.lookupTransform("/wheel_rl_link",  "/base_link", 
    				ros::Time(0), leftRearWheelToBaseLinkTransform);
  }catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  rightFrontWheelToBaseLink = stampedTfToIsometry(rightFrontWheelToBaseLinkTransform);
  leftFrontWheelToBaseLink = stampedTfToIsometry(leftFrontWheelToBaseLinkTransform);
  rightRearWheelToBaseLink = stampedTfToIsometry(rightRearWheelToBaseLinkTransform);
  leftRearWheelToBaseLink = stampedTfToIsometry(leftRearWheelToBaseLinkTransform);

  nhPriv.param("scene_update_name", scene_update_name, move_group::GET_PLANNING_SCENE_SERVICE_NAME);
  //scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

  nhPriv.getParam("allowed_collision_links", allowed_collision_links);

  //update_planning_scene();

  planning_scene_publisher = nhPriv.advertise<moveit_msgs::PlanningScene>("planning_scene_3dnav", 1, true);
  traversable_map_publisher = nhPriv.advertise<visualization_msgs::Marker>("travmap", 1, true); 
  pose_array_publisher = nhPriv.advertise<geometry_msgs::PoseArray>("expanded_states", 1, true);
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNavXYThetaLatFlourish::SetGoal(double x_m, double y_m, double theta_rad)
{
  // TODO if(EnvironmentNAVXYTHETALAT::SetGoal(x_m, y_m, theta_rad) == -1)
  if(SetG(x_m, y_m, theta_rad) == -1)
    return -1;

  int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
  int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
  int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);
  EnvNAVXYTHETALATHashEntry_t* OutHashEntry = (this->*GetHashEntry)(x, y, theta);
  ROS_ASSERT(OutHashEntry != NULL);   // should have been created by parent
  /*TODO if(in_full_body_collision(OutHashEntry)) {
    SBPL_ERROR("ERROR: goal state in collision\n", x, y);
    EnvNAVXYTHETALAT.goalstateid = -1;
    return -1;
    }*/

  return EnvNAVXYTHETALAT.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNavXYThetaLatFlourish::SetStart(double x_m, double y_m, double theta_rad)
{
  if(EnvironmentNAVXYTHETALAT::SetStart(x_m, y_m, theta_rad) == -1)
    return -1;

  int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
  int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
  int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);
  EnvNAVXYTHETALATHashEntry_t* OutHashEntry = (this->*GetHashEntry)(x, y, theta);
  ROS_ASSERT(OutHashEntry != NULL);

  //std::cout << "number of actions: " << EnvNAVXYTHETALATCfg.actionwidth << std::endl;
  /*TODO if (in_full_body_collision(OutHashEntry)) {
    SBPL_ERROR("ERROR: start state in collision\n", x, y);
    EnvNAVXYTHETALAT.startstateid = -1;
    return -1;
    }*/

  return EnvNAVXYTHETALAT.startstateid;
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNavXYThetaLatFlourish::CreateNewHashEntry_lookup(int X, int Y, int Theta){
  // the CreateNewHashEntry... functions always create a new entry, so we can assume that happens and
  // add the full_body_collision_infos entry now.
  // Do this before, instead of after the call as exceptions will appear only after a StateID2CoordTable
  // entry was created, so that at least StateID2CoordTable and full_body_collision_infos are 
  // consistent.
  //full_body_collision_infos.push_back(FullBodyCollisionInfo());
  EnvNAVXYTHETALATHashEntry_t* he = EnvironmentNAVXYTHETALAT::CreateNewHashEntry_lookup(X, Y, Theta);

  return he;
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNavXYThetaLatFlourish::CreateNewHashEntry_hash(int X, int Y, int Theta){
  //full_body_collision_infos.push_back(FullBodyCollisionInfo());
  EnvNAVXYTHETALATHashEntry_t* he = EnvironmentNAVXYTHETALAT::CreateNewHashEntry_hash(X, Y, Theta);

  return he;
}

int EnvironmentNavXYThetaLatFlourish::GetCellCost(int X, int Y, int Theta){
  int theta = NORMALIZEDISCTHETA(Theta, NAVXYTHETALAT_THETADIRS);
  sbpl_xy_theta_pt_t coords = discreteToContinuous(X, Y, Theta);
  Eigen::Isometry3f baseTrafo = Eigen::Isometry3f::Identity();
  Ais3dTools::TransformationRepresentation::getMatrixFromTranslationAndEuler<Eigen::Isometry3f, float>(coords.x, coords.y, 0, 0, 0, coords.theta, baseTrafo);

  // TODO - check wheel cells
  Eigen::Isometry3f rfWheelToGlobal = baseTrafo*rightFrontWheelToBaseLink;
  Eigen::Isometry3f lfWheelToGlobal = baseTrafo*leftFrontWheelToBaseLink;
  Eigen::Isometry3f rrWheelToGlobal = baseTrafo*rightRearWheelToBaseLink;
  Eigen::Isometry3f lrWheelToGlobal = baseTrafo*leftRearWheelToBaseLink;
  Eigen::Vector2f rfWheelCoordinates(rfWheelToGlobal.translation().x(), rfWheelToGlobal.translation().y());
  Eigen::Vector2f lfWheelCoordinates(lfWheelToGlobal.translation().x(), lfWheelToGlobal.translation().y());
  Eigen::Vector2f rrWheelCoordinates(rrWheelToGlobal.translation().x(), rrWheelToGlobal.translation().y());
  Eigen::Vector2f lrWheelCoordinates(lrWheelToGlobal.translation().x(), lrWheelToGlobal.translation().y());
  Eigen::Vector2i rfWheelIndex(tMap.worldToGrid(rfWheelCoordinates));
  Eigen::Vector2i lfWheelIndex(tMap.worldToGrid(lfWheelCoordinates));
  Eigen::Vector2i rrWheelIndex(tMap.worldToGrid(rrWheelCoordinates));
  Eigen::Vector2i lrWheelIndex(tMap.worldToGrid(lrWheelCoordinates));

  // check if position of the robot centre and the wheel cells are inside the map and the centre cell is not too high
  Eigen::Vector2i index(X, Y);
  if(!tMap.isInside(index) || tMap.cell(X, Y).getElevation() > robotSafeHeight
     || !tMap.isInside(rfWheelIndex) || !tMap.isInside(lfWheelIndex) 
     || !tMap.isInside(rrWheelIndex) || !tMap.isInside(lrWheelIndex))
    return INFINITECOST;
  
  float cellcost = 1.f/tMap.cell(rfWheelIndex.x(), rfWheelIndex.y()).getDistToObstacle()
      + 1.f/tMap.cell(lfWheelIndex.x(), lfWheelIndex.y()).getDistToObstacle()
      + 1.f/tMap.cell(rrWheelIndex.x(), rrWheelIndex.y()).getDistToObstacle()
      + 1.f/tMap.cell(lrWheelIndex.x(), lrWheelIndex.y()).getDistToObstacle();
  return cellcost;
}

int EnvironmentNavXYThetaLatFlourish::GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action){
  int cost;
  sbpl_2Dcell_t cell;
  EnvNAVXYTHETALAT3Dcell_t interm3Dcell;
  int i;
  

  float startcellcost = GetCellCost(SourceX, SourceY, SourceTheta);
  float endcellcost = GetCellCost(SourceX+action->dX, SourceY+action->dY, action->endtheta);
  
  if(startcellcost == INFINITECOST || endcellcost == INFINITECOST){
    return INFINITECOST;
  }

  /*sbpl_xy_theta_pt_t coords = discreteToContinuous(SourceX, SourceY, SourceTheta);
  //Eigen::Vector2i gridCoordinatesStart(SourceX, SourceY);
  //Eigen::Vector2f worldCoordinatesStart = tMap.gridToWorld(gridCoordinatesStart);
  Eigen::Isometry3f baseTrafoStart = Eigen::Isometry3f::Identity();
  Ais3dTools::TransformationRepresentation::getMatrixFromTranslationAndEuler<Eigen::Isometry3f, float>(coords.x, coords.y, 0, 0, 0, coords.theta, baseTrafoStart);

  Eigen::Isometry3f rfWheelToGlobalStart = baseTrafoStart*rightFrontWheelToBaseLink;
  Eigen::Isometry3f lfWheelToGlobalStart = baseTrafoStart*leftFrontWheelToBaseLink;
  Eigen::Isometry3f rrWheelToGlobalStart = baseTrafoStart*rightRearWheelToBaseLink;
  Eigen::Isometry3f lrWheelToGlobalStart = baseTrafoStart*leftRearWheelToBaseLink;
  Eigen::Vector2f rfWheelStartCoordinates(rfWheelToGlobalStart.translation().x(), rfWheelToGlobalStart.translation().y());
  Eigen::Vector2f lfWheelStartCoordinates(lfWheelToGlobalStart.translation().x(), lfWheelToGlobalStart.translation().y());
  Eigen::Vector2f rrWheelStartCoordinates(rrWheelToGlobalStart.translation().x(), rrWheelToGlobalStart.translation().y());
  Eigen::Vector2f lrWheelStartCoordinates(lrWheelToGlobalStart.translation().x(), lrWheelToGlobalStart.translation().y());
  Eigen::Vector2i rfWheelStartIndex(tMap.worldToGrid(rfWheelStartCoordinates));
  Eigen::Vector2i lfWheelStartIndex(tMap.worldToGrid(lfWheelStartCoordinates));
  Eigen::Vector2i rrWheelStartIndex(tMap.worldToGrid(rrWheelStartCoordinates));
  Eigen::Vector2i lrWheelStartIndex(tMap.worldToGrid(lrWheelStartCoordinates));

  //Eigen::Vector2i gridCoordinatesEnd(SourceX+action->dX, SourceY+action->dY);
  //Eigen::Vector2f worldCoordinatesEnd = tMap.gridToWorld(gridCoordinatesEnd);
  Eigen::Isometry3f baseTrafoEnd = Eigen::Isometry3f::Identity();
  int endTheta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);
  coords = discreteToContinuous(SourceX+action->dX, SourceY+action->dY, endTheta);
  Ais3dTools::TransformationRepresentation::getMatrixFromTranslationAndEuler<Eigen::Isometry3f, float>(coords.x, coords.y, 0, 0, 0, coords.theta, baseTrafoEnd);

  Eigen::Isometry3f rfWheelToGlobalEnd = baseTrafoEnd*rightFrontWheelToBaseLink;
  Eigen::Isometry3f lfWheelToGlobalEnd = baseTrafoEnd*leftFrontWheelToBaseLink;
  Eigen::Isometry3f rrWheelToGlobalEnd = baseTrafoEnd*rightRearWheelToBaseLink;
  Eigen::Isometry3f lrWheelToGlobalEnd = baseTrafoEnd*leftRearWheelToBaseLink;
  Eigen::Vector2f rfWheelEndCoordinates(rfWheelToGlobalEnd.translation().x(), rfWheelToGlobalEnd.translation().y());
  Eigen::Vector2f lfWheelEndCoordinates(lfWheelToGlobalEnd.translation().x(), lfWheelToGlobalEnd.translation().y());
  Eigen::Vector2f rrWheelEndCoordinates(rrWheelToGlobalEnd.translation().x(), rrWheelToGlobalEnd.translation().y());
  Eigen::Vector2f lrWheelEndCoordinates(lrWheelToGlobalEnd.translation().x(), lrWheelToGlobalEnd.translation().y());
  Eigen::Vector2i rfWheelEndIndex(tMap.worldToGrid(rfWheelEndCoordinates));
  Eigen::Vector2i lfWheelEndIndex(tMap.worldToGrid(lfWheelEndCoordinates));
  Eigen::Vector2i rrWheelEndIndex(tMap.worldToGrid(rrWheelEndCoordinates));
  Eigen::Vector2i lrWheelEndIndex(tMap.worldToGrid(lrWheelEndCoordinates));

  // check if start and end position of the robot centre and the wheel cells are inside the map and the centre cell is not too high
  Eigen::Vector2i startIndex(SourceX, SourceY);
  Eigen::Vector2i endIndex(SourceX + action->dX, SourceY + action->dY);
  if(!tMap.isInside(startIndex) || tMap.cell(SourceX, SourceY).getElevation() > robotSafeHeight
     || !tMap.isInside(rfWheelStartIndex) || !tMap.isInside(lfWheelStartIndex) 
     || !tMap.isInside(rrWheelStartIndex) || !tMap.isInside(lrWheelStartIndex))
    return INFINITECOST;
  if(!tMap.isInside(endIndex) || tMap.cell(SourceX + action->dX, SourceY + action->dY).getElevation() > robotSafeHeight
     || !tMap.isInside(rfWheelEndIndex) || !tMap.isInside(lfWheelEndIndex) 
     || !tMap.isInside(rrWheelEndIndex) || !tMap.isInside(lrWheelEndIndex))
     return INFINITECOST;*/
  
  // iterate over discretized center cells and compute cost based on them
  //unsigned char maxcellcost = 0;
  float maxcellcost = 0;
  for(i = 0; i < (int)action->interm3DcellsV.size(); i++){
    interm3Dcell = action->interm3DcellsV.at(i);
    interm3Dcell.x = interm3Dcell.x + SourceX;
    interm3Dcell.y = interm3Dcell.y + SourceY;
    float intermCost = GetCellCost(interm3Dcell.x, interm3Dcell.y, interm3Dcell.theta);
    // check if cell between start and end position os inside the map
    if(intermCost == INFINITECOST){
      return INFINITECOST;
    }
    
    maxcellcost = std::max(maxcellcost, intermCost);
  }
  
  //check collisions that for the particular footprint orientation along the action
  // if costs indicate possible obstacle between inner and outer circle of the robot
  //TODO - use correct footprint, define a reasonable cost_possibly_circumscribed_thresh
  if(EnvNAVXYTHETALATCfg.FootprintPolygon.size() > 1 && (int)maxcellcost >= EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh){
    //checks++;
 
    for(i = 0; i < (int)action->intersectingcellsV.size(); i++){
      //get the cell in the map
      cell = action->intersectingcellsV.at(i);
      cell.x = cell.x + SourceX;
      cell.y = cell.y + SourceY;
      
      //check validity
      Eigen::Vector2i interIndex(cell.x, cell.y);
      if(!tMap.isInside(interIndex) || tMap.cell(cell.x, cell.y).getElevation() > robotSafeHeight){
	return INFINITECOST;
      }
    }
  }
 
  //to ensure consistency of h2D:
  maxcellcost = std::max(maxcellcost, startcellcost);
  int currentmaxcost = (int)std::max(maxcellcost, endcellcost);
 
  cost = action->cost*(currentmaxcost+1); //use cell cost as multiplicative factor
  return cost;
  // full body collision check
  /*if(cost >= INFINITECOST)
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
  */

}


// Initialize Configuration
void EnvironmentNavXYThetaLatFlourish::SetConfiguration(int width, int height,
						    const unsigned char* mapdata,
						    int startx, int starty, int starttheta,
						    int goalx, int goaly, int goaltheta,
						    double cellsize_m, double nominalvel_mpersecs,
						    double timetoturn45degsinplace_secs,
						    const std::vector<sbpl_2Dpt_t> & robot_perimeterV)
{
  std::cout << "using new setConfiguration" << std::endl;
  EnvNAVXYTHETALATCfg.EnvWidth_c = width;
  EnvNAVXYTHETALATCfg.EnvHeight_c = height;
  EnvNAVXYTHETALATCfg.StartX_c = startx;
  EnvNAVXYTHETALATCfg.StartY_c = starty;
  EnvNAVXYTHETALATCfg.StartTheta = starttheta;
 
  if(EnvNAVXYTHETALATCfg.StartX_c < 0 || EnvNAVXYTHETALATCfg.StartX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETALATCfg.StartY_c < 0 || EnvNAVXYTHETALATCfg.StartY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETALATCfg.StartTheta < 0 || EnvNAVXYTHETALATCfg.StartTheta >= EnvNAVXYTHETALATCfg.NumThetaDirs) {
    SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
    throw new SBPL_Exception();
  }
  
  EnvNAVXYTHETALATCfg.EndX_c = goalx;
  EnvNAVXYTHETALATCfg.EndY_c = goaly;
  EnvNAVXYTHETALATCfg.EndTheta = goaltheta;

  if(EnvNAVXYTHETALATCfg.EndX_c < 0 || EnvNAVXYTHETALATCfg.EndX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal goal coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETALATCfg.EndY_c < 0 || EnvNAVXYTHETALATCfg.EndY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal goal coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETALATCfg.EndTheta < 0 || EnvNAVXYTHETALATCfg.EndTheta >= EnvNAVXYTHETALATCfg.NumThetaDirs) {
    SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
    throw new SBPL_Exception();
  }

  // TODO
  EnvNAVXYTHETALATCfg.FootprintPolygon = robot_perimeterV;

  EnvNAVXYTHETALATCfg.nominalvel_mpersecs = nominalvel_mpersecs;
  EnvNAVXYTHETALATCfg.cellsize_m = cellsize_m;
  EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;


  //allocate the 2D environment. Not used anymore -> Grid2D = NULL
  EnvNAVXYTHETALATCfg.Grid2D = NULL;
  /*EnvNAVXYTHETALATCfg.Grid2D = new unsigned char* [EnvNAVXYTHETALATCfg.EnvWidth_c];
  for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
    EnvNAVXYTHETALATCfg.Grid2D[x] = new unsigned char [EnvNAVXYTHETALATCfg.EnvHeight_c];
  }
  
  //environment:
  if (0 == mapdata) {
    for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
	EnvNAVXYTHETALATCfg.Grid2D[x][y] = 0;
      }
    }
  }
  else {
    for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
	EnvNAVXYTHETALATCfg.Grid2D[x][y] = mapdata[x+y*width];
      }
    }
    }*/
}


// PlanningScene handling

void EnvironmentNavXYThetaLatFlourish::update_planning_scene()
{
  return;
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

void EnvironmentNavXYThetaLatFlourish::update_planning_scene(planning_scene::PlanningSceneConstPtr scn)
{
  this->scene = planning_scene::PlanningScene::clone(scn);

  collision_detection::AllowedCollisionMatrix & allowed_collisions = scene->getAllowedCollisionMatrixNonConst();
  forEach(const std::string & cl, allowed_collision_links)
    allowed_collisions.setDefaultEntry(cl, true);
}

planning_scene::PlanningSceneConstPtr EnvironmentNavXYThetaLatFlourish::getPlanningScene()
{
  return scene;
}

void EnvironmentNavXYThetaLatFlourish::publish_planning_scene()
{
  moveit_msgs::PlanningScene msg;
  getPlanningScene()->getPlanningSceneMsg(msg);
  planning_scene_publisher.publish(msg);
}


void EnvironmentNavXYThetaLatFlourish::clear_full_body_collision_infos()
{
  for (size_t i = 0; i < full_body_collision_infos.size(); i++)
    {
      full_body_collision_infos[i].initialized = false;
    }
}


void EnvironmentNavXYThetaLatFlourish::publish_expanded_states()
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

void EnvironmentNavXYThetaLatFlourish::publish_traversable_map(){
  if(tMap.size()(0) == 0 || tMap.size()(1) == 0){
    std::cerr << "Error: Traversable Map has size 0!" << std::endl;
    return;
  }

  //TODO
  planningFrameID = "odom";

  visualization_msgs::Marker marker;
  marker.header.frame_id = planningFrameID;
  marker.header.stamp = ros::Time::now();
  marker.ns = "traversability_map";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
    
  float res = tMap.getResolution();
  marker.scale.x = res;
  marker.scale.y = res;
  marker.scale.z = res;
  marker.color.a = 1.0; 
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  float minElevation = tMap.cell(0,0).getElevation();
  float maxElevation = minElevation;
  for(size_t i = 0; i < tMap.size()(0); i++){
    for(size_t j = 0; j < tMap.size()(1); j++){
      if(tMap.cell(i,j).getElevation() != -1000){
	minElevation = std::min(minElevation, tMap.cell(i,j).getElevation());
	maxElevation = std::max(maxElevation, tMap.cell(i,j).getElevation());
      }
    }
  }
  for(size_t i = 0; i < tMap.size()(0); i++){
    for(size_t j = 0; j < tMap.size()(1); j++){
      if(tMap.cell(i,j).getElevation() != -1000){
	Eigen::Vector2i index(i,j);
	geometry_msgs::Point p;
	p.x = tMap.gridToWorld(index)(0);
	p.y = tMap.gridToWorld(index)(1);
	p.z = tMap.cell(i,j).getElevation();
	marker.points.push_back(p);
	std_msgs::ColorRGBA c;
	if(tMap.cell(i,j).getTraversable()){
	  c.r = 0;
	  c.g = 0;
	  //c.b = 0.2 + (tMap.cell(i,j).getElevation()-minElevation)*0.8f/(maxElevation-minElevation);
	  c.b = 1-(tMap.cell(i,j).getElevation()-minElevation)/(maxElevation-minElevation);
	  c.a = 1;
	}else{
	  c.r = 0.5;
	  c.g = 0;
	  c.b = 0;
	  c.a = 1;
	}
	marker.colors.push_back(c);
      }
    }
  }

  traversable_map_publisher.publish(marker);
}

sbpl_xy_theta_pt_t EnvironmentNavXYThetaLatFlourish::discreteToContinuous(int x, int y, int theta)
{
  sbpl_xy_theta_pt_t pose;
  pose.x = DISCXY2CONT(x, EnvNAVXYTHETALATCfg.cellsize_m);
  pose.y = DISCXY2CONT(y, EnvNAVXYTHETALATCfg.cellsize_m);
  pose.theta = DiscTheta2Cont(theta, EnvNAVXYTHETALATCfg.NumThetaDirs);
  return pose;
}

bool EnvironmentNavXYThetaLatFlourish::in_full_body_collision(EnvNAVXYTHETALATHashEntry_t* state)
{
  return get_full_body_collision_info(state).collision;
}

const EnvironmentNavXYThetaLatFlourish::FullBodyCollisionInfo& EnvironmentNavXYThetaLatFlourish::get_full_body_collision_info(EnvNAVXYTHETALATHashEntry_t* state)
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
