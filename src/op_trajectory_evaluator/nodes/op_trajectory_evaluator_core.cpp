/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "op_trajectory_evaluator_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include <algorithm>
#include <cmath>

namespace TrajectoryEvaluatorNS
{

TrajectoryEval::TrajectoryEval()
{
  bNewCurrentPos = false;
  bVehicleStatus = false;
  bWayGlobalPath = false;
  bWayGlobalPathToUse = false;
  m_bUseMoveingObjectsPrediction = false;

  // Wall filter defaults
  m_bEnableWallFilter = true;
  m_WallTurnIntensityLevel1 = 0.05;
  m_WallTurnIntensityLevel2 = 0.2;
  m_WallTurnIntensityLevel3 = 0.5;
  m_WallSkipDistGainLevel1  = 1.5;
  m_WallSkipDistGainLevel2  = 2.5;
  m_WallSkipDistGainLevel3  = 4.0;
  m_WallSafetyAttenLevel2   = 0.3;
  m_WallSafetyAttenLevel3   = 0.6;
  m_WallBaseSkipDistance     = 50.0;

  ros::NodeHandle _nh;
  UpdatePlanningParams(_nh);

  tf::StampedTransform transform;
  PlannerHNS::ROSHelpers::GetTransformFromTF("map", "world", transform);
  m_OriginPos.position.x  = transform.getOrigin().x();
  m_OriginPos.position.y  = transform.getOrigin().y();
  m_OriginPos.position.z  = transform.getOrigin().z();

  pub_CollisionPointsRviz = nh.advertise<visualization_msgs::MarkerArray>("dynamic_collision_points_rviz", 1);
  pub_LocalWeightedTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories_eval_rviz", 1);
  pub_LocalWeightedTrajectories = nh.advertise<autoware_msgs::LaneArray>("local_weighted_trajectories", 1);
  pub_TrajectoryCost = nh.advertise<autoware_msgs::Lane>("local_trajectory_cost", 1);
  pub_SafetyBorderRviz = nh.advertise<visualization_msgs::Marker>("safety_border", 1);

  sub_GoalRemainingDistance = nh.subscribe("/goal_remaining_distance", 1, &TrajectoryEval::callbackGetGoalRemainingDistance, this);  

  sub_current_pose = nh.subscribe("/current_pose", 10, &TrajectoryEval::callbackGetCurrentPose, this);

  int bVelSource = 1;
  nh.getParam("/op_motion_predictor/velocitySource", bVelSource);
  if(bVelSource == 0)
    sub_robot_odom = nh.subscribe("/odom", 10, &TrajectoryEval::callbackGetRobotOdom, this);
  else if(bVelSource == 1)
    sub_current_velocity = nh.subscribe("/current_velocity", 10, &TrajectoryEval::callbackGetVehicleStatus, this);
  else if(bVelSource == 2)
    sub_can_info = nh.subscribe("/can_info", 10, &TrajectoryEval::callbackGetCANInfo, this);

  sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &TrajectoryEval::callbackGetGlobalPlannerPath, this);
  sub_LocalPlannerPaths = nh.subscribe("/local_trajectories", 1, &TrajectoryEval::callbackGetLocalPlannerPath, this);
  sub_predicted_objects = nh.subscribe("/predicted_objects", 1, &TrajectoryEval::callbackGetPredictedObjects, this);
  sub_current_behavior = nh.subscribe("/current_behavior", 1, &TrajectoryEval::callbackGetBehaviorState, this);

  PlannerHNS::ROSHelpers::InitCollisionPointsMarkers(50, m_CollisionsDummy);
}

TrajectoryEval::~TrajectoryEval()
{
}

void TrajectoryEval::UpdatePlanningParams(ros::NodeHandle& _nh)
{
  _nh.getParam("/op_trajectory_evaluator/enablePrediction", m_bUseMoveingObjectsPrediction);

  _nh.getParam("/op_common_params/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
  _nh.getParam("/op_common_params/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);
  _nh.getParam("/op_common_params/enableSwerving", m_PlanningParams.enableSwerving);
  if(m_PlanningParams.enableSwerving)
    m_PlanningParams.enableFollowing = true;
  else
    _nh.getParam("/op_common_params/enableFollowing", m_PlanningParams.enableFollowing);

  _nh.getParam("/op_common_params/enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
  _nh.getParam("/op_common_params/enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);

  _nh.getParam("/op_common_params/maxVelocity", m_PlanningParams.maxSpeed);
  _nh.getParam("/op_common_params/minVelocity", m_PlanningParams.minSpeed);
  _nh.getParam("/op_common_params/maxLocalPlanDistance", m_PlanningParams.microPlanDistance);

  _nh.getParam("/op_common_params/pathDensity", m_PlanningParams.pathDensity);

  _nh.getParam("/op_common_params/rollOutDensity", m_PlanningParams.rollOutDensity);
  if(m_PlanningParams.enableSwerving)
    _nh.getParam("/op_common_params/rollOutsNumber", m_PlanningParams.rollOutNumber);
  else
    m_PlanningParams.rollOutNumber = 0;

  std::cout << "Rolls Number: " << m_PlanningParams.rollOutNumber << std::endl;

  _nh.getParam("/op_common_params/horizonDistance", m_PlanningParams.horizonDistance);
  _nh.getParam("/op_common_params/minFollowingDistance", m_PlanningParams.minFollowingDistance);
  _nh.getParam("/op_common_params/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
  _nh.getParam("/op_common_params/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
  _nh.getParam("/op_common_params/speedProfileFactor", m_PlanningParams.speedProfileFactor);

  _nh.getParam("/op_common_params/enableLaneChange", m_PlanningParams.enableLaneChange);

  _nh.getParam("/op_common_params/width", m_CarInfo.width);
  _nh.getParam("/op_common_params/length", m_CarInfo.length);
  _nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
  _nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
  _nh.getParam("/op_common_params/maxSteerAngle", m_CarInfo.max_steer_angle);
  _nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
  _nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);
  m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
  m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;

  // Wall filter parameters (turn intensity based)
  _nh.param("/op_trajectory_evaluator/enableWallFilter", m_bEnableWallFilter, true);
  _nh.param("/op_trajectory_evaluator/wallTurnIntensityLevel1", m_WallTurnIntensityLevel1, 0.05);
  _nh.param("/op_trajectory_evaluator/wallTurnIntensityLevel2", m_WallTurnIntensityLevel2, 0.2);
  _nh.param("/op_trajectory_evaluator/wallTurnIntensityLevel3", m_WallTurnIntensityLevel3, 0.5);
  _nh.param("/op_trajectory_evaluator/wallSkipDistGainLevel1", m_WallSkipDistGainLevel1, 1.5);
  _nh.param("/op_trajectory_evaluator/wallSkipDistGainLevel2", m_WallSkipDistGainLevel2, 2.5);
  _nh.param("/op_trajectory_evaluator/wallSkipDistGainLevel3", m_WallSkipDistGainLevel3, 4.0);
  _nh.param("/op_trajectory_evaluator/wallSafetyAttenLevel2", m_WallSafetyAttenLevel2, 0.3);
  _nh.param("/op_trajectory_evaluator/wallSafetyAttenLevel3", m_WallSafetyAttenLevel3, 0.6);
  _nh.param("/op_trajectory_evaluator/wallBaseSkipDistance", m_WallBaseSkipDistance, 50.0);

  ROS_INFO("[WallFilter] enable=%d, L1=%.3f, L2=%.3f, L3=%.3f",
           m_bEnableWallFilter, m_WallTurnIntensityLevel1, m_WallTurnIntensityLevel2, m_WallTurnIntensityLevel3);
}

void TrajectoryEval::callbackGetGoalRemainingDistance(const std_msgs::Float32ConstPtr &msg)
{
    // m_PlanningParams.remainingDistance = msg->data;
}

void TrajectoryEval::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, /*  */
  tf::getYaw(msg->pose.orientation));
  bNewCurrentPos = true;
}

void TrajectoryEval::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
  m_VehicleStatus.speed = msg->twist.linear.x;
  m_CurrentPos.v = m_VehicleStatus.speed;
  if(fabs(msg->twist.linear.x) > 0.25)
    m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
  UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
  bVehicleStatus = true;
}

void TrajectoryEval::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
  m_VehicleStatus.speed = msg->speed/3.6;
  m_CurrentPos.v = m_VehicleStatus.speed;
  m_VehicleStatus.steer = msg->angle * m_CarInfo.max_steer_angle / m_CarInfo.max_steer_value;
  UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
  bVehicleStatus = true;
}

void TrajectoryEval::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
  m_VehicleStatus.speed = msg->twist.twist.linear.x;
  m_CurrentPos.v = m_VehicleStatus.speed;
  if(fabs(msg->twist.twist.linear.x) > 0.25)
    m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
  UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
  bVehicleStatus = true;
}

void TrajectoryEval::callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
  if(msg->lanes.size() > 0)
  {

    bool bOldGlobalPath = m_GlobalPaths.size() == msg->lanes.size();

    m_GlobalPaths.clear();

    for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
    {
      PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), m_temp_path);

      PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_temp_path);
      m_GlobalPaths.push_back(m_temp_path);

      if(bOldGlobalPath)
      {
        bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(m_temp_path, m_GlobalPaths.at(i));
      }
    }

    if(!bOldGlobalPath)
    {
      bWayGlobalPath = true;
      std::cout << "Received New Global Path Evaluator! " << std::endl;
    }
    else
    {
      m_GlobalPaths.clear();
    }
  }
}

void TrajectoryEval::callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
  if(msg->lanes.size() > 0)
  {
    m_GeneratedRollOuts.clear();
    int globalPathId_roll_outs = -1;

    for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
    {
      std::vector<PlannerHNS::WayPoint> path;
      PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), path);
      m_GeneratedRollOuts.push_back(path);
      if(path.size() > 0)
        globalPathId_roll_outs = path.at(0).gid;
    }

    if(bWayGlobalPath && m_GlobalPaths.size() > 0 && m_GlobalPaths.at(0).size() > 0)
    {
      int globalPathId = m_GlobalPaths.at(0).at(0).gid;
      std::cout << "Before Synchronization At Trajectory Evaluator: GlobalID: " <<  globalPathId << ", LocalID: " << globalPathId_roll_outs << std::endl;

      if(globalPathId_roll_outs == globalPathId)
      {
        bWayGlobalPath = false;
        m_GlobalPathsToUse = m_GlobalPaths;
        std::cout << "Synchronization At Trajectory Evaluator: GlobalID: " <<  globalPathId << ", LocalID: " << globalPathId_roll_outs << std::endl;
      }
    }

    bRollOuts = true;
  }
}

void TrajectoryEval::callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
  m_PredictedObjects.clear();
  bPredictedObjects = true;

  PlannerHNS::DetectedObject obj;
  for(unsigned int i = 0 ; i <msg->objects.size(); i++)
  {
    if(msg->objects.at(i).id > 0)
    {
      PlannerHNS::ROSHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(msg->objects.at(i), obj);
      m_PredictedObjects.push_back(obj);
    }
  }
}

void TrajectoryEval::callbackGetBehaviorState(const geometry_msgs::TwistStampedConstPtr& msg)
{
  m_CurrentBehavior.iTrajectory = msg->twist.angular.z;
}

double TrajectoryEval::ComputeTurnIntensity()
{
  // yaw_rate = |speed * tan(steer) / wheel_base|
  if(fabs(m_VehicleStatus.steer) < 1e-4)
    return 0.0;

  // When speed is near zero but steer is non-zero (in-place turning), use steer as proxy
  if(fabs(m_VehicleStatus.speed) < 0.1)
  {
    return fabs(m_VehicleStatus.steer) / m_CarInfo.max_steer_angle * m_WallTurnIntensityLevel3;
  }

  double yaw_rate = fabs(m_VehicleStatus.speed * tan(m_VehicleStatus.steer) / m_CarInfo.wheel_base);
  return yaw_rate;
}

void TrajectoryEval::FilterWallObjects(const std::vector<PlannerHNS::DetectedObject>& input,
                                        std::vector<PlannerHNS::DetectedObject>& output)
{
  double turn_intensity = ComputeTurnIntensity();

  // If wall filter is disabled or not turning, pass through all objects
  if(!m_bEnableWallFilter || turn_intensity < m_WallTurnIntensityLevel1)
  {
    output = input;
    return;
  }

  // Determine turn level and corresponding skip distance gain
  double skip_dist_gain = 1.0;
  if(turn_intensity >= m_WallTurnIntensityLevel3)
    skip_dist_gain = m_WallSkipDistGainLevel3;
  else if(turn_intensity >= m_WallTurnIntensityLevel2)
    skip_dist_gain = m_WallSkipDistGainLevel2;
  else if(turn_intensity >= m_WallTurnIntensityLevel1)
    skip_dist_gain = m_WallSkipDistGainLevel1;

  double effective_skip_dist = m_WallBaseSkipDistance * skip_dist_gain;

  // Turn direction: positive steer = left turn, negative = right turn
  int turn_direction = (m_VehicleStatus.steer > 0) ? 1 : -1;

  int filter_count = 0;
  for(unsigned int i = 0; i < input.size(); i++)
  {
    const PlannerHNS::DetectedObject& obj = input.at(i);

    // Calculate object center distance from vehicle
    double obj_dx = obj.center.pos.x - m_CurrentPos.pos.x;
    double obj_dy = obj.center.pos.y - m_CurrentPos.pos.y;
    double obj_dist = hypot(obj_dx, obj_dy);

    // Calculate object angle relative to vehicle heading
    double angle_to_obj = atan2(obj_dy, obj_dx);
    double heading_diff = angle_to_obj - m_CurrentPos.pos.a;
    // Normalize to [-PI, PI]
    while(heading_diff > M_PI) heading_diff -= 2.0*M_PI;
    while(heading_diff < -M_PI) heading_diff += 2.0*M_PI;

    // Determine if object is on turn-inside side
    bool is_turn_inside = (turn_direction * heading_diff > 0);

    // Determine if object is mostly behind or to the side (not in front)
    bool is_behind_or_side = (fabs(heading_diff) > M_PI_2);

    // Wall-like geometric check: long and thin
    double max_dim = std::max(obj.w, obj.l);
    double min_dim = std::min(obj.w, obj.l);
    bool is_wall_like = false;
    if(min_dim > 0.01) // avoid division by zero
    {
      double aspect_ratio = max_dim / min_dim;
      is_wall_like = (aspect_ratio > 3.0 && min_dim < 1.0);
    }

    bool should_filter = false;

    // Level 3 (sharp turn): filter turn-inside and side objects
    if(turn_intensity >= m_WallTurnIntensityLevel3)
    {
      if(is_turn_inside || is_behind_or_side)
        should_filter = true;
    }
    // Level 2 (medium turn): filter turn-inside + wall-like, or side + wall-like
    else if(turn_intensity >= m_WallTurnIntensityLevel2)
    {
      if((is_turn_inside || is_behind_or_side) && is_wall_like)
        should_filter = true;
      else if(is_turn_inside && obj_dist > effective_skip_dist)
        should_filter = true;
    }
    // Level 1 (light turn): only filter wall-like objects on turn-inside
    else
    {
      if(is_turn_inside && is_wall_like && obj_dist > m_WallBaseSkipDistance)
        should_filter = true;
    }

    if(should_filter)
    {
      filter_count++;
      ROS_DEBUG("[WallFilter] Obj %d filtered: turn_intensity=%.3f, dist=%.2f, heading_diff=%.2f, wall_like=%d",
                i, turn_intensity, obj_dist, heading_diff, is_wall_like);
      continue;
    }

    output.push_back(obj);
  }

  if(filter_count > 0)
  {
    ROS_INFO("[WallFilter] Filtered %d/%zu objects, turn_intensity=%.3f",
             filter_count, input.size(), turn_intensity);
  }
}

void TrajectoryEval::MainLoop()
{
  ros::Rate loop_rate(30);

  PlannerHNS::WayPoint prevState, state_change;

  while (ros::ok())
  {
    ros::spinOnce();
    PlannerHNS::TrajectoryCost tc;

    if(bNewCurrentPos && m_GlobalPaths.size()>0)
    {
      m_GlobalPathSections.clear();

      for(unsigned int i = 0; i < m_GlobalPathsToUse.size(); i++)
      {
        t_centerTrajectorySmoothed.clear();
        PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_GlobalPathsToUse.at(i), m_CurrentPos, m_PlanningParams.horizonDistance , m_PlanningParams.pathDensity ,t_centerTrajectorySmoothed);
        m_GlobalPathSections.push_back(t_centerTrajectorySmoothed);
      }

      if(m_GlobalPathSections.size()>0)
      {
        // Apply wall filter before trajectory cost calculation
        std::vector<PlannerHNS::DetectedObject> filteredObjects;
        FilterWallObjects(m_PredictedObjects, filteredObjects);

        if(0)//m_bUseMoveingObjectsPrediction)
          tc = m_TrajectoryCostsCalculator.DoOneStepDynamic(m_GeneratedRollOuts, m_GlobalPathSections.at(0), m_CurrentPos,m_PlanningParams,  m_CarInfo,m_VehicleStatus, filteredObjects, m_CurrentBehavior.iTrajectory);
        else
          tc = m_TrajectoryCostsCalculator.DoOneStepStatic(m_GeneratedRollOuts, m_GlobalPathSections.at(0), m_CurrentPos,  m_PlanningParams,  m_CarInfo,m_VehicleStatus, filteredObjects);
        
        autoware_msgs::Lane l;
        l.closest_object_distance = tc.closest_obj_distance;
        l.closest_object_velocity = tc.closest_obj_velocity;
        l.cost = tc.cost;
        l.is_blocked = tc.bBlocked;
        l.lane_index = tc.index;
        pub_TrajectoryCost.publish(l);
      }

      if(m_TrajectoryCostsCalculator.m_TrajectoryCosts.size() == m_GeneratedRollOuts.size())
      {
        autoware_msgs::LaneArray local_lanes;
        for(unsigned int i=0; i < m_GeneratedRollOuts.size(); i++)
        {
          autoware_msgs::Lane lane;
          PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_GeneratedRollOuts.at(i), lane);
          lane.closest_object_distance = m_TrajectoryCostsCalculator.m_TrajectoryCosts.at(i).closest_obj_distance;
          lane.closest_object_velocity = m_TrajectoryCostsCalculator.m_TrajectoryCosts.at(i).closest_obj_velocity;
          lane.cost = m_TrajectoryCostsCalculator.m_TrajectoryCosts.at(i).cost;
          lane.is_blocked = m_TrajectoryCostsCalculator.m_TrajectoryCosts.at(i).bBlocked;
          lane.lane_index = i;
          local_lanes.lanes.push_back(lane);
        }

        pub_LocalWeightedTrajectories.publish(local_lanes);
      }
      else
      {
        ROS_ERROR("m_TrajectoryCosts.size() Not Equal m_GeneratedRollOuts.size()");
      }

      if(m_TrajectoryCostsCalculator.m_TrajectoryCosts.size()>0)
      {
        visualization_msgs::MarkerArray all_rollOuts;
        PlannerHNS::ROSHelpers::TrajectoriesToColoredMarkers(m_GeneratedRollOuts, m_TrajectoryCostsCalculator.m_TrajectoryCosts, m_CurrentBehavior.iTrajectory, all_rollOuts);
        pub_LocalWeightedTrajectoriesRviz.publish(all_rollOuts);

        PlannerHNS::ROSHelpers::ConvertCollisionPointsMarkers(m_TrajectoryCostsCalculator.m_CollisionPoints, m_CollisionsActual, m_CollisionsDummy);
        pub_CollisionPointsRviz.publish(m_CollisionsActual);

        //Visualize Safety Box
        visualization_msgs::Marker safety_box;
        PlannerHNS::ROSHelpers::ConvertFromPlannerHRectangleToAutowareRviz(m_TrajectoryCostsCalculator.m_SafetyBorder.points, safety_box);
        pub_SafetyBorderRviz.publish(safety_box);
      }
    }
    else
      sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array",   1,    &TrajectoryEval::callbackGetGlobalPlannerPath,   this);

    loop_rate.sleep();
  }
}

}