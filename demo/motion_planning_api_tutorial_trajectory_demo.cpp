/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <boost/scoped_ptr.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "moveit/robot_trajectory/robot_trajectory.h"
#include <string>
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit/trajectory_processing/iterative_spline_parameterization.h"
#include "moveit/trajectory_processing/trajectory_tools.h"
#include <vector>


using namespace std;
int trajectPlan(moveit_msgs::RobotTrajectory &tempTraject,moveit::planning_interface::MoveGroupInterface & ptr, bool sim = false)
{
    robot_trajectory::RobotTrajectory rt(ptr.getCurrentState()->getRobotModel(), "arm");
    rt.setRobotTrajectoryMsg(*ptr.getCurrentState(), tempTraject);

//    trajectory_processing::IterativeSplineParameterization totg;
    trajectory_processing::IterativeParabolicTimeParameterization totg;

    bool sucess = totg.computeTimeStamps(rt);

    if(!sucess)
    {
        ROS_ERROR_STREAM("spline.computeTimeStamps ERROR ");
        return -1;
    }
    ROS_ERROR_STREAM("spline.computeTimeStamps sucess ");
    moveit::planning_interface::MoveGroupInterface::Plan  plan;
    rt.getRobotTrajectoryMsg(tempTraject);
    plan.trajectory_ = tempTraject;

    moveit::planning_interface::MoveItErrorCode status;
    if(sim)
    {
        status = ptr.plan(plan);
    }else{
        status = ptr.execute(plan);
    }
    if(status != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_ERROR_STREAM("ptr.execute ERROR ");
        return -1;
    }
    return 0;
}

int trajectPreparePtp( const std::vector<double> & end ,  moveit::planning_interface::MoveGroupInterface & ptr, bool sim = false){
    std::vector<string> jointName = ptr.getJointNames();
    moveit_msgs::RobotTrajectory tempTraject;
    tempTraject.joint_trajectory.joint_names = std::move(jointName);
    tempTraject.joint_trajectory.points.clear();

    trajectory_msgs::JointTrajectoryPoint p[3];
    std::vector<double> currentPose = ptr.getCurrentJointValues();
    assert(end.size() == currentPose.size());

    vector<double> midPose;
    midPose.resize(6);
    for(int i = 0; i< end.size();i++){
       midPose[i] = (end[i] + currentPose[i])/2;
    }
    p[0].positions = std::move(currentPose);
    p[2].positions = std::move(end);
    p[1].positions = std::move(midPose);

    for(int i = 0; i< 3; i++){
        tempTraject.joint_trajectory.points.push_back(p[i]);
    }
    return trajectPlan(tempTraject, ptr,sim);
}


int trajectPrepareMulti( const vector<vector<double>> & end ,  moveit::planning_interface::MoveGroupInterface & ptr, bool sim = false){
    std::vector<string> jointName = ptr.getJointNames();
    moveit_msgs::RobotTrajectory tempTraject;
    tempTraject.joint_trajectory.joint_names = jointName;
    tempTraject.joint_trajectory.points.clear();

    if(end.size() <2 )
        return -2;

    for(int i = 0; i< end.size();i ++){
        vector<double> startPose,midPose,temPose;

        if(i == 0)
            startPose = ptr.getCurrentJointValues();
        else
            startPose = end[i-1];
        temPose = end[i];

        trajectory_msgs::JointTrajectoryPoint p[2];
        assert(temPose.size() == startPose.size());
        midPose.resize(6);
        for(int i = 0; i< jointName.size(); i++){
           midPose[i] = (temPose[i] + startPose[i])/2;
        }
        p[0].positions = std::move(startPose);
        p[1].positions = std::move(midPose);

        for(int i = 0; i< 2; i++){
            tempTraject.joint_trajectory.points.push_back(p[i]);
        }
    }

    trajectory_msgs::JointTrajectoryPoint endP;
    endP.positions = end[end.size()-1];
    tempTraject.joint_trajectory.points.push_back(endP);
    return trajectPlan(tempTraject, ptr, sim);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int i = 0;
    static const std::string PLANNING_GROUP = "arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//    for(int i = 0; i< 10; i++){
//        vector<double> p1 = vector<double>({0,-1.57,3.14,0,1.57,1.57});
//        trajectPreparePtp(p1, move_group);

//        vector<double> p2 = vector<double>({0,-1.57,3.14,0,1.57,3.14});
//        trajectPreparePtp(p2, move_group );


//        vector<double> p3 = vector<double>({0,-1.57,3.14,0,1.57,0});
//        trajectPreparePtp(p3, move_group );
//        sleep(1);
//    }

    while(true){
        i++;
        vector<double> p1 = vector<double>({-0.9,-0.86, 2.19,0,1.87,-0.9});
//        trajectPreparePtp(p1, move_group);

        vector<double> p2 = vector<double>({0,-1.5,3.0,0,1.57,3.14});
//        trajectPreparePtp(p2, move_group );


        vector<double> p3 = vector<double>({0,-1.57,3.14,0,1.57,0});
        vector<vector<double>> pp;
        pp.push_back(p1);pp.push_back(p2);pp.push_back(p3);
        trajectPrepareMulti(pp, move_group );
        sleep(1);
    }

    return 0;
}
