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
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>
#include <string>
#include <thread>
#define GROUP0 "arm0"
#define GROUP1 "arm1"


using namespace std;

int trajectPlan(moveit_msgs::RobotTrajectory &tempTraject,moveit::planning_interface::MoveGroupInterface & ptr, const string & groupName, bool sim = false)
{
    robot_trajectory::RobotTrajectory rt(ptr.getCurrentState()->getRobotModel(), groupName);
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

int createTrajectPlan(moveit_msgs::RobotTrajectory &tempTraject,moveit::planning_interface::MoveGroupInterface & ptr, const string & groupName, bool sim = false)
{
    robot_trajectory::RobotTrajectory rt(ptr.getCurrentState()->getRobotModel(), groupName);
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
    rt.getRobotTrajectoryMsg(tempTraject);

}


int trajectPrepareMultiSingleRobot( const vector<vector<double>> & end ,  moveit::planning_interface::MoveGroupInterface & ptr, bool sim = false){
    std::vector<string> jointName = ptr.getJointNames();
    moveit_msgs::RobotTrajectory tempTraject;
    tempTraject.joint_trajectory.joint_names = jointName;
    tempTraject.joint_trajectory.points.clear();

    if(end.size() <2 )
        return -2;

    for(int i = 0; i< end.size();i ++){
        vector<double> startPose,midPose,temPose;

        if(i == 0){

            startPose = ptr.getCurrentJointValues();

            for(int i = 0; i< jointName.size(); i++)
                startPose.push_back(startPose.at(i));
        }
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
    return trajectPlan(tempTraject, ptr,  GROUP0, sim);
}

void setStartState(moveit::planning_interface::MoveGroupInterface & ptr)
{
    ptr.setStartStateToCurrentState();
    robot_state::RobotState r(*ptr.getCurrentState());
    const robot_state::JointModelGroup* jointGroup = r.getJointModelGroup(ptr.getName());
    std::vector<double> joints = ptr.getCurrentJointValues();
    r.setJointGroupPositions(jointGroup, joints);
    ptr.setStartState(r);
}


int trajectPrepareMulti( const vector<vector<double>> & end ,
                         moveit::planning_interface::MoveGroupInterface & ptr,
                         trajectory_msgs::JointTrajectory& createTraj ,bool sim = false){
    std::vector<string> jointName = ptr.getJointNames();

    moveit_msgs::RobotTrajectory tempTraject;
    tempTraject.joint_trajectory.joint_names = jointName;
    tempTraject.joint_trajectory.points.clear();

    if(end.size() <2 )
        return -2;

    for(int i = 0; i< end.size();i ++){
        vector<double> startPose,midPose,temPose;

        if(i == 0){
            setStartState(ptr);
            startPose = ptr.getCurrentJointValues();
            std::cout << " startPose: "<<startPose[0]<<" "<<startPose[1]<<" "<<std::endl;
        }
        else
            startPose = end[i-1];
        temPose = end[i];

        trajectory_msgs::JointTrajectoryPoint p[2];
        assert(temPose.size() == startPose.size());
        midPose.resize(6);
        for(int i = 0; i< jointName.size(); i++){
           midPose[i] = (temPose[i] + startPose[i])/2;
        }
        p[0].positions = startPose;
        p[1].positions = std::move(midPose);

        for(int i = 0; i< 2; i++){
            tempTraject.joint_trajectory.points.push_back(p[i]);
        }
    }

    trajectory_msgs::JointTrajectoryPoint endP;
    endP.positions = end[end.size()-1];
    tempTraject.joint_trajectory.points.push_back(endP);

    createTrajectPlan(tempTraject, ptr, ptr.getName());
    createTraj.joint_names = tempTraject.joint_trajectory.joint_names;
    createTraj.points = tempTraject.joint_trajectory.points;

}


void trajectRobotMerge(const moveit_msgs::RobotTrajectory& trajectPath1,
                       const moveit_msgs::RobotTrajectory& trajectPath2,
                       moveit_msgs::RobotTrajectory& trajectPathOut)
{
    for(int i = 0; i< trajectPath1.joint_trajectory.points.size();  i++)
    {
        trajectory_msgs::JointTrajectoryPoint p[3];
        p[0] = trajectPath1.joint_trajectory.points.at(i);
        p[1] = trajectPath2.joint_trajectory.points.at(i);
        vector<double> merge(12);
        copy(p[0].positions.begin(),p[0].positions.end(), merge.begin());
        copy(p[1].positions.begin(),p[1].positions.end(), merge.begin()+6);
        p[3].positions = std::move(merge);

        merge.resize(12);
        copy(p[0].velocities.begin(),p[0].velocities.end(), merge.begin());
        copy(p[1].velocities.begin(),p[1].velocities.end(), merge.begin()+6);
        p[3].positions = std::move(merge);

        merge.resize(12);
        copy(p[0].accelerations.begin(),p[0].accelerations.end(), merge.begin());
        copy(p[1].accelerations.begin(),p[1].accelerations.end(), merge.begin()+6);
        p[3].positions = std::move(merge);

//        trajectPathOut.joint_trajectory.points.
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int i = 0;
//    static const std::string PLANNING_GROUP = "arm0";

    moveit::planning_interface::MoveGroupInterface move_group0(GROUP0);
    moveit::planning_interface::MoveGroupInterface move_group1(GROUP1);

    ros::Publisher pub0 = node_handle.advertise<trajectory_msgs::JointTrajectory>("/UR51/joint_path_command",1);
    ros::Publisher pub1 = node_handle.advertise<trajectory_msgs::JointTrajectory>("/UR52/joint_path_command",1);


    vector<double> p1 = vector<double>({-0.9,-0.86, 2.19,0,1.87,-0.9});

    vector<double> p2 = vector<double>({0,-1.5,3.0,0,1.57,3.14});

    vector<double> p3 = vector<double>({0,-1.57,3.14,0,1.57,0});

    vector<vector<double>> pp;
    pp.push_back(p1);pp.push_back(p2);pp.push_back(p3);


    trajectory_msgs::JointTrajectory createTraj0,createTraj1;

//    for(int i = 0; i< 100; i++){
    // move_group0
    trajectPrepareMulti(pp, move_group0,createTraj0 );

    //move_group1
    trajectPrepareMulti(pp, move_group1,createTraj1 );
//    }
    pub0.publish(createTraj0);
    pub1.publish(createTraj1);


    //
    sleep(1);



    return 0;
}
