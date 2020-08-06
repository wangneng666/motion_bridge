
#ifndef MOTION_BRIDGE_MOTION_H
#define MOTION_BRIDGE_MOTION_H

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
#include <tf/LinearMath/Quaternion.h>
#include "tf/tf.h"
//消息类型
#include "hirop_msgs/moveToSiglePose.h"
#include "hirop_msgs/moveToMultiPose.h"
#include "hirop_msgs/moveSigleAixs.h"
#include "hirop_msgs/moveLine.h"
#include "hirop_msgs/motionBridgeStart.h"
#include "geometry_msgs/Pose.h"
#include "std_srvs/Empty.h"
#include <eigen_conversions/eigen_msg.h>
#include "std_srvs/SetBool.h"
#include "hirop_msgs/dualRbtraject.h"
#include "industrial_msgs/RobotStatus.h"
#include <atomic>
using namespace std;


struct MoveGroup{
    moveit::planning_interface::MoveGroupInterface *move_group;
    robot_model::RobotModelConstPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;
    std::vector<std::string> joint_names;
    std::string groupName;
    string endlinkName;
    //std::atomic<int> motion_val;
    int motion_val;
};


class motion {
public:
    motion(ros::NodeHandle* node);

public:
    //生成move_group
    void generateMoveGroup(string groupName,string endLinkName);
    //运动规划
    int trajectPlan(moveit_msgs::RobotTrajectory &tempTraject, MoveGroup& MG, bool sim = false);

    //单点运动
    int trajectPreparePtp( const std::vector<double> & end , MoveGroup& MG, bool sim = false);

    //运动多点
    int trajectPrepareMulti( const vector<vector<double>> & end , MoveGroup& MG, bool sim = false);

    //运动学逆解
    bool robot_inverse(MoveGroup& MG,geometry_msgs::Pose& poseList,vector<double >& output);

private:
    ros::NodeHandle* Node;
    vector<double> startPos ;
    vector<MoveGroup> MoveGroupList;

    bool ass_flag_leftRbMotion=false;
    bool ass_flag_rightRbMotion=false;


    ros::Publisher pub_robLeft;
    ros::Publisher pub_robRight;
    ros::Subscriber rb0_robotstatus_subscriber;
    ros::Subscriber rb1_robotstatus_subscriber;


    ros::ServiceServer dualRobMotion_server;//双臂运动服务
    ros::ServiceServer motionBridgeStart_server;//运动桥启动服务
    ros::ServiceServer moveToSiglePose_server;//单点运动服务
    ros::ServiceServer moveToMultiPose_server;//多点运动服务
    ros::ServiceServer moveLine_server;       //直线运动服务
    ros::ServiceServer SigleAixs_server;      //单轴运动服务

    ros::ServiceServer test_server;

    bool dualRobMotion(hirop_msgs::dualRbtraject::Request& req, hirop_msgs::dualRbtraject::Response& res);

    void callback_rob0Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status);
    void callback_rob1Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status);

    bool motionBridgeStartCB(hirop_msgs::motionBridgeStart::Request& req, hirop_msgs::motionBridgeStart::Response& res);
    /***
     *单点运动
     * @param req
     * @param res
     * @return
     */
    bool moveToSiglePoseCB(hirop_msgs::moveToSiglePose::Request& req, hirop_msgs::moveToSiglePose::Response& res);

    /***
     *多点平滑运动
     * @param req
     * @param res
     * @return
     */
    bool moveToMultiPoseCB(hirop_msgs::moveToMultiPose::Request& req, hirop_msgs::moveToMultiPose::Response& res);

    /***
     * 直线运动（笛卡尔）
     * @param req
     * @param res
     * @return
     */
    bool moveLineCB(hirop_msgs::moveLine::Request& req, hirop_msgs::moveLine::Response& res);

    /***
     * 单轴运动
     * @param req
     * @param res
     * @return
     */
    bool moveSigleAixsCB(hirop_msgs::moveSigleAixs::Request& req, hirop_msgs::moveSigleAixs::Response& res);

    bool testCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    int trajectPrepareMulti( const vector<vector<double>> & end ,
                             moveit::planning_interface::MoveGroupInterface & ptr,
                             trajectory_msgs::JointTrajectory& createTraj );

    void setStartState(moveit::planning_interface::MoveGroupInterface & ptr);

    int createTrajectPlan(moveit_msgs::RobotTrajectory &tempTraject,moveit::planning_interface::MoveGroupInterface & ptr, const string & groupName);

};


#endif //MOTION_BRIDGE_MOTION_H
