#include "motion.h"

motion::motion(ros::NodeHandle* node):Node(node){
    string nodename = ros::this_node::getName();
    cout<<"nodename= "<<nodename<<endl;
    Node->getParam(nodename+"/movegroup_name",group);
    // Node->getParam(nodename+"/endLink_name",end_link);
    cout<<"movegroup_name = "<<group<<endl;
    // cout<<"endLink_name = "<<end_link<<endl;
    try{
        move_group = new moveit::planning_interface::MoveGroupInterface(group);
    }catch(std::runtime_error &e){
        ROS_ERROR_STREAM(e.what());
        return ;
    }
    end_link = move_group->getEndEffectorLink();
//    assert(startPos.size() > 0);
    kinematic_model = move_group->getRobotModel();
    kinematic_state = move_group->getCurrentState();
//    kinematic_state->setToDefaultValues();
    joint_model_group = const_cast<robot_state::JointModelGroup*>(kinematic_model->getJointModelGroup(group));
    joint_names = joint_model_group->getJointModelNames();
    move_group->setMaxVelocityScalingFactor(1);
    move_group->setMaxAccelerationScalingFactor(1);
    if(joint_model_group == nullptr){
        return ;
    }

    ROS_INFO_STREAM("initKinematic IS OK");

    motionBridgeStart_server=Node->advertiseService("motionBridgeStart", &motion::motionBridgeStartCB, this);
    test_server=Node->advertiseService("test_server", &motion::testCB, this);
    moveToSiglePose_server=Node->advertiseService("moveToSiglePose", &motion::moveToSiglePoseCB, this);
    moveToMultiPose_server=Node->advertiseService("moveToMultiPose", &motion::moveToMultiPoseCB, this);
    moveLine_server=Node->advertiseService("moveLine", &motion::moveLineCB, this);
    SigleAixs_server=Node->advertiseService("SigleAixs", &motion::moveSigleAixsCB, this);
}



int motion::trajectPlan(moveit_msgs::RobotTrajectory &tempTraject, moveit::planning_interface::MoveGroupInterface &ptr,bool sim) 
{
    robot_trajectory::RobotTrajectory rt(ptr.getCurrentState()->getRobotModel(), group);
    rt.setRobotTrajectoryMsg(*ptr.getCurrentState(), tempTraject);
//    trajectory_processing::IterativeSplineParameterization totg;
    trajectory_processing::IterativeParabolicTimeParameterization totg;
    bool sucess = totg.computeTimeStamps(rt, 0.1, 0.1);
    if(!sucess)
    {
        ROS_ERROR_STREAM("spline.computeTimeStamps ERROR ");
        return -1;
    }
    ROS_INFO_STREAM("spline.computeTimeStamps sucess ");
    moveit::planning_interface::MoveGroupInterface::Plan  plan;

    // plan.trajectory_ = tempTraject;
    rt.getRobotTrajectoryMsg(plan.trajectory_);
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
        for(int i=0; i<plan.trajectory_.joint_trajectory.points.size(); i++)
        {
            ROS_ERROR_STREAM(i<< ": " << plan.trajectory_.joint_trajectory.points[i]);
        }
        return -1;
    }
    ROS_INFO_STREAM("MOVE SUCCESS");
    return 0;
}

int motion::trajectPreparePtp(const std::vector<double> &end, moveit::planning_interface::MoveGroupInterface &ptr,bool sim) {

    std::vector<string> jointName = move_group->getJointNames();
    moveit_msgs::RobotTrajectory tempTraject;
    tempTraject.joint_trajectory.joint_names = std::move(jointName);
    tempTraject.joint_trajectory.points.clear();

    trajectory_msgs::JointTrajectoryPoint p[3];
    std::vector<double> currentPose = move_group->getCurrentJointValues();

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

int motion::trajectPrepareMulti(const vector<vector<double>> &end, moveit::planning_interface::MoveGroupInterface &ptr,bool sim) {
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
        cout<<"temPose_size="<<temPose.size()<<endl;
        cout<<"startPose_size="<<startPose.size()<<endl;
        assert(temPose.size() == startPose.size());
        midPose.resize(6);
        for(int i = 0; i< jointName.size(); i++){
            midPose[i] = (temPose[i] + startPose[i])/2;
        }
        p[0].positions = std::move(startPose);
        p[1].positions = std::move(midPose);

        for(int i = 0; i< 2; i++)
        {
            tempTraject.joint_trajectory.points.push_back(p[i]);
        }
    }

    trajectory_msgs::JointTrajectoryPoint endP;
    endP.positions = end[end.size()-1];
    tempTraject.joint_trajectory.points.push_back(endP);
    return trajectPlan(tempTraject, ptr, sim);
}

bool motion::robot_inverse(geometry_msgs::Pose& pose,vector<double >& output){
    if(!kinematic_state->setFromIK(joint_model_group, pose, end_link, 10, 0.1)){
        ROS_ERROR( "IK error " );
        return false;
    }
    // 返回计算后的关节角
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, output);
    return true;
}

void motion::QtoE(double q1, double q2, double q3, double q0, double& A, double &B, double &C){
    A = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
    B = asin(2*(q0*q2-q1*q3));
    C = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}

bool motion::motionBridgeStartCB(hirop_msgs::motionBridgeStart::Request& req, hirop_msgs::motionBridgeStart::Response& res) {
//    end_link="link6";
//    group="arm0";
    group = req.moveGroup_name.data();
    end_link=req.endLink_name.data();//    end_link = move_group->getEndEffectorLink();
    cout<<group<<endl;
    cout<<end_link<<endl;
    try{
    move_group = new moveit::planning_interface::MoveGroupInterface(group);
    }catch(std::runtime_error &e){
        ROS_ERROR_STREAM(e.what());
        res.is_success= false;
        return true;
    }

//    assert(startPos.size() > 0);
    kinematic_model = move_group->getRobotModel();
    kinematic_state = move_group->getCurrentState();
//    kinematic_state->setToDefaultValues();
    joint_model_group = const_cast<robot_state::JointModelGroup*>(kinematic_model->getJointModelGroup(group));
    joint_names = joint_model_group->getJointModelNames();

    if(joint_model_group == nullptr){
        res.is_success= false;
        return true;
    }

    ROS_INFO_STREAM("initKinematic IS OK");

    res.is_success= true;
    return true;
}


//去到单个点
bool motion::moveToSiglePoseCB(hirop_msgs::moveToSiglePose::Request &req, hirop_msgs::moveToSiglePose::Response &res) {
    vector<double> tmp_pose;
    for (int i = 0; i < req.pose_joints_angle.joints_angle.data.size(); ++i) {
        tmp_pose.push_back(req.pose_joints_angle.joints_angle.data[i]);
    }

    res.is_success = trajectPreparePtp(tmp_pose, *move_group, false) == 0;
    return true;
}

//去到多个点
bool motion::moveToMultiPoseCB(hirop_msgs::moveToMultiPose::Request &req, hirop_msgs::moveToMultiPose::Response &res) {
    vector<vector<double>> poseList;
    for (int i = 0; i < req.poseList_joints_angle.size(); ++i) {
        vector<double> tmp_pose;
        for (int j = 0; j < req.poseList_joints_angle[i].joints_angle.data.size(); ++j) {
            tmp_pose.push_back(req.poseList_joints_angle[i].joints_angle.data[j]);
        }
        poseList.push_back(tmp_pose);
    }
    res.is_success=trajectPrepareMulti(poseList,*move_group, false)==0;
    return true;
}


bool motion::moveLineCB(hirop_msgs::moveLine::Request &req, hirop_msgs::moveLine::Response &res) {
    move_group->setStartStateToCurrentState();
    //1.获取当前位姿
    startPos= move_group->getCurrentJointValues();
    cout<<startPos.size()<<endl;
    kinematic_state->setJointGroupPositions(joint_model_group, startPos);
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(end_link);
    geometry_msgs::Pose SPose;//末端位姿
    tf::poseEigenToMsg(end_effector_state, SPose);

    //2.更新下一点三维坐标值
    geometry_msgs::Pose pose;
    pose.position=SPose.position;
    pose.orientation=SPose.orientation;
    pose.position.x+=req.Cartesian_x;
    pose.position.y+=req.Cartesian_y;
    pose.position.z+=req.Cartesian_z;
    ROS_INFO_STREAM(pose);

//    pose.position.x+=0.1;
//    pose.position.y+=0.1;
//    pose.position.z+=0.1;
    //3.运动学逆解得到关节值
    vector<double > joint_pose;
    vector<double> vector_pose = move_group->getCurrentJointValues();

    cout<<"当前关节角[0]"<<vector_pose[0]<<endl;
    cout<<"当前关节角[1]"<<vector_pose[1]<<endl;
    cout<<"当前关节角[2]"<<vector_pose[2]<<endl;
    cout<<"当前关节角[3]"<<vector_pose[3]<<endl;
    cout<<"当前关节角[4]"<<vector_pose[4]<<endl;
    cout<<"当前关节角[5]"<<vector_pose[5]<<endl;
    // robot_inverse(SPose,joint_pose);
    if(!robot_inverse(pose,joint_pose)){
        res.is_success=false;
        return true;
    }
    cout<<"逆解后[0]"<<joint_pose[0]<<endl;
    cout<<"逆解后[1]"<<joint_pose[1]<<endl;
    cout<<"逆解后[2]"<<joint_pose[2]<<endl;
    cout<<"逆解后[3]"<<joint_pose[3]<<endl;
    cout<<"逆解后[4]"<<joint_pose[4]<<endl;
    cout<<"逆解后[5]"<<joint_pose[5]<<endl;

    res.is_success=trajectPreparePtp(joint_pose,*move_group, false)==0;
    return true;
}

bool motion::moveSigleAixsCB(hirop_msgs::moveSigleAixs::Request &req, hirop_msgs::moveSigleAixs::Response &res) {
    cout<<"开始服务"<<endl;
    //1.获取当前关节角度
    move_group->setStartStateToCurrentState();
    vector<double> joint_angles = move_group->getCurrentJointValues();
    //2.对某一轴进行旋转
    uint16_t index_axis = req.index_axis;
    joint_angles[index_axis] = req.angle;
    // joint_angles[index_axis]+=(req.angle/180)*M_PI;

    //执行运动
    res.is_success=trajectPreparePtp(joint_angles,*move_group, false)==0;
    return true;
}

bool motion::testCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
//    vector<double> p0 = vector<double>({-0.9,-0.86, 2.19,0,1.87,-0.9});
//    ros::ServiceClient client=Node->serviceClient<hirop_msgs::moveSiglePose>("moveToSiglePose");
//    hirop_msgs::moveSiglePose srv;
//    srv.request.joint_pose.verpos.data.resize(6);
//    for(int i=0;i<6;i++){
//        srv.request.joint_pose.verpos.data[i]=p0[i];
//    }
//    if(client.call(srv)){
//        cout<<"服务调用完毕"<<endl;
//    } else{
//        cout<<"服务连接失败"<<endl;
//    }

        vector<double> p1 = vector<double>({-0.9,-0.86, 2.19,0,1.87,-0.9});
        vector<double> p2 = vector<double>({0,-1.5,3.0,0,1.57,3.14});
        vector<double> p3 = vector<double>({0,-1.57,3.14,0,1.57,0});
        vector<vector<double>> pp;
        pp.push_back(p1);pp.push_back(p2);pp.push_back(p3);

        ros::ServiceClient client=Node->serviceClient<hirop_msgs::moveToMultiPose>("moveToMultiPose");
        hirop_msgs::moveToMultiPose srv;
        srv.request.poseList_joints_angle.resize(3);
        for (int j = 0; j <3; ++j) {
            srv.request.poseList_joints_angle[j].joints_angle.data.resize(6);
            for(int i=0;i<6;i++){
                srv.request.poseList_joints_angle[j].joints_angle.data[i]=pp[j][i];
            }
        }
        if(client.call(srv)){
            cout<<"服务调用完毕"<<endl;
        } else{
            cout<<"服务连接失败"<<endl;
        }

//    ros::ServiceClient client=Node->serviceClient<hirop_msgs::moveLine>("moveLine");
//    hirop_msgs::moveLine srv;
//    srv.request.cartesian_x=0;
//    srv.request.cartesian_y=0;
//    srv.request.cartesian_z=0;
//    if(client.call(srv)){
//        cout<<"服务调用完毕"<<endl;
//    } else{
//        cout<<"服务连接失败"<<endl;
//    }

//    ros::ServiceClient client=Node->serviceClient<hirop_msgs::moveSigleAixs>("SigleAixs");
//    hirop_msgs::moveSigleAixs srv;
//    srv.request.index_axis=5;
//    srv.request.angle=10.0;
//    if(client.call(srv)){
//        cout<<"服务调用完毕"<<endl;
//    } else{
//        cout<<"服务连接失败"<<endl;
//    }

//    ros::ServiceClient client=Node->serviceClient<hirop_msgs::motionBridgeStart>("motionBridgeStart");
//    hirop_msgs::motionBridgeStart srv;
//    srv.request.moveGroup_name="arm0";
//    srv.request.endLink_name="link6";
//    if(client.call(srv)){
//        cout<<"服务调用完毕"<<endl;
//    } else{
//        cout<<"服务连接失败"<<endl;
//    }

    return true;
}


