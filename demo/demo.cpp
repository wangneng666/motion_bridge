

#include "motion.h"

motion::motion(ros::NodeHandle* node):Node(node){

}

bool motion::start() {
    end_link="link6";
    group="arm0";
    try{
        move_group = new moveit::planning_interface::MoveGroupInterface(group);
    }catch(std::runtime_error &e){
        ROS_ERROR_STREAM(e.what());
        return false;
    }
//    end_link = move_group->getEndEffectorLink();
//    cout<<"end_link="<<end_link<<endl;
//    move_group->setStartState(*move_group->getCurrentState());


//    assert(startPos.size() > 0);
    kinematic_model = move_group->getRobotModel();
    kinematic_state = move_group->getCurrentState();
    kinematic_state->setToDefaultValues();
    joint_model_group = const_cast<robot_state::JointModelGroup*>(kinematic_model->getJointModelGroup(group));
    joint_names = joint_model_group->getJointModelNames();

    if(joint_model_group == nullptr)
        return false;

    ROS_INFO_STREAM("initKinematic IS OK");

    moveToSiglePose_server=Node->advertiseService("moveToSiglePose", &motion::moveToSiglePoseCB, this);
    moveToMultiPose_server=Node->advertiseService("moveToMultiPose", &motion::moveToMultiPoseCB, this);
    moveLine_server=Node->advertiseService("moveLine", &motion::moveLineCB, this);
    SigleAixs_server=Node->advertiseService("SigleAixs", &motion::moveSigleAixsCB, this);
    test_server=Node->advertiseService("test_server", &motion::testCB, this);

    return true;
}



int motion::trajectPlan(moveit_msgs::RobotTrajectory &tempTraject, moveit::planning_interface::MoveGroupInterface &ptr,bool sim) {
    robot_trajectory::RobotTrajectory rt(ptr.getCurrentState()->getRobotModel(), "arm0");
    rt.setRobotTrajectoryMsg(*ptr.getCurrentState(), tempTraject);

//    trajectory_processing::IterativeSplineParameterization totg;
    trajectory_processing::IterativeParabolicTimeParameterization totg;

    bool sucess = totg.computeTimeStamps(rt);

    if(!sucess)
    {
        ROS_ERROR_STREAM("spline.computeTimeStamps ERROR ");
        return -1;
    }
    ROS_INFO_STREAM("spline.computeTimeStamps sucess ");
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
    cout<<"end.size="<<end.size()<<endl;
    cout<<"currentPose.size="<<currentPose.size()<<endl;
    cout<<"currentPosep[0]="<<currentPose[0]<<endl;
    cout<<"currentPosep[1]="<<currentPose[1]<<endl;
    cout<<"currentPosep[2]="<<currentPose[2]<<endl;
    cout<<"currentPosep[3]="<<currentPose[3]<<endl;
    cout<<"currentPosep[4]="<<currentPose[4]<<endl;
    cout<<"currentPosep[5]="<<currentPose[5]<<endl;

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
        ROS_ERROR( "运动学逆解失败 " );
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

//去到单个点
bool motion::moveToSiglePoseCB(hirop_msgs::moveSiglePose::Request &req, hirop_msgs::moveSiglePose::Response &res) {
    vector<double> pose;
    for (int i = 0; i < req.joint_pose.verpos.data.size(); ++i) {
        pose.push_back(req.joint_pose.verpos.data[i]);
    }
    cout<<"去到单个点"<<endl;
    trajectPreparePtp(pose,*move_group, false);
    res.ret=0;
    return true;
}

//去到多个点
bool motion::moveToMultiPoseCB(hirop_msgs::moveMultiPose::Request &req, hirop_msgs::moveMultiPose::Response &res) {
    vector<vector<double>> poseList;
    for (int i = 0; i < req.joint_poseList.size(); ++i) {
        vector<double> tmp_pose;
        for (int j = 0; j < req.joint_poseList[i].verpos.data.size(); ++j) {
            tmp_pose.push_back(req.joint_poseList[i].verpos.data[j]);
        }
        poseList.push_back(tmp_pose);
    }
    trajectPrepareMulti(poseList,*move_group, false);
    res.ret=0;
    return true;
}


bool motion::moveLineCB(hirop_msgs::moveLine::Request &req, hirop_msgs::moveLine::Response &res) {
    //1.获取当前位姿
//    startPos= move_group->getCurrentJointValues();
//    kinematic_state->setJointGroupPositions(joint_model_group, startPos);
//    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(end_link);
//    tf::poseEigenToMsg(end_effector_state, SPose);
    move_group->setStartStateToCurrentState();
    const geometry_msgs::PoseStamped &stamped = move_group->getCurrentPose(end_link);
//    move_group->getCurrentState()->getpos
    cout<<"x="<<stamped.pose.position.x<<endl;
    cout<<"y="<<stamped.pose.position.y<<endl;
    cout<<"z="<<stamped.pose.position.z<<endl;
    cout<<"orientation_x="<<stamped.pose.orientation.x<<endl;
    cout<<"orientation_y="<<stamped.pose.orientation.y<<endl;
    cout<<"orientation_z="<<stamped.pose.orientation.z<<endl;
    cout<<"orientation_w="<<stamped.pose.orientation.w<<endl;
    double A,B,C;
    QtoE(stamped.pose.orientation.x, stamped.pose.orientation.y, stamped.pose.orientation.z, stamped.pose.orientation.w, A, B, C);
    cout<<"初始欧拉角"<<"a="<<A<<"b="<<B<<"c="<<C<<endl;
    //2.更新下一点三维坐标值
    geometry_msgs::Pose pose;
    pose.position=stamped.pose.position;
    pose.orientation=stamped.pose.orientation;
    pose.position.x+=0;
    pose.position.y+=0;
    pose.position.z+=0;
    QtoE(stamped.pose.orientation.x, stamped.pose.orientation.y, stamped.pose.orientation.z, stamped.pose.orientation.w, A, B, C);
    cout<<"赋值后欧拉角"<<"a="<<A<<" b="<<B<<" c="<<C<<endl;
    //3.运动学逆解得到关节值
    vector<double > joint_pose;
    vector<double> vector_pose = move_group->getCurrentJointValues();
    cout<<"当前关节角[0]"<<vector_pose[0]<<endl;
    cout<<"当前关节角[1]"<<vector_pose[1]<<endl;
    cout<<"当前关节角[2]"<<vector_pose[2]<<endl;
    cout<<"当前关节角[3]"<<vector_pose[3]<<endl;
    cout<<"当前关节角[4]"<<vector_pose[4]<<endl;
    cout<<"当前关节角[5]"<<vector_pose[5]<<endl;
    robot_inverse(pose,joint_pose);
    cout<<"逆解后[0]"<<joint_pose[0]<<endl;
    cout<<"逆解后[1]"<<joint_pose[1]<<endl;
    cout<<"逆解后[2]"<<joint_pose[2]<<endl;
    cout<<"逆解后[3]"<<joint_pose[3]<<endl;
    cout<<"逆解后[4]"<<joint_pose[4]<<endl;
    cout<<"逆解后[5]"<<joint_pose[5]<<endl;

//    trajectPreparePtp(joint_pose,*move_group, false);
    res.ret=0;
    return true;
}

bool motion::moveSigleAixsCB(hirop_msgs::moveSigleAixs::Request &req, hirop_msgs::moveSigleAixs::Response &res) {
    cout<<"开始服务"<<endl;
    //1.获取当前关节角度
    vector<double> joint_angles = move_group->getCurrentJointValues();
    cout<<"getCurrentJointValues[0]"<<joint_angles[0]<<endl;
    cout<<"getCurrentJointValues[1]"<<joint_angles[1]<<endl;
    cout<<"getCurrentJointValues[2]"<<joint_angles[2]<<endl;
    cout<<"getCurrentJointValues[3]"<<joint_angles[3]<<endl;
    cout<<"getCurrentJointValues[4]"<<joint_angles[4]<<endl;
    cout<<"getCurrentJointValues[5]"<<joint_angles[5]<<endl;
    //2.对某一轴进行旋转
    uint16_t index_axis = req.index_axis;
    joint_angles[index_axis]+=(req.angle/180)*M_PI;

    vector<double> joint_radians;
    //3.角度转弧度
//    for (int i = 0; i < joint_angles.size(); ++i) {
//        joint_radians.push_back((joint_angles[i]/180)*M_PI);
//    }
    cout<<"ssss2"<<endl;
    trajectPreparePtp(joint_angles,*move_group, false);
    res.ret=0;
    return true;
}

bool motion::testCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    vector<double> p0 = vector<double>({-0.9,-0.86, 2.19,0,1.87,-0.9});
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

//        vector<double> p1 = vector<double>({-0.9,-0.86, 2.19,0,1.87,-0.9});
//        vector<double> p2 = vector<double>({0,-1.5,3.0,0,1.57,3.14});
//        vector<double> p3 = vector<double>({0,-1.57,3.14,0,1.57,0});
//        vector<vector<double>> pp;
//        pp.push_back(p1);pp.push_back(p2);pp.push_back(p3);
//
//        ros::ServiceClient client=Node->serviceClient<hirop_msgs::moveMultiPose>("moveToMultiPose");
//        hirop_msgs::moveMultiPose srv;
//        srv.request.joint_poseList.resize(3);
//        for (int j = 0; j <3; ++j) {
//            srv.request.joint_poseList[j].verpos.data.resize(6);
//            for(int i=0;i<6;i++){
//                srv.request.joint_poseList[j].verpos.data[i]=pp[j][i];
//            }
//        }
//        if(client.call(srv)){
//            cout<<"服务调用完毕"<<endl;
//        } else{
//            cout<<"服务连接失败"<<endl;
//        }

    ros::ServiceClient client=Node->serviceClient<hirop_msgs::moveLine>("moveLine");
    hirop_msgs::moveLine srv;
    srv.request.cartesian_x=0;
    srv.request.cartesian_y=0;
    srv.request.cartesian_z=0;
    if(client.call(srv)){
        cout<<"服务调用完毕"<<endl;
    } else{
        cout<<"服务连接失败"<<endl;
    }

//    ros::ServiceClient client=Node->serviceClient<hirop_msgs::moveSigleAixs>("SigleAixs");
//    hirop_msgs::moveSigleAixs srv;
//    srv.request.index_axis=5;
//    srv.request.angle=10.0;
//    if(client.call(srv)){
//        cout<<"服务调用完毕"<<endl;
//    } else{
//        cout<<"服务连接失败"<<endl;
//    }
    return true;
}