#include "motion.h"

motion::motion(ros::NodeHandle* node):Node(node){
    generateMoveGroup("arm0","link6");
    generateMoveGroup("arm1","link6");
    ass_flag_leftRbMotion=false;
    ass_flag_rightRbMotion=false;

    pub_robLeft  =Node->advertise<trajectory_msgs::JointTrajectory>("/UR51/joint_path_command",1);
    pub_robRight =Node->advertise<trajectory_msgs::JointTrajectory>("/UR52/joint_path_command",1);

    rb0_robotstatus_subscriber=Node->subscribe<industrial_msgs::RobotStatus>("/UR51/robot_status",1,boost::bind(&motion::callback_rob0Status_subscriber,this,_1));
    rb1_robotstatus_subscriber=Node->subscribe<industrial_msgs::RobotStatus>("/UR52/robot_status",1,boost::bind(&motion::callback_rob1Status_subscriber,this,_1));

    dualRobMotion_server=Node->advertiseService("motion_bridge/dualRobMotion_JointTraject", &motion::dualRobMotion, this);
    motionBridgeStart_server=Node->advertiseService("motion_bridge/motionBridgeStart", &motion::motionBridgeStartCB, this);
    test_server=Node->advertiseService("motion_bridge/test_server", &motion::testCB, this);
    moveToSiglePose_server=Node->advertiseService("motion_bridge/moveToSiglePose", &motion::moveToSiglePoseCB, this);
    moveToMultiPose_server=Node->advertiseService("motion_bridge/moveToMultiPose", &motion::moveToMultiPoseCB, this);
    moveLine_server=Node->advertiseService("motion_bridge/moveLine", &motion::moveLineCB, this);
    SigleAixs_server=Node->advertiseService("motion_bridge/SigleAixs", &motion::moveSigleAixsCB, this);
}

void motion::generateMoveGroup(string groupName,string endLinkName){
    MoveGroup tmp_MoveGroup;
    tmp_MoveGroup.groupName=groupName;
    tmp_MoveGroup.endlinkName=endLinkName;
    try{
        tmp_MoveGroup.move_group = new moveit::planning_interface::MoveGroupInterface(groupName);
    }catch(std::runtime_error &e){
        ROS_ERROR_STREAM(e.what());
        return ;
    }
    tmp_MoveGroup.endlinkName=endLinkName;
//    assert(startPos.size() > 0);
    tmp_MoveGroup.kinematic_model = tmp_MoveGroup.move_group->getRobotModel();
    tmp_MoveGroup.kinematic_state = tmp_MoveGroup.move_group->getCurrentState();
//    kinematic_state->setToDefaultValues();
    tmp_MoveGroup.joint_model_group = const_cast<robot_state::JointModelGroup*>(tmp_MoveGroup.kinematic_model->getJointModelGroup(tmp_MoveGroup.groupName));
    tmp_MoveGroup.joint_names = tmp_MoveGroup.joint_model_group->getJointModelNames();
    if(tmp_MoveGroup.joint_model_group == nullptr){
        return ;
    }
    MoveGroupList.push_back(tmp_MoveGroup);
    ROS_INFO_STREAM(groupName<<" initKinematic IS OK");
}

bool motion::dualRobMotion(hirop_msgs::dualRbtraject::Request& req, hirop_msgs::dualRbtraject::Response& res){

//    for (int i = 0; i <req.robotMotionTraject_list.size(); ++i) {
//        cout<<"机器人数量"<<endl
//        if(req.robotMotionTraject_list[i].moveGroup_name==MoveGroupList[i].groupName){
//            pub_robLeft.publish(req.robotMotionTraject_list[i].robot_jointTra);
//        }
//    }
    bool flag_leftRbMotion=false;
    bool flag_rightRbMotion=false;
    ass_flag_leftRbMotion=false;
    ass_flag_rightRbMotion=false;
    if(req.robotMotionTraject_list[0].moveGroup_name==MoveGroupList[0].groupName){
        if(req.robotMotionTraject_list[0].robot_jointTra.points.size()>0){
            pub_robLeft.publish(req.robotMotionTraject_list[0].robot_jointTra);
            flag_leftRbMotion=true;
            ROS_INFO_STREAM("left robot motion");
        }

    }
    if(req.robotMotionTraject_list[1].moveGroup_name==MoveGroupList[1].groupName){
        if(req.robotMotionTraject_list[1].robot_jointTra.points.size()>0){
            pub_robRight.publish(req.robotMotionTraject_list[1].robot_jointTra);
            ROS_INFO_STREAM("right robot motion");
            flag_rightRbMotion=true;
        }

    }
    if(!flag_leftRbMotion){
        ass_flag_leftRbMotion=true;
    }
    if(!flag_rightRbMotion){
        ass_flag_rightRbMotion=true;
    }
    //启动监听
//    ROS_INFO_STREAM("dual motion running0");
    std::atomic<bool> isstop;
    isstop = false;
    int count=0;
    while(!isstop){
        //isstop=(MoveGroupList[0].motion==1)&&(MoveGroupList[1].motion==1);
        //isstop=((MoveGroupList[0].motion_val==1)||!flag_leftRbMotion)&&((MoveGroupList[1].motion_val==1)||!flag_rightRbMotion);
        //isstop=(ass_flag_leftRbMotion||(!flag_leftRbMotion))&&(ass_flag_rightRbMotion||(!flag_rightRbMotion));
        isstop=ass_flag_leftRbMotion&&ass_flag_rightRbMotion;
//        ROS_INFO_STREAM(ass_flag_leftRbMotion<<ass_flag_rightRbMotion);
        usleep(100);
        count++;
        if(count>10000*10){
            isstop=true;
            ROS_INFO_STREAM("timer is ok");
        }
    }

    ROS_INFO_STREAM("dual motion running1");
//    sleep(2);
    isstop= false;
    while(!isstop){
        isstop=(MoveGroupList[0].motion_val==0)&&(MoveGroupList[1].motion_val==0);
        usleep(100);
    }
    ROS_INFO_STREAM("dual motion run over");
    res.is_success=true;

}

int motion::trajectPlan(moveit_msgs::RobotTrajectory &tempTraject, MoveGroup& MG,bool sim)
{
    robot_trajectory::RobotTrajectory rt(MG.move_group->getCurrentState()->getRobotModel(), MG.groupName);
    rt.setRobotTrajectoryMsg(*MG.move_group->getCurrentState(), tempTraject);
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
        status = MG.move_group->plan(plan);
    }else{
        status = MG.move_group->execute(plan);
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

int motion::trajectPreparePtp(const std::vector<double> &end,  MoveGroup& MG,bool sim) {

    std::vector<string> jointName = MG.move_group->getJointNames();
    moveit_msgs::RobotTrajectory tempTraject;
    tempTraject.joint_trajectory.joint_names = std::move(jointName);
    tempTraject.joint_trajectory.points.clear();

    trajectory_msgs::JointTrajectoryPoint p[3];
    std::vector<double> currentPose = MG.move_group->getCurrentJointValues();

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
    return trajectPlan(tempTraject, MG,sim);

}

int motion::trajectPrepareMulti(const vector<vector<double>> &end,  MoveGroup& MG,bool sim) {
    std::vector<string> jointName = MG.move_group->getJointNames();
    moveit_msgs::RobotTrajectory tempTraject;
    tempTraject.joint_trajectory.joint_names = jointName;
    tempTraject.joint_trajectory.points.clear();

    if(end.size() <2 )
        return -2;

    for(int i = 0; i< end.size();i ++){
        vector<double> startPose,midPose,temPose;

        if(i == 0)
            startPose = MG.move_group->getCurrentJointValues();
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
    return trajectPlan(tempTraject, MG, sim);
}

bool motion::robot_inverse(MoveGroup& MG,geometry_msgs::Pose& pose,vector<double >& output){
    if(!MG.kinematic_state->setFromIK(MG.joint_model_group, pose, MG.endlinkName, 10, 0.1)){
        ROS_ERROR( "IK error " );
        return false;
    }
    // 返回计算后的关节角
    std::vector<double> joint_values;
    MG.kinematic_state->copyJointGroupPositions(MG.joint_model_group, output);
    return true;
}


bool motion::motionBridgeStartCB(hirop_msgs::motionBridgeStart::Request& req, hirop_msgs::motionBridgeStart::Response& res) {

//    try{
//    move_group = new moveit::planning_interface::MoveGroupInterface(group);
//    }catch(std::runtime_error &e){
//        ROS_ERROR_STREAM(e.what());
//        res.is_success= false;
//        return true;
//    }
//
////    assert(startPos.size() > 0);
//    kinematic_model = move_group->getRobotModel();
//    kinematic_state = move_group->getCurrentState();
////    kinematic_state->setToDefaultValues();
//    joint_model_group = const_cast<robot_state::JointModelGroup*>(kinematic_model->getJointModelGroup(group));
//    joint_names = joint_model_group->getJointModelNames();
//
//    if(joint_model_group == nullptr){
//        res.is_success= false;
//        return true;
//    }

    ROS_INFO_STREAM("initKinematic IS OK  ");

    res.is_success= true;
    return true;
}


//去到单个点
bool motion::moveToSiglePoseCB(hirop_msgs::moveToSiglePose::Request &req, hirop_msgs::moveToSiglePose::Response &res) {
    int robot_num=-1;
    for (int j = 0; j <MoveGroupList.size(); ++j) {
            if(MoveGroupList[j].groupName==req.moveGroup_name){
                robot_num=j;
        }
    }
    if(robot_num==-1){
        res.is_success= false;
        res.info="此move_group找不到";
        return true;
    }

    vector<double> tmp_pose;
    for (int i = 0; i < req.pose_joints_angle.joints_angle.data.size(); ++i) {
        tmp_pose.push_back(req.pose_joints_angle.joints_angle.data[i]);
    }

    res.is_success = trajectPreparePtp(tmp_pose, MoveGroupList[robot_num], false) == 0;
    return true;
}

//去到多个点
bool motion::moveToMultiPoseCB(hirop_msgs::moveToMultiPose::Request &req, hirop_msgs::moveToMultiPose::Response &res) {
    int robot_num=-1;
    for (int j = 0; j <MoveGroupList.size(); ++j) {
        if(MoveGroupList[j].groupName==req.moveGroup_name){
            robot_num=j;
        }
    }
    if(robot_num==-1){
        res.is_success= false;
        res.info="此move_group找不到";
        return true;
    }
    vector<vector<double>> poseList;
    for (int i = 0; i < req.poseList_joints_angle.size(); ++i) {
        vector<double> tmp_pose;
        for (int j = 0; j < req.poseList_joints_angle[i].joints_angle.data.size(); ++j) {
            tmp_pose.push_back(req.poseList_joints_angle[i].joints_angle.data[j]);
        }
        poseList.push_back(tmp_pose);
    }
    res.is_success=trajectPrepareMulti(poseList,MoveGroupList[robot_num], false)==0;
    return true;
}


bool motion::moveLineCB(hirop_msgs::moveLine::Request &req, hirop_msgs::moveLine::Response &res) {
    int robot_num=-1;
    for (int j = 0; j <MoveGroupList.size(); ++j) {
        if(MoveGroupList[j].groupName==req.moveGroup_name){
            robot_num=j;
        }
    }
    if(robot_num==-1){
        res.is_success= false;
        res.info="此move_group找不到";
        return true;
    }
    MoveGroupList[robot_num].move_group->setStartStateToCurrentState();
    //1.获取当前位姿
    startPos= MoveGroupList[robot_num].move_group->getCurrentJointValues();
    cout<<startPos.size()<<endl;
    MoveGroupList[robot_num].kinematic_state->setJointGroupPositions(MoveGroupList[robot_num].joint_model_group, startPos);
    const Eigen::Affine3d &end_effector_state = MoveGroupList[robot_num].kinematic_state->getGlobalLinkTransform(MoveGroupList[robot_num].endlinkName);
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
    vector<double> vector_pose = MoveGroupList[robot_num].move_group->getCurrentJointValues();

    cout<<"当前关节角[0]"<<vector_pose[0]<<endl;
    cout<<"当前关节角[1]"<<vector_pose[1]<<endl;
    cout<<"当前关节角[2]"<<vector_pose[2]<<endl;
    cout<<"当前关节角[3]"<<vector_pose[3]<<endl;
    cout<<"当前关节角[4]"<<vector_pose[4]<<endl;
    cout<<"当前关节角[5]"<<vector_pose[5]<<endl;
    // robot_inverse(SPose,joint_pose);
    if(!robot_inverse(MoveGroupList[robot_num],pose,joint_pose)){
        res.is_success=false;
        return true;
    }
    cout<<"逆解后[0]"<<joint_pose[0]<<endl;
    cout<<"逆解后[1]"<<joint_pose[1]<<endl;
    cout<<"逆解后[2]"<<joint_pose[2]<<endl;
    cout<<"逆解后[3]"<<joint_pose[3]<<endl;
    cout<<"逆解后[4]"<<joint_pose[4]<<endl;
    cout<<"逆解后[5]"<<joint_pose[5]<<endl;

    res.is_success=trajectPreparePtp(joint_pose,MoveGroupList[robot_num], false)==0;
    return true;
}

//目前此接口的角度参数改为了绝对关节角度（弧度），非增量式！！！
bool motion::moveSigleAixsCB(hirop_msgs::moveSigleAixs::Request &req, hirop_msgs::moveSigleAixs::Response &res) {
    int robot_num=-1;
    for (int j = 0; j <MoveGroupList.size(); ++j) {
        if(MoveGroupList[j].groupName==req.moveGroup_name){
            robot_num=j;
        }
    }
    if(robot_num==-1){
        res.is_success= false;
        res.info="此move_group找不到";
        return true;
    }
    //1.获取当前关节角度
    MoveGroupList[robot_num].move_group->setStartStateToCurrentState();
    vector<double> joint_angles = MoveGroupList[robot_num].move_group->getCurrentJointValues();
    //2.对某一轴进行旋转
    uint16_t index_axis = req.index_axis;
    joint_angles[index_axis] = req.angle;
    // joint_angles[index_axis]+=(req.angle/180)*M_PI;

    //执行运动
    res.is_success=trajectPreparePtp(joint_angles,MoveGroupList[robot_num], false)==0;
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

//        vector<double> p1 = vector<double>({-0.9,-0.86, 2.19,0,1.87,-0.9});
//        vector<double> p2 = vector<double>({0,-1.5,3.0,0,1.57,3.14});
//        vector<double> p3 = vector<double>({0,-1.57,3.14,0,1.57,0});
//        vector<vector<double>> pp;
//        pp.push_back(p1);pp.push_back(p2);pp.push_back(p3);
//
//        ros::ServiceClient client=Node->serviceClient<hirop_msgs::moveToMultiPose>("moveToMultiPose");
//        hirop_msgs::moveToMultiPose srv;
//        srv.request.poseList_joints_angle.resize(3);
//        for (int j = 0; j <3; ++j) {
//            srv.request.poseList_joints_angle[j].joints_angle.data.resize(6);
//            for(int i=0;i<6;i++){
//                srv.request.poseList_joints_angle[j].joints_angle.data[i]=pp[j][i];
//            }
//        }
//        if(client.call(srv)){
//            cout<<"服务调用完毕"<<endl;
//        } else{
//            cout<<"服务连接失败"<<endl;
//        }

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

    vector<double> p1 = vector<double>({-0.9,-0.86, 2.19,0,1.87,-0.9});

    vector<double> p2 = vector<double>({0,-1.5,3.0,0,1.57,3.14});

    vector<double> p3 = vector<double>({0,-1.57,3.14,0,1.57,0});

    vector<vector<double>> pp;
    pp.push_back(p1);pp.push_back(p2);pp.push_back(p3);


    trajectory_msgs::JointTrajectory createTraj0,createTraj1;

//    for(int i = 0; i< 100; i++){
    // move_group0
    trajectPrepareMulti(pp, *MoveGroupList[0].move_group,createTraj0 );

    //move_group1
    trajectPrepareMulti(pp, *MoveGroupList[1].move_group,createTraj1 );
//    }
//    pub_robLeft.publish(createTraj0);
//    pub_robRight.publish(createTraj1);
    ros::ServiceClient client=Node->serviceClient<hirop_msgs::dualRbtraject>("motion_bridge/dualRobMotion_JointTraject");
    hirop_msgs::dualRbtraject srv;
    srv.request.robotMotionTraject_list.resize(2);
    srv.request.robotMotionTraject_list[0].moveGroup_name="arm0";
    srv.request.robotMotionTraject_list[0].robot_jointTra=createTraj0;
    srv.request.robotMotionTraject_list[1].moveGroup_name="arm1";
    srv.request.robotMotionTraject_list[1].robot_jointTra=createTraj1;
    if(client.call(srv)){
        if(srv.response.is_success){
         cout<<"service call over"<<endl;
        }
    }else{
        cout<<"service connect faile"<<endl;
    }



    return true;
}

void motion::callback_rob0Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status) {
    MoveGroupList[0].motion_val=robot_status->in_motion.val;
    if(robot_status->in_motion.val==1){
        ass_flag_leftRbMotion=true;
    }
//    cout<<MoveGroupList[0].motion<<endl;
}

void motion::callback_rob1Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status) {
    MoveGroupList[1].motion_val=robot_status->in_motion.val;
    if(robot_status->in_motion.val==1){
        ass_flag_rightRbMotion=true;
//        cout<<"listen motion==1"<<endl;
    }
//    cout<<MoveGroupList[1].motion<<endl;
}

int motion::trajectPrepareMulti( const vector<vector<double>> & end ,
                         moveit::planning_interface::MoveGroupInterface & ptr,
                         trajectory_msgs::JointTrajectory& createTraj ){
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

void motion::setStartState(moveit::planning_interface::MoveGroupInterface & ptr)
{
    ptr.setStartStateToCurrentState();
    robot_state::RobotState r(*ptr.getCurrentState());
    const robot_state::JointModelGroup* jointGroup = r.getJointModelGroup(ptr.getName());
    std::vector<double> joints = ptr.getCurrentJointValues();
    r.setJointGroupPositions(jointGroup, joints);
    ptr.setStartState(r);
}

int motion::createTrajectPlan(moveit_msgs::RobotTrajectory &tempTraject,moveit::planning_interface::MoveGroupInterface & ptr, const string & groupName)
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
    ROS_INFO_STREAM("spline.computeTimeStamps sucess ");
    rt.getRobotTrajectoryMsg(tempTraject);

}
