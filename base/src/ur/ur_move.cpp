#include "base/ur_move.h"


// void logOperation(chess_msgs::Operation &op)
// {
//     ROS_INFO("type: %d, pick[%d - (%f, %f)] ==>  place[%d - (%f, %f)]", op.type, op.pick_type, op.pick_x, op.pick_y, op.place_type, op.place_x, op.place_y);
// }

void logPose(geometry_msgs::Pose &pose)
{
    ROS_INFO("position(%f, %f, %f), orientation(%f, %f, %f, %f)", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

void logStartEnd(geometry_msgs::Pose &start, geometry_msgs::Pose &end)
{
    ROS_INFO("\t\t\t\tstart(%f, %f, %f) ==> end(%f, %f, %f)", start.position.x, start.position.y, start.position.z, end.position.x, end.position.y, end.position.z);
}

void UrMove::logCurrentPose(std::string str)
{
    geometry_msgs::Pose pose = move_group_.getCurrentPose().pose;
    ROS_INFO("%sposition(%f, %f, %f)", str.c_str(), pose.position.x, pose.position.y, pose.position.z);
}

UrMove::UrMove(std::string group_name) :
    move_group_(group_name)
{
    io_client_ = nh_.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");
    
    // 配置 move_group 参数
    double planning_time,tolerance;
    ros::param::get("~planning_time", planning_time);
    ROS_INFO("planning_time : %f",planning_time);
    ros::param::get("~tolerance", tolerance);
    ros::param::get("~scale", scale);
    ros::param::get("~slow_scale", slow_scale);
    move_group_.setPlanningTime(planning_time);
    move_group_.setGoalTolerance(tolerance);
    move_group_.setMaxVelocityScalingFactor(scale);
    move_group_.setMaxAccelerationScalingFactor(scale*scale);

    // 获取配置参数
    ros::param::get("~sim", sim);
    ros::param::get("~sleep", sleep);

    ros::param::get("~up_height", up_height);
    // 抓取参数
    ros::param::get("~position_z", position_z);
    // ee_step
    ros::param::get("~eef_step", eef_step);
    // 等待机械臂的位姿
    ros::param::get("~wait_p_x", wait_p_x);
    ros::param::get("~wait_p_y", wait_p_y);
    ros::param::get("~wait_p_z", wait_p_z);
    ros::param::get("~wait_o_x", wait_o_x);
    ros::param::get("~wait_o_y", wait_o_y);
    ros::param::get("~wait_o_z", wait_o_z);
    ros::param::get("~wait_o_w", wait_o_w);

    // far
    ros::param::get("~far_o_x", far_o_x);
    ros::param::get("~far_o_y", far_o_y);
    ros::param::get("~far_o_z", far_o_z);
    ros::param::get("~far_o_w", far_o_w);
    // near
    ros::param::get("~near_o_x", near_o_x);
    ros::param::get("~near_o_y", near_o_y);
    ros::param::get("~near_o_z", near_o_z);
    ros::param::get("~near_o_w", near_o_w);
    
}
UrMove::~UrMove()
{
}

void UrMove::updateSpeed(bool slow)
{
    double ss = slow ? slow_scale : scale;
    move_group_.setMaxVelocityScalingFactor(ss);
    move_group_.setMaxAccelerationScalingFactor(ss * ss);
    ROS_INFO("\t\t\tscale :%f ", ss);
}

// bool UrMove::play(chess_msgs::Operation &op)
// {   
//     ROS_WARN(">>>>>>>>>>>>>>> Begin Operation >>>>>>>>>>>>>>>>>>>>>>>");
//     logOperation(op);
    
//     switch(op.type) 
//     {
//         case 0: // 走棋
//             if(!this->go(op)) return false;
//             break;
//         case 1: // 吃子
//             if(!this->capture(op)) return false;
//             break;
//         case 2: // 回等待位
//             if(!this->wait()) return false;
//             break;
//     }

//     ROS_WARN(">>>>>>>>>>>>>>> End Operation. >>>>>>>>>>>>>>>>>>>>>>>");
//     return true;
// }

// // 走棋
// bool UrMove::go(chess_msgs::Operation &op)
// {
//     ROS_INFO("\tbegin go...");

//     // 抓棋
//     if(!this->pick(op)) 
//         return false;

//     // 放到目的位置
//     geometry_msgs::Pose pose;
//     pose.position.x = op.place_x;
//     pose.position.y = op.place_y;
//     pose.position.z = position_z;
//     pose.orientation.x = op.place_type == 0 ? far_o_x : near_o_x;
//     pose.orientation.y = op.place_type == 0 ? far_o_y : near_o_y;
//     pose.orientation.z = op.place_type == 0 ? far_o_z : near_o_z;
//     pose.orientation.w = op.place_type == 0 ? far_o_w : near_o_w;
//     logPose(pose);
//     // 移动到目标位置
//     if(!this->movePath(pose))
//         return false;
//     // 打开转手
//     this->controlGripper(true);
//     // 抬手
//     this->moveUp(pose);
    
//     ROS_INFO("\tend go.");
//     return true;
// } 

// // 吃棋     
// bool UrMove::capture(chess_msgs::Operation &op)
// {
//     ROS_INFO("\tbegin capture...");

//     // 抓棋
//     if(!this->pick(op)) 
//         return false;

//     // 放到等待位置
//     geometry_msgs::Pose pose;
//     pose.position.x = wait_p_x;
//     pose.position.y = wait_p_y;
//     pose.position.z = wait_p_z;
//     pose.orientation.x = wait_o_x;
//     pose.orientation.y = wait_o_y;
//     pose.orientation.z = wait_o_z;
//     pose.orientation.w = wait_o_w;
//     logPose(pose);
//     // 移动
//     if(!this->movePath(pose))
//         return false;
//     // 打开转手
//     this->controlGripper(true);
    
//     ROS_INFO("\tend capture.");
//     return true;
// }

// 回等待位
bool UrMove::wait() 
{
    ROS_INFO("\tbegin wait...");

    // 设置目标位姿
    geometry_msgs::Pose pose;
    pose.position.x = 0.0600368141071;
    pose.position.y = -0.461724470293;
    pose.position.z = 0.11800;
    pose.orientation.x =-0.251979405781;
    pose.orientation.y = 0.658020218454;
    pose.orientation.z = 0.283308220893;
    pose.orientation.w = 0.650578375865;
    // 移动
    if(!this->movePath(pose))
        return false;
    
    ROS_INFO("\tend wait.");
    return true;
}


bool UrMove::pick(geometry_msgs::Pose target_pose)
{       
    ROS_INFO("\tbegin pick...");

    // 1> 打开抓手
    this->controlGripper(true);
    
    // 设置目标位姿
    geometry_msgs::Pose pose;
    pose.position.x = target_pose.position.x;
    pose.position.y = target_pose.position.y;
    pose.position.z = target_pose.position.z;
    pose.orientation.x = target_pose.orientation.x;
    pose.orientation.y = target_pose.orientation.y;
    pose.orientation.z = target_pose.orientation.z;
    pose.orientation.w = target_pose.orientation.w;
    
    // 2> 向下移动到目标位置
    if(!this->movePath(pose))
        return false;
    
    // 3> 关闭抓手
    this->controlGripper(false);

    // 4> 抬手
    this->moveUp(pose);

    ROS_INFO("\tend pick.");

    return true;
}

bool UrMove::controlGripper(bool isOpen)
{
    if(sim) 
        return true;
    
    ROS_INFO("\t\tbegin control gripper...");

    ur_msgs::SetIO ur_io;
    ur_io.request.fun = 1;  // 数字输出
    ur_io.request.pin = 16; // 抓手
    ur_io.request.state = isOpen ? 1 : 0;
    if (io_client_.call(ur_io))
    {
        ROS_INFO("\t\t\tur io call success : %d", ur_io.response.success);
    }
    else
    {
        ROS_INFO("\t\t\tur io call failure!");
        return false;
    }

    ROS_INFO("\t\tend control gripper.");
    ros::WallDuration(sleep).sleep();

    return true;
}

bool UrMove::moveUp(geometry_msgs::Pose &pose)
{
    ROS_INFO("\t\tbegin move up...");
    this->logCurrentPose("\t\t\t");
    geometry_msgs::Pose tempPose = pose;
    tempPose.position.z += up_height;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(tempPose);

    if(!this->moveCartesian(waypoints))
        return false;
    this->logCurrentPose("\t\t\t");
    ROS_INFO("\t\tend move up.");
    return true;
}

bool UrMove::movePath(geometry_msgs::Pose &pose) {
    ROS_INFO("\t\tbegin move Path...");
    
    // 移动XY
    if(!this->moveXY(pose))
        return false;
    // 移动Z
    if(!this->moveZ(pose))
        return false;
        
    // // 获取起点坐标
    // geometry_msgs::Pose startPose = move_group_.getCurrentPose().pose;
    // if(startPose.position.z < position_z) 
    // {
    //     // 移动Z
    //     if(!this->moveZ(pose))
    //         return false;
    //     // 移动XY
    //     if(!this->moveXY(pose))
    //         return false;
    // } 
    // else 
    // {
    //     // 移动XY
    //     if(!this->moveXY(pose))
    //         return false;
    //     // 移动Z
    //     if(!this->moveZ(pose))
    //         return false;
    // }

    ROS_INFO("\t\tend move Path.");
    return true;
}

bool UrMove::moveXY(geometry_msgs::Pose &pose)
{
    eef_step = 0.01;
    ROS_INFO("\t\t\tbegin move XY...");
    this->logCurrentPose("\t\t\t\t");
    // 1> 获取起点坐标
    geometry_msgs::Pose startPose = move_group_.getCurrentPose().pose;
    logStartEnd(startPose, pose);
    double startX = startPose.position.x * 1000;
    double startY = startPose.position.y * 1000;
    double endX = pose.position.x * 1000;
    double endY = pose.position.y * 1000;

    double line = sqrt(abs(endX - startX)*abs(endX - startX) + abs(endY - startY)*abs(endY - startY));
    
    if(line < 0.0000000001)
        return true;

    double step = eef_step * 1000;
    // 2> 计算移动 Line 路点
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose tempPose = startPose;
    int count = (int) line/step;
    ROS_INFO("\t\t\tline :%f step:%f  count:%d", line, step, count);
    // this->updateSpeed(count <= 6);
    double tempX, tempY;
    for(int i= 0; i < count; i++) 
    {
        tempX = startX +  (step/line)*(endX - startX) * i;
        tempY = startY + (step/line)*(endY - startY) * i;
        tempPose.position.x = tempX / 1000;
        tempPose.position.y = tempY / 1000;
        if(i > count/2) { // 改变姿势
            tempPose.orientation = pose.orientation;
        }
        waypoints.push_back(tempPose);
        // std::cout << tempPose.position.x << " " << tempPose.position.y << std::endl;
    }
    // 加上最后的点
    tempPose.position.x = pose.position.x;
    tempPose.position.y = pose.position.y;
    waypoints.push_back(tempPose);

    if(!this->moveCartesian(waypoints))
        return false;
    this->logCurrentPose("\t\t\t\t");
    ROS_INFO("\t\t\tend move XY.");
    return true;
}

bool UrMove::moveZ(geometry_msgs::Pose &pose)
{
    eef_step = 0.01;
    ROS_INFO("\t\t\tbegin move Z...");
    this->logCurrentPose("\t\t\t\t");
    // 1> 获取起点坐标
    geometry_msgs::Pose startPose = move_group_.getCurrentPose().pose;
    double startZ = startPose.position.z * 1000;
    double endZ = pose.position.z * 1000;
    double line = abs(endZ - startZ);

    if(line < 0.0000000001)
        return true;

    double step = eef_step * 1000;
    // 2> 计算移动 Z 路点
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose tempPose = startPose;
    tempPose.orientation = pose.orientation;
    int count = (int) line/step;
    double tempZ;
    for(int i = 0; i < count; i++) 
    {
        tempZ = startZ + step/line * (endZ - startZ) * i;
        tempPose.position.z = tempZ / 1000;
        waypoints.push_back(tempPose);
        // std::cout << tempPose.position.z << std::endl;
    }
    
    // 加上最后的点
    tempPose.position.z = pose.position.z;
    waypoints.push_back(tempPose);

    if(!this->moveCartesian(waypoints))
        return false;
    
    this->logCurrentPose("\t\t\t\t");
    ROS_INFO("\t\t\tend move Z.");
    return true;
}

bool UrMove::move(geometry_msgs::Pose &pose)
{   
    ROS_INFO("\t\t\tbegin move ....");
    move_group_.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    //bool success = move_group_.plan(plan);
    //ROS_INFO("\t\t\t\tmoveit planning %s!", success ? "success" : "faulure");
    if (move_group_.plan(plan)) // 执行
    {
        move_group_.execute(plan);
	    ROS_INFO("\t\t\t\tmoveit planning success!");
    } 

    ROS_INFO("\t\t\tend move.");
    ros::WallDuration(sleep).sleep();

    return true;
}

bool UrMove::moveCartesian(std::vector<geometry_msgs::Pose> waypoints)
{
    ROS_INFO("\t\t\t\tbegin cartesian move...");

    // for(int i = 0; i < waypoints.size(); i++) {
    //     logPose(waypoints[i]);
    // }

    moveit_msgs::RobotTrajectory trajectory;
    int maxtries    = 1;
    double fraction = 0;
    int attempts    = 0;
    const double jump_threshold = 0.0;
    const double eef_step       = 0.01;
    while (fraction < 1 && attempts < maxtries)
    {
        fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts += 1;
        if(attempts % 10 == 0)
        {
            ROS_INFO("\t\t\t\t\tStill trying after %d attempts...", attempts);
        }
        ROS_INFO("\t\t\t\t\tmove plan (%.2f%% acheived)", fraction * 100.0);
    }
    bool success = (fraction == 1);
    ROS_INFO("\t\t\t\t\tCartesian %s.", success ? "success" : "failure");
    if(success) 
    {
        moveit::planning_interface::MoveGroupInterface::Plan pathPlan;
        pathPlan.trajectory_ = trajectory;
        move_group_.execute(pathPlan);
        ROS_INFO("\t\t\t\t\tCartesian execution.");
    }

    ROS_INFO("\t\t\t\tend cartesian end.");
    ros::WallDuration(sleep).sleep();

    return success;
}

bool UrMove::moveConstraints(geometry_msgs::Pose &pose)
{
    ROS_INFO("begin cartesian move ....");
    // 添加关节限制
    moveit_msgs::OrientationConstraint oc;
    oc.link_name = "wrist_3_link";
    oc.header.frame_id = "base_link";
    oc.orientation.x = pose.orientation.x;
    oc.orientation.y = pose.orientation.y;
    oc.orientation.z = pose.orientation.z;
    oc.orientation.w = pose.orientation.w;
    oc.weight = 1.0;
    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(oc);
    move_group_.setPathConstraints(constraints);

    bool success = this->move(pose);

    move_group_.clearPathConstraints();

    ROS_INFO("end cartesian move ....");
    ros::WallDuration(sleep).sleep();
    return success;
}

void UrMove::testMoveZ()
{
    geometry_msgs::Pose pose = move_group_.getCurrentPose().pose;
    // pose.position.x = 0.045;
    // pose.position.y = -0.45;
    pose.position.z = 0.058;
    // if(!this->moveXY(pose))
    //     return;
    if(!this->moveZ(pose))
        return;
    pose.position.z = 0.028;

    if(!this->moveZ(pose))
        return;

    // this->logCurrentPose("\t\t\t");
    // std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(pose);
    // this->moveCartesian(waypoints);
    // this->logCurrentPose("\t\t\t");

}