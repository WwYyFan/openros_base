#ifndef BASE_UR_MOVE_H
#define BASE_UR_MOVE_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <ur_msgs/SetIO.h>
#include <geometry_msgs/Pose.h>

// #include <chess_msgs/Operation.h>

class UrMove 
{
    private:
        bool sim;
        double scale, slow_scale;
        double sleep;
        double up_height, down_height;
        double position_z;
        double eef_step;
        double wait_p_x, wait_p_y, wait_p_z;
        double wait_o_x, wait_o_y, wait_o_z, wait_o_w;
        double far_o_x, far_o_y, far_o_z, far_o_w;
        double near_o_x, near_o_y, near_o_z, near_o_w;
        
        void logCurrentPose(std::string str);
        void updateSpeed(bool slow);    // 更新速度
        
        // bool go(chess_msgs::Operation &op);         // 走棋
        // bool capture(chess_msgs::Operation &op);    // 吃棋
       
       
        
        bool moveUp(geometry_msgs::Pose &pose);
        
        bool moveXY(geometry_msgs::Pose &pose);   
        bool moveZ(geometry_msgs::Pose &pose);

        bool move(geometry_msgs::Pose &pose);
        bool moveCartesian(std::vector<geometry_msgs::Pose> waypoints);
        bool moveConstraints(geometry_msgs::Pose &pose);
        
    protected:
        ros::NodeHandle nh_;
        moveit::planning_interface::MoveGroupInterface move_group_;
        ros::ServiceClient io_client_;
        
    public:
        bool isMoving;
        UrMove(std::string group_name);
        ~UrMove(void);

        
        bool movePath(geometry_msgs::Pose &pose);

        bool wait();                                // 回等待位置


        bool pick(geometry_msgs::Pose target_pose);       // 抓棋
        // bool play(chess_msgs::Operation &op);
        bool controlGripper(bool isOpen);           // 控制抓手
        void testMoveZ();
        
};

#endif
