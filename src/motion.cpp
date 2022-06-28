#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include "kinematic.h"
#include <eigen3/Eigen/Core>

#define viapoint 0

using std::vector;
using std::string;
using namespace std;

const double PI = 3.14159265358979;
const int RATE = 20;
const double distance_two_robot = 0.74;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "position_commander");
    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<sensor_msgs::JointState>("/joint_states", 1000);
    ros::Rate loop_rate(RATE);
    
    // kinematic* robot = new kinematic();
    kinematic robot;
    // ER20_1, ER20_2, ER10
    RowVector6d pose1, pose2, pose3; // (x, y, z, rx, ry, rz);
    RowVector6d initialJointAngle, joint_robot1, joint_robot2, joint_robot3;
    initialJointAngle<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    robot.setJointPosition(initialJointAngle, ER10);
    robot.setJointPosition(initialJointAngle, ER20_1);
    robot.setJointPosition(initialJointAngle, ER20_2);
    RowVector6d pose;
    Matrix4d T1, T2, temp;
    pose1 <<0.2, -(1-distance_two_robot/2), 1.2, 0, PI/2, -PI/2;  // pose of robot 1;
    pose2 <<0.2, 1-distance_two_robot/2, 1.2, 0, PI/2, PI/2;  // pose of robot 2;
    // pose1 <<1.0, 0, 1.2, 0, PI/2, -PI/2;  // pose of robot 1;
    // pose2 <<1.0, 0, 1.2, 0, PI/2, PI/2;  // pose of robot 2;
    joint_robot1 = robot.IK_pose(pose1, ER20_1);
    joint_robot2 = robot.IK_pose(pose2, ER20_2);
    // PTP motion, moving to initial position
    // move 10 seconds
    MatrixXd joint_position_robot1 = robot.trajectory_plan_6axes(initialJointAngle, joint_robot1, 10, 10*RATE);
    MatrixXd joint_position_robot2 = robot.trajectory_plan_6axes(initialJointAngle, joint_robot2, 10, 10*RATE);
    int step = joint_position_robot1.rows();
    cout<<step<<" steps to go"<<endl;
#if viapoint
    // add a viapoint for robot 1
    pose1<<0, -0.4, 1.2, 0, PI/2, 0;
    RowVector6d viaposition = robot.IK_pose(pose1, ER20_1);
    jointTrajectory viapoint(viaposition, 5);
    vector<jointTrajectory> viapoints;
    viapoints.push_back(viapoint);
    vector<RowVector6d> interpolated_trajectory = cubicSplinePlan(joint_robot1, initialJointAngle, viapoints, 10*RATE, 10);
#endif   
    // cout<<interpolated_trajectory.size();

    sensor_msgs::JointState jointState;
    jointState.name.resize(12);
    jointState.name = {
        // "mid_joint_1", "mid_joint_2", "mid_joint_3", "mid_joint_4", "mid_joint_5", "mid_joint_6",
    "left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6", 
    "right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5", "right_joint_6"};
    jointState.position.resize(12);
    int i;
    while(ros::ok())
    {
        for (i = 0; i < step; i++)
        {
            jointState.header.stamp = ros::Time::now();
            // jointState.position = {0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0};
                    // joint_robot1(0), joint_robot1(1), joint_robot1(2), joint_robot1(3), joint_robot1(4), joint_robot1(5),
                    // joint_robot2(0), joint_robot2(1), joint_robot2(2), joint_robot2(3), joint_robot2(4), joint_robot2(5)
            jointState.position = {
                // 0,0,0,0,0,0, 
                    joint_position_robot1(i,0),joint_position_robot1(i,1),joint_position_robot1(i,2),joint_position_robot1(i,3),joint_position_robot1(i,4),joint_position_robot1(i,5),                
                    joint_position_robot2(i,0),joint_position_robot2(i,1),joint_position_robot2(i,2),joint_position_robot2(i,3),joint_position_robot2(i,4),joint_position_robot2(i,5)};
            pub.publish(jointState);
            loop_rate.sleep();
        }
#if viapoint
        for (i = 0; i < interpolated_trajectory.size(); i++)
        {
            jointState.header.stamp = ros::Time::now();
            jointState.position = {0,0,0,0,0,0, interpolated_trajectory[i](0), interpolated_trajectory[i](1), interpolated_trajectory[i](2), interpolated_trajectory[i](3), interpolated_trajectory[i](4), interpolated_trajectory[i](5), 0,0,0,0,0,0};
            pub.publish(jointState);
            loop_rate.sleep();
        }
#endif
    }
}