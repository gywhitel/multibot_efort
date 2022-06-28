#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <path_generate.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "arithmatic.h"
#include "transformation.h"
#include "kinematic.h"
#include "sensor_msgs/JointState.h"
#include <fstream>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "cubicSpline.h"
// #include "color_print.h"

#define WRITE_FILE 1
#define FRONT 1
#define BACK 1

const double PI = 3.14159265358979;
// determine the center of point cloud
// the location of robot 1 and robot 2 w.r.t robot 3
// seems useless
const double y_r1 = 1;  // 0.984 m
const double y_r2 = 1.00;  // 1.003 m
const double x_r1 = 2.094;
const double x_r2 = 2.092;
const double r1_r2_compensation_x = 0.04;     // x_r2 -= r1_r2_compensation

// workpiece pose w.r.t. robot 3 base
const double x_position = 1.0;      // y = 0 
const double height = 1.1;
const double roll = 0, pitch = 0, yaw = M_PI/2;     // ZYX euler angle
const double workpiece_length = 0.44;   // 55~58cm

const double workpiece_position_x = x_position+0.13;
const double workpiece_position_y = 0.045;
const double workpiece_position_z = height+0.01;


using std::vector;
using namespace std;

ofstream create_record(string filename)
{
    ofstream file;
    file.open(string("/home/yinghao/Documents/efort/jointTraj/") + filename);
    file<<"R1J1,R1J2,R1J3,R1J4,R1J5,R1J6,R2J1,R2J2,R2J3,R2J4,R2J5,R2J6,R3J1,R3J2,R3J3,R3J4,R3J5,R3J6\n";
    file.setf(ios::fixed, ios::floatfield);
    file.precision(8);
    return file;
}

ofstream create_record_robot3(string filename)
{
    ofstream file;
    file.open(string("/home/yinghao/Documents/efort/jointTraj/") + filename);
    file<<"R3J1,R3J2,R3J3,R3J4,R3J5,R3J6\n";
    file.setf(ios::fixed, ios::floatfield);
    file.precision(8);
    return file;
}

void record_3robot_trajectory(ofstream& file, RowVector6d joint_robot1, RowVector6d joint_robot2, RowVector6d joint_robot3)
{
    file<<joint_robot1(0)<<','<<joint_robot1(1)<<','<<joint_robot1(2)<<','<<joint_robot1(3)<<','<<joint_robot1(4)<<','<<joint_robot1(5)<<','
    <<joint_robot2(0)<<','<<joint_robot2(1)<<','<<joint_robot2(2)<<','<<joint_robot2(3)<<','<<joint_robot2(4)<<','<<joint_robot2(5)<<','
    <<joint_robot3(0)<<','<<joint_robot3(1)<<','<<joint_robot3(2)<<','<<joint_robot3(3)<<','<<joint_robot3(4)<<','<<joint_robot3(5)<<endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle node_handle;
    ros::Duration(1).sleep();
    ros::Publisher pub = node_handle.advertise<sensor_msgs::JointState>("/joint_states",100);
    sensor_msgs::JointState jointState;
    jointState.name.resize(18);
    jointState.name = {"mid_joint_1", "mid_joint_2", "mid_joint_3", "mid_joint_4", "mid_joint_5", "mid_joint_6",
    "left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6", 
    "right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5", "right_joint_6"
    };
    jointState.position.resize(18);

    const int RATE = 50;
    ros::Rate rate = RATE;

    std::string dataPath = argv[1];
    kinematic robot;
    tf2::Quaternion q;
    // q.setEuler(0, M_PI, M_PI/2);  // YXZ order for curve_steel
    q.setEuler(0, 0, M_PI/2);  // YXZ order for self-developed point cloud
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped cloud_location;
    cloud_location.header.frame_id = "mid_base_link";
    cloud_location.child_frame_id = "pointcloud";
    cloud_location.transform.translation.x = 2.1 - 0.2;
    cloud_location.transform.translation.y = 0;
    cloud_location.transform.translation.z = height;
    cloud_location.transform.rotation.w = q.w();
    cloud_location.transform.rotation.x = q.x();
    cloud_location.transform.rotation.y = q.y();
    cloud_location.transform.rotation.z = q.z();

    RowVector6d pose1, pose2, joint_robot1, joint_robot2, joint_robot3, initialPosition;
    initialPosition<<0,0,0,0,0,0;

    robot.setJointPosition(initialPosition, ER20_1);
    robot.setJointPosition(initialPosition, ER20_2);
    robot.setJointPosition(initialPosition, ER20_3);

//  ----- PHASE 1 Moving to gripping position ------
    const double distance_two_robot = workpiece_length/2;
    pose1 <<0.2, -(y_r1-distance_two_robot), height, PI/2, PI/4, 0;  // pose of robot 1;
    // pose2 <<0.2-r1_r2_compensation_x, y_r2-distance_two_robot-0.1, height, 0, PI/2, PI/2;  // pose of robot 2;
    pose2 <<0.2, y_r2-distance_two_robot, height, 0, PI/2, PI/2;  // pose of robot 2 for simulation;
    joint_robot1 = robot.IK_pose(pose1, ER20_1);
    joint_robot2 = robot.IK_pose(pose2, ER20_2);
    MatrixXd joint_position_robot1 = robot.trajectory_plan_6axes(initialPosition, joint_robot1, 10, 10*RATE);
    MatrixXd joint_position_robot2 = robot.trajectory_plan_6axes(initialPosition, joint_robot2, 10, 10*RATE);
    int i, j, step = joint_position_robot1.rows();
    ros::Duration(5).sleep();
#if WRITE_FILE
    ofstream move_gripping_pose = create_record("dualarm_move2grip_pose.csv");
    // move_gripping_pose.open("/home/yinghao/Documents/efort/data/dualarm_move2grip_pose.csv", ios::out);
    // move_gripping_pose.setf(ios::fixed, ios::floatfield);
    // move_gripping_pose.precision(8);
    // move_gripping_pose<<"R1J1,R1J2,R1J3,R1J4,R1J5,R1J6,R2J1,R2J2,R2J3,R2J4,R2J5,R2J6,R3J1,R3J2,R3J3,R3J4,R3J5,R3J6\n";
//     // move_gripping_pose<<joint_robot1(0)<<','<<joint_robot1(1)<<','<<joint_robot1(2)<<','<<joint_robot1(3)<<','<<joint_robot1(4)<<','<<joint_robot1(5)<<','<<joint_robot2(0)<<','<<joint_robot2(1)<<','<<joint_robot2(2)<<','<<joint_robot2(3)<<','<<joint_robot2(4)<<','<<joint_robot2(5)<<','<<0<<','<<0<<','<<0<<','<<0<<','<<0<<','<<0<<endl;
//     // move_gripping_pose.close();
#endif
    for (i = 0; i < step; i++)
    {
        jointState.header.stamp = ros::Time::now();
        jointState.position = {0,0,0,0,0,0, 
                joint_position_robot1(i,0),joint_position_robot1(i,1),joint_position_robot1(i,2),joint_position_robot1(i,3),joint_position_robot1(i,4),joint_position_robot1(i,5),                
                joint_position_robot2(i,0),joint_position_robot2(i,1),joint_position_robot2(i,2),joint_position_robot2(i,3),joint_position_robot2(i,4),joint_position_robot2(i,5)};

        pub.publish(jointState);
        cloud_location.header.stamp = ros::Time::now();
        broadcaster.sendTransform(cloud_location);
        rate.sleep();
    #if WRITE_FILE
        move_gripping_pose<<joint_position_robot1(i,0)<<','<<joint_position_robot1(i,1)<<','<<joint_position_robot1(i,2)<<','<<joint_position_robot1(i,3)<<','<<joint_position_robot1(i,4)<<','<<joint_position_robot1(i,5)<<','<<                
                joint_position_robot2(i,0)<<','<<joint_position_robot2(i,1)<<','<<joint_position_robot2(i,2)<<','<<joint_position_robot2(i,3)<<','<<joint_position_robot2(i,4)<<','<<joint_position_robot2(i,5)<<','<<0<<','<<0<<','<<0<<','<<0<<','<<0<<','<<0<<endl;
    #endif
    }
    robot.setJointPosition(joint_position_robot1.row(step-1), ER20_1);
    robot.setJointPosition(joint_position_robot2.row(step-1), ER20_2);
    ros::Duration(0.5).sleep();

#if WRITE_FILE
    move_gripping_pose.close();
    ofstream move_polishing_pose = create_record("dualarm_move_forward.csv");
    // move_polishing_pose.open("/home/yinghao/Documents/efort/data/joint_angle_mv2.csv", ios::out);
    // move_polishing_pose<<"R1J1,R1J2,R1J3,R1J4,R1J5,R1J6,R2J1,R2J2,R2J3,R2J4,R2J5,R2J6,R3J1,R3J2,R3J3,R3J4,R3J5,R3J6\n";
    // move_polishing_pose.setf(ios::fixed, ios::floatfield);
    // move_polishing_pose.precision(8);
#endif

    // PHASE 2 move the workpiece linearly in X axis to polishing position
    for (double dx = 0.2; dx <= 2.1 - x_position; dx += 0.001)
    {
        cloud_location.header.stamp = ros::Time::now();
        cloud_location.transform.translation.x = 2.1 - dx;
        q.setRPY((dx - 0.2) / (2.1 - x_position - 0.2) * roll, pitch, yaw);
        cloud_location.transform.rotation.w = q.w();
        cloud_location.transform.rotation.x = q.x();
        cloud_location.transform.rotation.y = q.y();
        cloud_location.transform.rotation.z = q.z();

        pose1(0) = dx;
        pose2(0) = dx;
        // pose2(0) = dx - r1_r2_compensation_x;
        // TODO add rotation about the Z axis of end-effector
        joint_robot1 = robot.IK_pose(pose1, ER20_1);
        joint_robot2 = robot.IK_pose(pose2, ER20_2);
        jointState.header.stamp = ros::Time::now();

        jointState.position = {0,0,0,0,0,0, 
                joint_robot1(0),joint_robot1(1),joint_robot1(2),joint_robot1(3),joint_robot1(4),joint_robot1(5),                
                joint_robot2(0),joint_robot2(1),joint_robot2(2),joint_robot2(3),joint_robot2(4),joint_robot2(5)};
        
        robot.setJointPosition(joint_robot1, ER20_1);
        robot.setJointPosition(joint_robot2, ER20_2);
        pub.publish(jointState);
        broadcaster.sendTransform(cloud_location);
        rate.sleep();

#if WRITE_FILE
        move_polishing_pose<<joint_robot1(0)<<','<<joint_robot1(1)<<','<<joint_robot1(2)<<','<<joint_robot1(3)<<','<<joint_robot1(4)<<','<<joint_robot1(5)<<','
        <<joint_robot2(0)<<','<<joint_robot2(1)<<','<<joint_robot2(2)<<','<<joint_robot2(3)<<','<<joint_robot2(4)<<','<<joint_robot2(5)<<','
        <<0<<','<<0<<','<<0<<','<<0<<','<<0<<','<<0<<endl;
#endif
    }
#if WRITE_FILE
    move_polishing_pose.close();
#endif
// -------------------- Ready to polish ----------------->
    

// <------------------- Polishing path planning -----------------------
    rviz_visual_tools::RvizVisualTools visual_tool("mid_base_link", "/rviz_marker");

    std::vector<pointVec> paths, connection;
    generate_path_from_pointcloud(dataPath, paths, connection);
    std::vector<pointVec> paths_up = paths, connection_up = connection;
    double distance = paths[paths.size()-1].back().x - paths[0][0].x;
    // transformation from workpiece frame to 3rd robot base frame
    Eigen::Isometry3d transformation = Eigen::Isometry3d::Identity();
    // DEBUG x,y offset. Workpiece center w.r.t robot3 

    // transformation.translate(Eigen::Vector3d(x_position+0.13, 0.045, height+0.01));
    transformation.translate(Eigen::Vector3d(workpiece_position_x, workpiece_position_y, workpiece_position_z));
    // transformation.rotate(EulerAngle_to_Quaternion(PI,0,PI/2));   // orientation of curve_steel
    transformation.rotate(EulerAngle_to_Quaternion(roll,pitch,yaw));   // orientation of curve_steel
    // path visualization
    transformPointVec(paths, transformation);
    transformPointVec(connection, transformation);
    cloud_location.transform.translation.x = x_position;
    cloud_location.transform.translation.z = height;
    cloud_location.header.stamp = ros::Time::now();
    broadcaster.sendTransform(cloud_location);
    vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose pose;
    for (i = 0; i < paths.size(); i++)
    {
        for (j = 0; j < paths[i].size(); j++)
        {
            pose = PointNormal_to_Pose(paths[i][j]);
            waypoints.push_back(pose);
        }
        if (i == paths.size() - 1)
            break;
        for (auto iter = connection[i].begin(); iter != connection[i].end(); iter++)
        {
            pose = PointNormal_to_Pose(*iter);
            waypoints.push_back(pose);
        }
    }   
    namespace rvt = rviz_visual_tools;
    visual_tool.publishPath(waypoints, rvt::LIME_GREEN, rvt::XSMALL);
    // visual_tool.prompt("press to publish markers");
    visual_tool.trigger();
    // geometry_msgs::Pose normal_vector;
    for (i = 0; i < waypoints.size(); i+=50)
    {
        // normal_vector = waypoints[i];
        // inverse before transformation
        // cout<<"Original orientation: "<<waypoints[i].orientation;
        // normal_vector.orientation.x = -normal_vector.orientation.x;
        // normal_vector.orientation.y = -normal_vector.orientation.y;
        // normal_vector.orientation.z = -normal_vector.orientation.z;
        // cout<<" normal vector orientation: "<<normal_vector.orientation<<endl;
        
        visual_tool.publishZArrow(waypoints[i], rvt::BLUE, rvt::XXXXSMALL, 0.02);
    }
    visual_tool.trigger();
// PHASE 3 robot 3 end-effector moves to polishing start position
    geometry_msgs::Pose start = waypoints[0];
    // PROBLEM boundary singularity
    start.position.x -= 0.1;
    start.position.z += 0.2;
    // start.position.y -= distance/2;
    joint_robot3 = robot.IK_pose(start, ER20_3);
    // std::cout<<"Forward kinematics:\n"<<robot.fk(joint_robot3, ER20)<<std::endl;
    // MatrixXd joint_position_robot3 = robot.trajectory_plan_6axes(initialPosition, joint_robot3, 10, 100);

    // cloud_location.header.stamp = ros::Time::now();
    // broadcaster.sendTransform(cloud_location);

    // for (i = 0; i < joint_position_robot3.rows(); i++)
    // {
    //     jointState.header.stamp = ros::Time::now();
    //     jointState.position = {joint_position_robot3(i,0), joint_position_robot3(i,1), joint_position_robot3(i,2), joint_position_robot3(i,3), joint_position_robot3(i,4), joint_position_robot3(i,5),
    //     joint_robot1(0),joint_robot1(1),joint_robot1(2),joint_robot1(3),joint_robot1(4),joint_robot1(5),  joint_robot2(0),joint_robot2(1),joint_robot2(2),joint_robot2(3),joint_robot2(4),joint_robot2(5)
    //     };
    //     cloud_location.header.stamp = ros::Time::now();
    //     broadcaster.sendTransform(cloud_location);
    //     pub.publish(jointState);
    //     rate.sleep();
    // }

    jointTrajectory via;
    via.jointPosition = joint_robot3;
    via.time = 5;
    vector<jointTrajectory> viapoints;
    viapoints.push_back(via);
    RowVector6d joint_robot3_2 = robot.IK_pose(waypoints[0], ER20_3);
    // joint_position_robot3 = robot.trajectory_plan_6axes(joint_robot3, joint_robot3_2 ,10, 100);
    auto joint_position_robot3_cubic = cubicSplinePlan(RowVector6d::Zero(), joint_robot3_2, viapoints, 500, 10);
    ofstream before_polishing = create_record_robot3("robot3move2polish.csv");
// #if FRONT
    for (i = 0; i < joint_position_robot3_cubic.size(); i++)
    {
        jointState.header.stamp = ros::Time::now();
        jointState.position = {joint_position_robot3_cubic[i](0), joint_position_robot3_cubic[i](1), joint_position_robot3_cubic[i](2), joint_position_robot3_cubic[i](3), joint_position_robot3_cubic[i](4), joint_position_robot3_cubic[i](5),
        joint_robot1(0),joint_robot1(1),joint_robot1(2),joint_robot1(3),joint_robot1(4),joint_robot1(5),  joint_robot2(0),joint_robot2(1),joint_robot2(2),joint_robot2(3),joint_robot2(4),joint_robot2(5)
        };
        cloud_location.header.stamp = ros::Time::now();
        broadcaster.sendTransform(cloud_location);
        pub.publish(jointState);
        rate.sleep();
        before_polishing<<joint_position_robot3_cubic[i](0)<<','<< joint_position_robot3_cubic[i](1)<<','<<joint_position_robot3_cubic[i](2)<<','<< joint_position_robot3_cubic[i](3)<<','<< joint_position_robot3_cubic[i](4)<<','<< joint_position_robot3_cubic[i](5)<<endl;
    }
/*
#if WRITE_FILE
    ofstream before_polishing;
    before_polishing.open("/home/yinghao/Documents/efort/data/joint_angle_mv3.csv", ios::out);
    before_polishing<<"R1J1,R1J2,R1J3,R1J4,R1J5,R1J6,R2J1,R2J2,R2J3,R2J4,R2J5,R2J6,R3J1,R3J2,R3J3,R3J4,R3J5,R3J6\n";
    before_polishing.setf(ios::fixed, ios::floatfield);
    before_polishing.precision(8);
    before_polishing<<joint_robot1(0)<<','<<joint_robot1(1)<<','<<joint_robot1(2)<<','<<joint_robot1(3)<<','<<joint_robot1(4)<<','<<joint_robot1(5)<<','<<  joint_robot2(0)<<','<<joint_robot2(1)<<','<<joint_robot2(2)<<','<<joint_robot2(3)<<','<<joint_robot2(4)<<','<<joint_robot2(5)<<','<<joint_robot3(0)<<','<<joint_robot3(1)<<','<<joint_robot3(2)<<','<<joint_robot3(3)<<','<<joint_robot3(4)<<','<<joint_robot3(5)<<endl;
    before_polishing<<joint_robot1(0)<<','<<joint_robot1(1)<<','<<joint_robot1(2)<<','<<joint_robot1(3)<<','<<joint_robot1(4)<<','<<joint_robot1(5)<<','<<  joint_robot2(0)<<','<<joint_robot2(1)<<','<<joint_robot2(2)<<','<<joint_robot2(3)<<','<<joint_robot2(4)<<','<<joint_robot2(5)<<','<<joint_robot3_2(0)<<','<<joint_robot3_2(1)<<','<<joint_robot3_2(2)<<','<<joint_robot3_2(3)<<','<<joint_robot3_2(4)<<','<<joint_robot3_2(5)<<endl;
    before_polishing.close();

    ofstream polishing;
    polishing.open("/home/yinghao/Documents/efort/data/joint_angle_mv4.csv", ios::out);
    polishing<<"R1J1,R1J2,R1J3,R1J4,R1J5,R1J6,R2J1,R2J2,R2J3,R2J4,R2J5,R2J6,R3J1,R3J2,R3J3,R3J4,R3J5,R3J6\n";
    polishing.setf(ios::fixed, ios::floatfield);
    polishing.precision(8);
#endif
*/
#if WRITE_FILE
    before_polishing.close();
    ofstream polishing = create_record_robot3("robot3front_polishing.csv");
#endif
    ROS_INFO("Ready to start polishing.");
    // PHASE 4 Polishing
    for (i = 0; i < waypoints.size(); i+=1)
    {
        // waypoints[i].position.y -= djoint_trajistance/2;  // point cloud origin to path start
        joint_robot3 = robot.IK_pose(waypoints[i], ER20_3);
        jointState.header.stamp = ros::Time::now();
        jointState.position = {joint_robot3(0), joint_robot3(1), joint_robot3(2), joint_robot3(3), joint_robot3(4), joint_robot3(5),
        joint_robot1(0),joint_robot1(1),joint_robot1(2),joint_robot1(3),joint_robot1(4),joint_robot1(5),  joint_robot2(0),joint_robot2(1),joint_robot2(2),joint_robot2(3),joint_robot2(4),joint_robot2(5)
        };
        cloud_location.header.stamp = ros::Time::now();
        broadcaster.sendTransform(cloud_location);
        pub.publish(jointState);
        rate.sleep();
#if WRITE_FILE
        // polishing<<joint_robot1(0)<<','<<joint_robot1(1)<<','<<joint_robot1(2)<<','<<joint_robot1(3)<<','<<joint_robot1(4)<<','<<joint_robot1(5)<<',' <<joint_robot2(0)<<','<<joint_robot2(1)<<','<<joint_robot2(2)<<','<<joint_robot2(3)<<','<<joint_robot2(4)<<','<<joint_robot2(5)<<','<<joint_robot3(0)<<','<<joint_robot3(1)<<','<<joint_robot3(2)<<','<<joint_robot3(3)<<','<<joint_robot3(4)<<','<<joint_robot3(5)<<endl;
        polishing<<joint_robot3(0)<<','<<joint_robot3(1)<<','<<joint_robot3(2)<<','<<joint_robot3(3)<<','<<joint_robot3(4)<<','<<joint_robot3(5)<<endl;
#endif
    }
#if WRITE_FILE
    polishing.close();
    ofstream withdraw = create_record_robot3("robot3after_front_polishing.csv");
#endif
    // detach the end-effector from the workpiece
    auto end = waypoints.back();
    end.position.z += 0.1;
    end.position.x -= 0.2;
    joint_robot3_2 = robot.IK_pose(end, ER20_3);
    MatrixXd joint_position_robot3 = robot.trajectory_plan_6axes(joint_robot3, joint_robot3_2, 10, 100);
    for (i = 0; i < joint_position_robot3.rows(); i+=10)
    {
        jointState.header.stamp = ros::Time::now();
        jointState.position = {joint_position_robot3(i,0), joint_position_robot3(i,1), joint_position_robot3(i,2), joint_position_robot3(i,3), joint_position_robot3(i,4), joint_position_robot3(i,5),
        joint_robot1(0),joint_robot1(1),joint_robot1(2),joint_robot1(3),joint_robot1(4),joint_robot1(5),  joint_robot2(0),joint_robot2(1),joint_robot2(2),joint_robot2(3),joint_robot2(4),joint_robot2(5)
        };
        // cloud_location.header.stamp = ros::Time::now();
        // broadcaster.sendTransform(cloud_location);
        pub.publish(jointState);
        withdraw<<joint_position_robot3(i,0)<<','<< joint_position_robot3(i,1)<<','<< joint_position_robot3(i,2)<<','<< joint_position_robot3(i,3)<<','<< joint_position_robot3(i,4)<<','<< joint_position_robot3(i,5)<<endl;
        rate.sleep(); 
    }

// #if WRITE_FILE
//     ofstream homing;
//     homing.open("/home/yinghao/Documents/efort/data/joint_angle_mv5.csv", ios::out);
//     homing<<"R1J1,R1J2,R1J3,R1J4,R1J5,R1J6,R2J1,R2J2,R2J3,R2J4,R2J5,R2J6,R3J1,R3J2,R3J3,R3J4,R3J5,R3J6\n";
//     homing.setf(ios::fixed, ios::floatfield);
//     homing.precision(8);
//     homing<<joint_robot1(0)<<','<<joint_robot1(1)<<','<<joint_robot1(2)<<','<<joint_robot1(3)<<','<<joint_robot1(4)<<','<<joint_robot1(5)<<','
//         <<joint_robot2(0)<<','<<joint_robot2(1)<<','<<joint_robot2(2)<<','<<joint_robot2(3)<<','<<joint_robot2(4)<<','<<joint_robot2(5)<<','
//         <<joint_robot3(0)<<','<<joint_robot3(1)<<','<<joint_robot3(2)<<','<<joint_robot3(3)<<','<<joint_robot3(4)<<','<<joint_robot3(5)<<endl;
//     homing.close();
// #endif
//     cloud_location.header.stamp = ros::Time::now();
//     broadcaster.sendTransform(cloud_location);
// #endif
#if WRITE_FILE
    withdraw.close();
    ofstream flipTraj = create_record("dualarm_flip.csv");
#endif

#if BACK
// ----------- PHASE 5 FLIP ----------------
// <----------------- rotate workpiece -------------
    for (i = 1; i < 51; i++)
    {
        cloud_location.header.stamp = ros::Time::now();
        q.setEuler(M_PI*i/double(49), 0, M_PI/2);
        cloud_location.transform.rotation.w = q.w();
        cloud_location.transform.rotation.x = q.x();
        cloud_location.transform.rotation.y = q.y();
        cloud_location.transform.rotation.z = q.z();
        broadcaster.sendTransform(cloud_location);

        jointState.header.stamp = ros::Time::now();
        jointState.position = {joint_robot3_2(0), joint_robot3_2(1), joint_robot3_2(2), joint_robot3_2(3), joint_robot3_2(4), joint_robot3_2(5),
        joint_robot1(0),joint_robot1(1),joint_robot1(2),joint_robot1(3),joint_robot1(4),joint_robot1(5) - i*M_PI/double(50),  joint_robot2(0),joint_robot2(1),joint_robot2(2),joint_robot2(3),joint_robot2(4),joint_robot2(5) + i*M_PI/double(50)};
        pub.publish(jointState);
        rate.sleep();
        flipTraj<<joint_robot1(0)<<','<<joint_robot1(1)<<','<<joint_robot1(2)<<','<<joint_robot1(3)<<','<<joint_robot1(4)<<','<<joint_robot1(5) - i*M_PI/double(50)<<','<<  joint_robot2(0)<<','<<joint_robot2(1)<<','<<joint_robot2(2)<<','<<joint_robot2(3)<<','<<joint_robot2(4)<<','<<joint_robot2(5) + i*M_PI/double(50)<<endl;
    }

    Eigen::Isometry3d flip = Eigen::Isometry3d::Identity();
    flip.translate(Eigen::Vector3d(workpiece_position_x, workpiece_position_y, workpiece_position_z-0.07));
    flip.rotate(EulerAngle_to_Quaternion(M_PI,0,M_PI_2));
    // flip.translate(Eigen::Vector3d(workpiece_position_x, workpiece_position_y, workpiece_position_z));
    transformPointVec(paths_up, flip);
    transformPointVec(connection_up, flip);
    flip = Isometry3d::Identity();
    vector<geometry_msgs::Pose> waypoints_back;
    
    for (i = paths_up.size()-1; i >= 0; i--)
    {
        for (j = paths_up[i].size()-1; j >= 0; j--)
        {
            paths_up[i][j].normal_x *= -1;
            paths_up[i][j].normal_y *= -1;
            paths_up[i][j].normal_z *= -1;
            pose = PointNormal_to_Pose(paths_up[i][j]);
            waypoints_back.push_back(pose);
        }
        if (i == 0) break;
        for (auto riter = connection_up[i-1].rbegin(); riter != connection_up[i-1].rend(); riter++)
        {
            riter->normal_x *= -1;
            riter->normal_y *= -1;
            riter->normal_z *= -1;
            pose = PointNormal_to_Pose(*riter);
            waypoints_back.push_back(pose);
        }
    }
    visual_tool.publishPath(waypoints_back, rvt::CYAN, rvt::XSMALL);
    for (i = 0; i < waypoints_back.size(); i+=50)
    {
        visual_tool.publishZArrow(waypoints_back[i],rvt::ORANGE, rvt::XXXXSMALL, 0.02);
    }
    visual_tool.trigger();
    start = waypoints_back[0];
    start.position.x -= 0.1;
    start.position.z += 0.1;
    initialPosition = joint_robot3_2;
    joint_robot3 = robot.IK_pose(start, ER20_3);
    // joint_position_robot3 = robot.trajectory_plan_6axes(joint_robot3_2, joint_robot3 ,10, 100);
    joint_robot3_2 = robot.IK_pose(waypoints_back[0], ER20_3);
    jointTrajectory via2;
    via2.jointPosition = joint_robot3;
    via2.time = 5;
    viapoints.clear();
    viapoints.push_back(via2);
    auto robot3move2polish = cubicSplinePlan(initialPosition, joint_robot3_2, viapoints,500, 10);

    // for (i = 0; i < joint_position_robot3.rows(); i++)
    // {
    //     jointState.header.stamp = ros::Time::now();
    //     jointState.position = {joint_position_robot3(i,0), joint_position_robot3(i,1), joint_position_robot3(i,2), joint_position_robot3(i,3), joint_position_robot3(i,4), joint_position_robot3(i,5),
    //     joint_robot1(0),joint_robot1(1),joint_robot1(2),joint_robot1(3),joint_robot1(4),joint_robot1(5),  joint_robot2(0),joint_robot2(1),joint_robot2(2),joint_robot2(3),joint_robot2(4),joint_robot2(5)
    //     };
    //     cloud_location.header.stamp = ros::Time::now();
    //     broadcaster.sendTransform(cloud_location);
    //     pub.publish(jointState);
    //     after_flip<<joint_position_robot3(i,0)<<','<<joint_position_robot3(i,1)<<','<< joint_position_robot3(i,2)<<','<< joint_position_robot3(i,3)<<','<< joint_position_robot3(i,4)<<','<< joint_position_robot3(i,5)<<','<<endl;
    //     rate.sleep();
    // }
#if WRITE_FILE
    flipTraj.close();
    ofstream before_back_polishing = create_record_robot3("robot3move2polish_back.csv");
#endif
    // joint_position_robot3 = robot.trajectory_plan_6axes(joint_robot3, joint_robot3_2 ,10, 100);
    for (i = 0; i < robot3move2polish.size(); i++)
    {
        jointState.header.stamp = ros::Time::now();
        jointState.position = {robot3move2polish[i](0), robot3move2polish[i](1), robot3move2polish[i](2), robot3move2polish[i](3), robot3move2polish[i](4), robot3move2polish[i](5),
        joint_robot1(0),joint_robot1(1),joint_robot1(2),joint_robot1(3),joint_robot1(4),joint_robot1(5),  joint_robot2(0),joint_robot2(1),joint_robot2(2),joint_robot2(3),joint_robot2(4),joint_robot2(5)
        };
        cloud_location.header.stamp = ros::Time::now();
        broadcaster.sendTransform(cloud_location);
        pub.publish(jointState);
        before_back_polishing<<robot3move2polish[i](0)<<','<<robot3move2polish[i](1)<<','<< robot3move2polish[i](2)<<','<< robot3move2polish[i](3)<<','<<robot3move2polish[i](4)<<','<<robot3move2polish[i](5)<<','<<endl;
        rate.sleep();
    }

    // ------------- PHASE 6 Polish back side --------------
#if WRITE_FILE
    before_back_polishing.close();
    // record_3robot_trajectory(after_flip, joint_robot1, joint_robot2, joint_robot3_2);
    ofstream back_polishing = create_record_robot3("robot3polish_back.csv");
#endif

    for (i = 0; i < waypoints_back.size(); i+=1)
    {
        // waypoints[i].position.y -= distance/2;  // point cloud origin to path start
        joint_robot3 = robot.IK_pose(waypoints_back[i], ER20_3);
        jointState.header.stamp = ros::Time::now();
        jointState.position = {joint_robot3(0), joint_robot3(1), joint_robot3(2), joint_robot3(3), joint_robot3(4), joint_robot3(5),
        joint_robot1(0),joint_robot1(1),joint_robot1(2),joint_robot1(3),joint_robot1(4),joint_robot1(5),  joint_robot2(0),joint_robot2(1),joint_robot2(2),joint_robot2(3),joint_robot2(4),joint_robot2(5)
        };
#if WRITE_FILE
        // record_3robot_trajectory(back_polishing, joint_robot1, joint_robot2, joint_robot3);
        back_polishing<<joint_robot3(0)<<','<<joint_robot3(1)<<','<< joint_robot3(2)<<','<< joint_robot3(3)<<','<< joint_robot3(4)<<','<< joint_robot3(5)<<endl;
#endif
        cloud_location.header.stamp = ros::Time::now();
        broadcaster.sendTransform(cloud_location);
        pub.publish(jointState);
        rate.sleep();
    }
    end = waypoints_back.back();
    end.position.x -= 0.1;
    end.position.y += 0.1;
    end.position.z += 0.2;
    joint_robot3_2 = robot.IK_pose(end, ER20_3);
#if WRITE_FILE
    back_polishing.close();
    ofstream finish = create_record_robot3("robot3finish.csv");
#endif
    joint_position_robot3 = robot.trajectory_plan_6axes(joint_robot3, joint_robot3_2, 10, 100);
    for (i = 0; i < joint_position_robot3.rows(); i++)
    {
        jointState.header.stamp = ros::Time::now();
        jointState.position = {joint_position_robot3(i,0), joint_position_robot3(i,1), joint_position_robot3(i,2), joint_position_robot3(i,3), joint_position_robot3(i,4), joint_position_robot3(i,5),
        joint_robot1(0),joint_robot1(1),joint_robot1(2),joint_robot1(3),joint_robot1(4),joint_robot1(5),  joint_robot2(0),joint_robot2(1),joint_robot2(2),joint_robot2(3),joint_robot2(4),joint_robot2(5)
        };
        cloud_location.header.stamp = ros::Time::now();
        broadcaster.sendTransform(cloud_location);
        pub.publish(jointState);
        finish<<joint_position_robot3(i,0)<<','<<joint_position_robot3(i,1)<<','<< joint_position_robot3(i,2)<<','<< joint_position_robot3(i,3)<<','<< joint_position_robot3(i,4)<<','<< joint_position_robot3(i,5)<<endl;
        rate.sleep();
    }
#endif
#if WRITE_FILE
    finish.close();
#endif
// ------------------ rotate workpiece ---------->
    return 0;
}