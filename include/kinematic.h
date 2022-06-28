#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <iostream>  //   /usr/include/eigen3/  Eigen3头文件的默认存放位置
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <string>
#include "cubicSpline.h"
#include <cmath>
#include <math.h>
#include <geometry_msgs/Pose.h>


#define pi 3.14159265358979
using namespace Eigen;
using std::string;

enum robot_type
{
    ER10,
    ER20,
    ER20_1,
    ER20_2,
    ER20_3
};


// NOTE Should use namespace instead of class, since there are no attributes in the class
class kinematic
{   
    protected:

        RowVector6d ER10_joints, ER20_1_joints, ER20_2_joints, ER20_3_joints;

        // T1_2 denotes representation of robot 2 with respect to robot 1
        Matrix4d T1_2, T2_1, T1_3, T3_1;
        // The tribot system in based on the base frame of robot 1 

    public:

        kinematic();
        
        RowVector6d getJointPosition(robot_type robot)
        {
            switch (robot)
            {
                case ER10:  return ER10_joints;
                case ER20_1: return ER20_1_joints;
                case ER20_2: return ER20_2_joints;
                case ER20_3: return ER20_3_joints;
            }
        }

        void setJointPosition(RowVector6d jointPosition, robot_type robot)
        {
            switch (robot)
            {
                case ER10:  ER10_joints = jointPosition; break;
                case ER20_1:  ER20_1_joints = jointPosition; break;
                case ER20_2: ER20_2_joints = jointPosition; break;
                case ER20_3: ER20_3_joints = jointPosition; break;
            }
        }

        /*
        * @brief Inverse kinematics of ER10 and ER20 robots
        * @param T1 The 4x4 homogenous pose matrix of robot end-effector 
        * @param robot_name "ER10" or "ER20"
        * @param iknum select a set of inverse kinematics solution
        */
        // MatrixXd ik(MatrixXd T1, string robot_name, int iknum);
        MatrixXd ik(Matrix4d T1, robot_type robot_name);

        /*
        * @brief Inverse kinematics of ER10 and ER20 robots
        * @param T1 The 4x4 homogenous pose matrix of robot end-effector 
        * @param robot_name "ER10", "ER20_1", or "ER20_2"
        * @param iknum select a set of inverse kinematics solution
        * @param currentPosition
        */
        RowVector6d optimalIK(const Matrix4d &T1, robot_type robot_name);
        
        /*
        * @brief Inverse kinematics of manipulators
        * @param pose 1x6 vector of cartesian pose (x, y, z, rx, ry, rz). Unit (m, rad). The orientation is represented by ZYX Euler angle.
        * @return 1x6 vector of joint angles
        */
        RowVector6d IK_pose(const RowVector6d &pose, robot_type robot);
        
        RowVector6d IK_pose(geometry_msgs::Pose pose, robot_type robot);
        

        /*
        * @brief convert 1x6 pose vector to 4x4 transformation matrix
        * @param
        * @return
        */
        Matrix4d pose2T(RowVector6d pose);
        
        /*
        * @brief transform representation with respect to robot 2
        * @param
        * @return
        */
        Matrix4d wrt_T2(Matrix4d T)
        {
            return T2_1 * T;
        }

        /*
        * @brief transform representation with respect to robot 2
        * @param
        * @return
        */
        Matrix4d wrt_T3(Matrix4d T)
        {
            return T3_1 * T;
        }

        /*
        * @brief matrix of DH parameters
        * @param 
        */
        Matrix4d T_i1_i(double alpha,double a,double d,double theta);

        /*
        * @brief Rotation about X axis. Return a 3x3 rotation matrix
        * @param
        */
        Matrix3d Rx(double a);

        /*
        * @brief Rotation about Y axis. Return a 3x3 rotation matrix
        * @param
        */
        Matrix3d Ry(double a);

        /*
        * @brief Rotation about Z axis. Return a 3x3 rotation matrix
        * @param
        */
        Matrix3d Rz(double a);

        /*
        * @brief forward kinematics. Return a 4x4 transformaiton matrix unit: mm
        * @param theta 6x1 vector of joint position
        * @param robot_name "ER20" or "ER10"
        */
        Matrix4d fk(RowVector6d theta, robot_type robot_name);

        /*
        * @brief trajectory planning in joint space, using fifth order polynomial interpolation. Equivalent to movej.
        * @param intial initial joint position (rad)
        * @param final final joint position (rad)
        * @param time total motion time (s)
        * @param step the number of interval that you want to split the motion into
        * @return (step+1)x1 vector of joint angles
        */
        MatrixXd trajectory_plan(double initial, double final, double time, int step );

        /*
        * @brief trajectory planning in joint space, using fifth order polynomial interpolation. Equivalent to movej.
        * @param intialPosition, 1x6 vector initial joint position (rad)
        * @param endPosition, 1x6 vector final joint position (rad)
        * @param time total motion time (s)
        * @param step the number of interval that you want to split the motion into
        * @return (step+1)x1 vector of joint angles
        */
        MatrixXd trajectory_plan_6axes(RowVector6d initialPosition, RowVector6d endPosition, double time, int step);

        MatrixXd euler(double rx, double ry, double rz);
        
        /*
        * @brief return a 3x3 rotation matrix of a ZYX euler angle
        * @param
        */
        Matrix3d eulerZYX(double rx,double ry,double rz);

        /*
        * @brief velocity planning
        * @param
        */
        MatrixXd velocity_plan(double initial, double final, double time, int step);
        
        /*
        * @brief
        * @param
        */
        MatrixXd multiarm_kinematic(MatrixXd po_or, double l1, double l2, double l1_radius, double l2_radius, double l3_radius, int iknum);
        
        /*
        * @brief security constraints of robot motion, in case of too large movement
        * @param
        */
        bool robot_safe(MatrixXd last_step,MatrixXd current_step);
        
        // detect whether the data from ik is NAN or INF
        bool ik_erro_detection(MatrixXd Q);
        
        MatrixXd constant_velocity_plan(double initial, double final, int step);

}; 

#endif