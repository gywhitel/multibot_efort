#include "kinematic.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <string>
#include <cmath>
#include <math.h>
#include <vector>
#include "color_print.h"

#define pi 3.14159265358979

using std::vector;
using std::array;
using std::string;
using std::cout;
using std::endl;
using namespace Eigen;

/*
* @brief constraint joint angles within [-PI, PI]
* @param
* @return
*/
double jointRange(double jointAngle)
{
    while (jointAngle > pi)
    {
        jointAngle -= 2*pi;
    }
    while (jointAngle < -pi)
    {
        jointAngle += 2*pi;
    }
    return jointAngle;
}

double jointRange2(double jointAngle)
{
    while (jointAngle > 2*pi)
    {
        jointAngle -= 2*pi;
    }
    while (jointAngle < -2*pi)
    {
        jointAngle += 2*pi;
    }
    return jointAngle;
}

kinematic::kinematic()
{
            T1_2 = Matrix4d::Identity();
            T1_2(1,3) = -2;
            T2_1 = Matrix4d::Identity();
            T2_1(1,3) = 2;
            T1_3<< -1, 0, 0, 2.1,
                    0, -1, 0, -1,
                    0, 0, 1,  0,
                    0, 0, 0, 1;
            T3_1 = T1_3.inverse();
};

MatrixXd kinematic::ik(Matrix4d T1, robot_type robot_name)
{

    MatrixXd T67(4,4);
    MatrixXd T(4,4);
    double r11, r12, r13, px,
           r21, r22, r23, py,
           r31, r32, r33, pz;

    double L1, L2, L3, L4, L5, L6, L7, tool_length = 0.0;

    if (robot_name == ER20_1 | robot_name == ER20_2 | robot_name == ER20 | robot_name == ER20_3)
    {
        L1 = 0.504; 
        L2 = 0.7812709; 
        L3 = 0.1403276; 
        L4 = 0.7602922; 
        L5 = 0.137; 
        L6 = 0.170; 
        L7 = 0.3278/1000;
        if (ER20_1 == robot_name | ER20_2 == robot_name)
            tool_length = 0.068;
        if (ER20_3 == robot_name)
            tool_length = 0.30;
    }

    if (robot_name == ER10)
    {
        L1 = 0.422; 
        L2 = 0.6812414; 
        L3 = 0.1740841; 
        L4 = 0.7450417; 
        L5 = 0.1175; 
        L6 = 0.1948212; 
        L7 = 0.2289/1000;
        tool_length = 0.120;
    }

    T67 = T_i1_i( 0.0,0.0,L5+tool_length,0.0 );
    // Matrix4d tool = T_i1_i(0,tool_length,0,0);
    T = T1 * T67.inverse();
    r11 = T(0,0); r12 = T(0,1); r13 = T(0,2); px = T(0,3);
    r21 = T(1,0); r22 = T(1,1); r23 = T(1,2); py = T(1,3);
    r31 = T(2,0); r32 = T(2,1); r33 = T(2,2); pz = T(2,3);

    MatrixXd theta(8,6);
    MatrixXd out(1,6);
    double theta1, theta2, theta3, theta4, theta5, theta6, theta23;
    double A1, B1, C1, D1, E1, F1, G1, H1, I1, J1, el, lamda, sin23, cos23, f, K ,O;
    MatrixXd sysm(2,4);
    sysm << 1, 1,  -1, -1,
            1, -1, 1,  -1;
    // MatrixXd Matrix1(2,6);
    // MatrixXd Matrix2(2,6); 
    for (int i = 0; i < 4; i++)
    {
        //theta1
        // theta1 = atan2(py,px);

        f = sqrt(pow(px,2) + pow(py,2));
        K = -L7/f;
        O = sqrt(1 - pow(L7,2)/pow(f,2));
        theta1= atan2(py,px) - atan2(K,sysm(0,i) * O);
        theta1 = jointRange(theta1);
        //theta3
        A1 = px*cos(theta1) + py*sin(theta1) - L6;
        B1 = pz - L1;
        lamda = (pow(A1,2) + pow(B1,2) - pow(L2,2) - pow(L3,2) - pow(L4,2)) / (2 * L2);
        el = sqrt(pow(L3,2) + pow(L4,2));
        theta3 = atan2(lamda/el,sysm(1,i) * sqrt(1 - pow(lamda,2)/pow(el,2))) - atan2(L3,L4);
        // theta3 = jointRange(theta3);
        //-------------------------------------------------------------------------
        //theta23
        C1 = L3 + L2 * cos(theta3);
        D1 = -L4 - L2 * sin(theta3);
        cos23 = (A1*C1 + B1*D1) / (pow(A1,2) + pow(B1,2));
        sin23 = (B1*C1 - A1*D1) / (pow(A1,2) + pow(B1,2));
        theta23 = atan2(sin23,cos23);
        //theta2
        theta2 = theta23 - theta3;
        theta2 = jointRange(theta2);
        //-------------------------------------------------------------------------
        //theta4
        E1 = r33*cos(theta2)*sin(theta3) + r33*cos(theta3)*sin(theta2) + r13*cos(theta1)*cos(theta2)*cos(theta3) + r23*cos(theta2)*cos(theta3)*sin(theta1) - r13*cos(theta1)*sin(theta2)*sin(theta3) - r23*sin(theta1)*sin(theta2)*sin(theta3);
        F1 = r13*sin(theta1) - r23*cos(theta1);
        theta4 = atan2(F1,E1);
        // theta4 = jointRange(theta4);
        //-------------------------------------------------------------------------
        //theta5
        G1 = r13*sin(theta1)*sin(theta4) - r23*cos(theta1)*sin(theta4) + r33*cos(theta2)*cos(theta4)*sin(theta3) + r33*cos(theta3)*cos(theta4)*sin(theta2) + r13*cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4) + r23*cos(theta2)*cos(theta3)*cos(theta4)*sin(theta1) - r13*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) - r23*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3);
        H1 = r33*sin(theta2)*sin(theta3) - r33*cos(theta2)*cos(theta3) + r13*cos(theta1)*cos(theta2)*sin(theta3) + r13*cos(theta1)*cos(theta3)*sin(theta2) + r23*cos(theta2)*sin(theta1)*sin(theta3) + r23*cos(theta3)*sin(theta1)*sin(theta2);
        theta5 = atan2(G1,H1);
        //-------------------------------------------------------------------------
        //theta6
        I1 = r11*cos(theta4)*sin(theta1) - r21*cos(theta1)*cos(theta4) - r31*cos(theta2)*sin(theta3)*sin(theta4) - r31*cos(theta3)*sin(theta2)*sin(theta4) - r11*cos(theta1)*cos(theta2)*cos(theta3)*sin(theta4) - r21*cos(theta2)*cos(theta3)*sin(theta1)*sin(theta4) + r11*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4) + r21*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4);   //sin
        J1 = r31*cos(theta2)*cos(theta3)*sin(theta5) - r21*cos(theta1)*cos(theta5)*sin(theta4) + r11*cos(theta5)*sin(theta1)*sin(theta4) - r31*sin(theta2)*sin(theta3)*sin(theta5) + r31*cos(theta2)*cos(theta4)*cos(theta5)*sin(theta3) + r31*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2) - r11*cos(theta1)*cos(theta2)*sin(theta3)*sin(theta5) - r11*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta5) - r21*cos(theta2)*sin(theta1)*sin(theta3)*sin(theta5) - r21*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta5) + r11*cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4)*cos(theta5) + r21*cos(theta2)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta1) - r11*cos(theta1)*cos(theta4)*cos(theta5)*sin(theta2)*sin(theta3) - r21*cos(theta4)*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3);
        theta6 = atan2(I1,J1);
        theta6 = jointRange2(theta6);
        // cout<< theta1 <<' '<< theta2<<' ' << theta3<<' ' << theta4<<' ' << theta5 <<' '<< theta6<<' '<<endl;
        

        theta.block(i,0,1,6) << theta1, jointRange(theta2)-pi/2, theta3, theta4, theta5, theta6;
        theta.block(i+4,0,1,6) << theta1, jointRange(theta2)-pi/2, theta3, jointRange2(theta4 + pi), -theta5, jointRange2(theta6 + pi) ;
        // theta.block(i,0,1,6) << theta1, theta2, theta3, theta4, theta5, theta6;
        // theta.block(i+4,0,1,6) << theta1, theta2, theta3, theta4 + pi, -theta5, theta6 + pi;
        
    }
    // return theta;
    RowVector6d currentPosition;
    switch (robot_name)
    {
        case ER20_1:
            currentPosition = ER20_1_joints;
            break;
        case ER20_2:
            currentPosition = ER20_2_joints;
            break;
        case ER20_3:
            currentPosition = ER20_3_joints;
            break;
    }
    //the output is one group of 8 groups
    out = theta.block(0,0,1,6);
    
    if (out(3) - currentPosition(3) > 0.9*2*M_PI)
    {
        out(3) = jointRange2(out(3)- 2*M_PI);
    }
    if (out(3) - currentPosition(3) < 0.9*2*M_PI)
    {
        out(3) = jointRange2(out(3) + 2*M_PI);
    } 
    // if (out(5) - currentPosition(5) > 0.9*2*M_PI)
    // {
    //     out(5) = jointRange2(out(5)- 2*M_PI);
    // }
    // if (out(5) - currentPosition(5) < 0.9*2*M_PI)
    // {
    //     out(5) = jointRange2(out(5) + 2*M_PI);
    // } 
    return out;
}

RowVector6d kinematic::optimalIK(const Matrix4d& T1, robot_type robot_name)
{
    Matrix4d T67;
    Matrix4d T;

    double r11, r12, r13, px,
           r21, r22, r23, py,
           r31, r32, r33, pz;

    double L1, L2, L3, L4, L5, L6, L7, tool_length = 0.0;
    RowVector6d currentPosition;
    if (robot_name == ER10)
    {
        L1 = 0.422; 
        L2 = 0.6812414; 
        L3 = 0.1740841; 
        L4 = 0.7450417; 
        L5 = 0.1175; 
        L6 = 0.1948212; 
        L7 = 0.0002289;
        currentPosition = ER10_joints;
    }

    if (robot_name == ER20_1 | robot_name == ER20_2 | robot_name == ER20 | robot_name == ER20_3)
    {
        L1 = 0.504;     // P1
        L2 = 0.7812709;     // P4
        L3 = 0.1403276;     // P5
        L4 = 0.7602922;     // P6
        L5 = 0.137;     // P7
        L6 = 0.170;     // P2
        L7 = 0.0003278;    // P3
        if (ER20_1 == robot_name | ER20_2 == robot_name)
            tool_length = 0.068;
        if (robot_name == ER20_3)    
            tool_length = 0.30;
    }
    T67 = T_i1_i( 0.0, 0.0, L5 + tool_length, 0.0 );  // ?
    T = T1 * T67.inverse();
    r11 = T(0,0); r12 = T(0,1); r13 = T(0,2); px = T(0,3);
    r21 = T(1,0); r22 = T(1,1); r23 = T(1,2); py = T(1,3);
    r31 = T(2,0); r32 = T(2,1); r33 = T(2,2); pz = T(2,3);

    MatrixXd theta(8,6);
    MatrixXd out(1,6);
    double theta1, theta2, theta3, theta4, theta5, theta6, theta23;
    double A1, B1, C1, D1, E1, F1, G1, H1, I1, J1, el, lamda, sin23, cos23, f, K ,O;
    MatrixXd sysm(2,4);
    sysm << 1, 1,  -1, -1,
            1, -1, 1,  -1;
    // MatrixXd Matrix1(2,6);
    // MatrixXd Matrix2(2,6); 
    for (int i = 0; i < 4; i++)
    {
        //theta1
        // theta1 = atan2(py,px);

        f = sqrt(pow(px,2) + pow(py,2));
        K = -L7/f;
        O = sqrt(1 - pow(L7,2)/pow(f,2));
        theta1= atan2(py,px) - atan2(K,sysm(0,i) * O);
        theta1 = jointRange(theta1);
        //theta3
        A1 = px*cos(theta1) + py*sin(theta1) - L6;
        B1 = pz - L1;
        lamda = (pow(A1,2) + pow(B1,2) - pow(L2,2) - pow(L3,2) - pow(L4,2)) / (2 * L2);
        el = sqrt(pow(L3,2) + pow(L4,2));
        theta3 = atan2(lamda/el,sysm(1,i) * sqrt(1 - pow(lamda,2)/pow(el,2))) - atan2(L3,L4);
        theta3 = jointRange(theta3);
        //-------------------------------------------------------------------------
        //theta23
        C1 = L3 + L2 * cos(theta3);
        D1 = -L4 - L2 * sin(theta3);
        cos23 = (A1*C1 + B1*D1) / (pow(A1,2) + pow(B1,2));
        sin23 = (B1*C1 - A1*D1) / (pow(A1,2) + pow(B1,2));
        theta23 = atan2(sin23,cos23);
        //theta2
        theta2 = theta23 - theta3;
        theta2 = jointRange(theta2);
        //-------------------------------------------------------------------------
        //theta4
        E1 = r33*cos(theta2)*sin(theta3) + r33*cos(theta3)*sin(theta2) + r13*cos(theta1)*cos(theta2)*cos(theta3) + r23*cos(theta2)*cos(theta3)*sin(theta1) - r13*cos(theta1)*sin(theta2)*sin(theta3) - r23*sin(theta1)*sin(theta2)*sin(theta3);
        F1 = r13*sin(theta1) - r23*cos(theta1);
        theta4 = atan2(F1,E1);
        theta4 = jointRange2(theta4);
        //-------------------------------------------------------------------------
        //theta5
        G1 = r13*sin(theta1)*sin(theta4) - r23*cos(theta1)*sin(theta4) + r33*cos(theta2)*cos(theta4)*sin(theta3) + r33*cos(theta3)*cos(theta4)*sin(theta2) + r13*cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4) + r23*cos(theta2)*cos(theta3)*cos(theta4)*sin(theta1) - r13*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) - r23*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3);
        H1 = r33*sin(theta2)*sin(theta3) - r33*cos(theta2)*cos(theta3) + r13*cos(theta1)*cos(theta2)*sin(theta3) + r13*cos(theta1)*cos(theta3)*sin(theta2) + r23*cos(theta2)*sin(theta1)*sin(theta3) + r23*cos(theta3)*sin(theta1)*sin(theta2);
        theta5 = atan2(G1,H1);
        //-------------------------------------------------------------------------
        //theta6
        I1 = r11*cos(theta4)*sin(theta1) - r21*cos(theta1)*cos(theta4) - r31*cos(theta2)*sin(theta3)*sin(theta4) - r31*cos(theta3)*sin(theta2)*sin(theta4) - r11*cos(theta1)*cos(theta2)*cos(theta3)*sin(theta4) - r21*cos(theta2)*cos(theta3)*sin(theta1)*sin(theta4) + r11*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4) + r21*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4);   //sin
        J1 = r31*cos(theta2)*cos(theta3)*sin(theta5) - r21*cos(theta1)*cos(theta5)*sin(theta4) + r11*cos(theta5)*sin(theta1)*sin(theta4) - r31*sin(theta2)*sin(theta3)*sin(theta5) + r31*cos(theta2)*cos(theta4)*cos(theta5)*sin(theta3) + r31*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2) - r11*cos(theta1)*cos(theta2)*sin(theta3)*sin(theta5) - r11*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta5) - r21*cos(theta2)*sin(theta1)*sin(theta3)*sin(theta5) - r21*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta5) + r11*cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4)*cos(theta5) + r21*cos(theta2)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta1) - r11*cos(theta1)*cos(theta4)*cos(theta5)*sin(theta2)*sin(theta3) - r21*cos(theta4)*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3);
        theta6 = atan2(I1,J1);
        // cout<< theta1 <<' '<< theta2<<' ' << theta3<<' ' << theta4<<' ' << theta5 <<' '<< theta6<<' '<<endl;
        
        theta.block(i,0,1,6) << theta1, jointRange(theta2)-pi/2, theta3, theta4, theta5, theta6;
        theta.block(i+4,0,1,6) << theta1, jointRange(theta2)-pi/2, theta3, jointRange2(theta4 + pi), -theta5, jointRange2(theta6 + pi);
        
    }

    switch (robot_name)
    {
        case ER20_1:
            currentPosition = ER20_1_joints;
            break;
        case ER20_2:
            currentPosition = ER20_2_joints;
            break;
        case ER20_3:
            currentPosition = ER20_3_joints;
            break;
    }
    //the output is one group of 8 groups
    double sum = 0, cost = 1e10;
    int select=0;
    for (int i = 0; i < theta.rows(); i++)
    {
        sum = 0.0;
        for (int j = 0; j < theta.cols(); j++)
        {
            sum += pow(theta(i,j) - currentPosition(j), 2);
        }
        sum += pow(theta(i,0) - currentPosition(0), 2) * 10;
        sum += pow(theta(i,1) - currentPosition(1), 2) * 10;
        if (cost > sum)
        {
            cost = sum;
            select = i;
        }
    }
    out = theta.block(select, 0, 1, 6);
    // if ( (abs(out(3) - currentPosition(3)) > M_PI_2 * 0.8) & (abs(out(5) - currentPosition(5)) > M_PI_2 * 0.8) )
    // {
    //     out(3) = jointRange(out(3) - M_PI);
    //     out(5) = jointRange(out(5) - M_PI);
    // }
    if (out(3) - currentPosition(3) > 0.9*2*M_PI)
    {
        out(3) = jointRange2(out(3)- 2*M_PI);
    }
    if (out(3) - currentPosition(3) < 0.9*2*M_PI)
    {
        out(3) = jointRange2(out(3) + 2*M_PI);
    }    

    return out;  
}

/*
* @brief 
* @param pose 1x6 pose vector (x,y,z,rx,ry,rz)
* @return 1x6 joint vector
*/
RowVector6d kinematic::IK_pose(const RowVector6d &pose, robot_type robot)
{
    // convert 1x6 pose vector to 4x4 transformation matrix
    Matrix4d T = Matrix4d::Identity();
    T.block(0,3,3,1)<<pose(0), pose(1), pose(2);
    T.block(0,0,3,3) = Rz(pose(5)) * Ry(pose(4)) * Rx(pose(3));
    // T.block(0,0,3,3) = Rx(pose(3)) * Ry(pose(4)) * Rz(pose(5));
    // cout<<T<<endl;
    // RowVector6d jointPosition = optimalIK(T, robot);
    RowVector6d jointPosition = ik(T, robot);
    setJointPosition(jointPosition, robot);
    return jointPosition;
}

RowVector6d kinematic::IK_pose(geometry_msgs::Pose pose, robot_type robot)
{
    // opposite to normal vector
    Eigen::Quaterniond orientation = Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z); 
    Matrix4d T = Matrix4d::Identity();
    T.block(0,3,3,1)<<pose.position.x, pose.position.y, pose.position.z;
    T.block(0,0,3,3) = orientation.toRotationMatrix();
    // RowVector6d jointPosition = optimalIK(T, robot);
    RowVector6d jointPosition = ik(T, robot);
    setJointPosition(jointPosition, robot);
    // // T = fk(jointPosition, robot);
    // // cout<<"("<<pose.position.x<<","<<pose.position.y<<","<<pose.position.z<<"), ("<<T(0,3)<<T(1,3)<<T(2,3)<<")\n";
    // GREEN_PRINT("Ideal position:");
    // printf("(%4f, %4f, %4f)", pose.position.x, pose.position.y, pose.position.z);
    // YELLOW_PRINT("Actual position:");
    // printf("(%4f, %4f, %4f)\n", T(0,3), T(1,3), T(2,3));
    return jointPosition;
}


Matrix4d kinematic::fk(RowVector6d theta, robot_type robot_name)
{

    double L1, L2, L3, L4, L5, L6, L7;
    if (robot_name == ER20_1 || robot_name == ER20_2 || ER20)
    {
        L1 = 0.504; 
        L2 = 0.7812709; 
        L3 = 0.1403276; 
        L4 = 0.7602922; 
        L5 = 0.137; 
        L6 = 0.170; 
        L7 = 0.0003278;
    }

    if (robot_name == ER10)
    {
        L1 = 0.422; 
        L2 = 0.6812414; 
        L3 = 0.1740841; 
        L4 = 0.7450417; 
        L5 = 0.1175; 
        L6 = 0.1948212; 
        L7 = 0.0002289;
    }
    
    MatrixXd DH(6,4);
    MatrixXd T01(4,4);
    MatrixXd T12(4,4);
    MatrixXd T23(4,4);
    MatrixXd T34(4,4);
    MatrixXd T45(4,4);
    MatrixXd T56(4,4);
    MatrixXd T06(4,4);
    Matrix4d tool = Matrix4d::Identity();

    DH << 0,          0,         L1,    theta(0),
          pi/2,       L6,        L7,    theta(1)+pi/2,
          0,          L2,        0,     theta(2),
          pi/2,       L3,        L4,    theta(3),
          -pi/2,      0,         0,     theta(4),
          pi/2,       0,         L5,     theta(5);

    T01 = T_i1_i( DH(0,0),DH(0,1),DH(0,2),DH(0,3) );
    T12 = T_i1_i( DH(1,0),DH(1,1),DH(1,2),DH(1,3) );
    T23 = T_i1_i( DH(2,0),DH(2,1),DH(2,2),DH(2,3) );
    T34 = T_i1_i( DH(3,0),DH(3,1),DH(3,2),DH(3,3) );
    T45 = T_i1_i( DH(4,0),DH(4,1),DH(4,2),DH(4,3) );
    T56 = T_i1_i( DH(5,0),DH(5,1),DH(5,2),DH(5,3) ); 
    if (robot_name == ER10)
        tool(2,3) = 0.12;
    T06 = T01 * T12 * T23 * T34 * T45 * T56 * tool; 
    
    return T06;

}

Matrix4d kinematic::T_i1_i(double alpha,double a,double d,double theta)
{
    Matrix4d T;

    T << cos(theta),              -sin(theta),             0,             a,
     sin(theta)*cos(alpha),   cos(theta)*cos(alpha),   -sin(alpha),   -sin(alpha)*d,
     sin(theta)*sin(alpha),   cos(theta)*sin(alpha),   cos(alpha),    cos(alpha)*d,
     0,                       0,                       0,             1;
    
    return T;
}



Matrix3d kinematic::Rx(double a)
{
    Matrix3d T;

    T << 1,     0,          0,
         0,     cos(a),     -sin(a), 
         0,     sin(a),     cos(a);

    return T;
}

Matrix3d kinematic::Ry(double a)
{
    Matrix3d T;

    T << cos(a),    0,      sin(a),
         0,         1,      0,
         -sin(a),   0,      cos(a);

    return T;
}

Matrix3d kinematic::Rz(double a)
{
    Matrix3d T;

    T << cos(a),    -sin(a),    0,
         sin(a),    cos(a),     0, 
         0,         0,          1;

    return T;
}
    



MatrixXd kinematic::euler(double rx,double ry,double rz)
{
    Matrix3d T;
    T = Rx(rx) * Ry(ry) * Rz(rz);
    return T;
}

Matrix3d kinematic::eulerZYX(double rx,double ry,double rz)
{
    Matrix3d T;
    T = Rz(rz) * Ry(ry) * Rx(rx);
    return T;
}

Matrix4d kinematic::pose2T(RowVector6d pose)
{
    Matrix4d T;
    T.block(0,0,3,3) = eulerZYX(pose(3), pose(4), pose(5));
    T(0,3) = pose(0);
    T(1,3) = pose(1);
    T(2,3) = pose(2);
    return T;
}


MatrixXd kinematic::trajectory_plan(double initial, double final, double time, int step)
{

    // MatrixXd para(1,6);
    RowVector6d para(1,6);
    MatrixXd q(step + 1,1);
    double a0,a1,a2,a3,a4,a5;
    para <<initial,final,0,0,0,0;

    a0=para(0,0);
    a1=para(0,2);
    a2=para(0,4)/2;
    a3=(20*(para(0,1)-para(0,0))-(8*para(0,3)+12*para(0,2))*time-(3*para(0,4)-2*para(0,5))*pow(time,2))/(2*pow(time,3));
    a4=(30*(para(0,0)-para(0,1))+(14*para(0,3)+16*para(0,2))*time+(3*para(0,4)-2*para(0,5))*pow(time,2))/(2*pow(time,4));
    a5=(12*(para(0,1)-para(0,0))-(6*para(0,3)+6*para(0,2))*time-(para(0,4)-para(0,5))*pow(time,2))/(2*pow(time,5));


    // time=time;tt=time/step;
    double t;
    
    for(int i = 0; i <= step; i++)
    {

        t = (time/(double)step) * i;
        q(i,0)=a0 + a1 * t + a2 * pow(t,2) + a3 * pow(t,3) + a4 * pow(t,4) + a5 * pow(t,5);

    }
    return q;

}

MatrixXd kinematic::trajectory_plan_6axes(RowVector6d initialPosition, RowVector6d endPosition, double time, int step)
{
    // MatrixXd q0(step+1,1), q1(step+1,1), q2(step+1,1), q3(step+1,1), q4(step+1,1), q5(step+1,1);
    MatrixXd jointAngle(step+1, 6);
    for (int i = 0; i < 6; i++)
    {
        jointAngle.block(0,i,step+1,1) = trajectory_plan(initialPosition(i), endPosition(i), time, step);
    }
    return jointAngle;
}

MatrixXd kinematic::velocity_plan(double initial, double final, double time, int step)
{

    MatrixXd para(1,6);
    MatrixXd q_dot(step + 1,1);
    double a0,a1,a2,a3,a4,a5;
    para <<initial,final,0,0,0,0;

    a0=para(0,0);
    a1=para(0,2);
    a2=para(0,4)/2;
    a3=(20*(para(0,1)-para(0,0))-(8*para(0,3)+12*para(0,2))*time-(3*para(0,4)-2*para(0,5))*pow(time,2))/(2*pow(time,3));
    a4=(30*(para(0,0)-para(0,1))+(14*para(0,3)+16*para(0,2))*time+(3*para(0,4)-2*para(0,5))*pow(time,2))/(2*pow(time,4));
    a5=(12*(para(0,1)-para(0,0))-(6*para(0,3)+6*para(0,2))*time-(para(0,4)-para(0,5))*pow(time,2))/(2*pow(time,5));


    // time=time;tt=time/step;
    double t;
    
    for(int i = 0; i <= step; i++)
    {

        t = (time/(double)step) * i;
        q_dot(i,0)=a1 + 2 * a2 * t + 3 * a3 * pow(t,2) + 4 * a4 * pow(t,3) + 5 * a5 * pow(t,4);

    }


    return q_dot;

}


MatrixXd kinematic::constant_velocity_plan(double initial, double final, int step)
{

    MatrixXd q(step + 1,1);
    double add;
    for (int i = 0; i <= step; i++)
    {

        add = final - initial;
        q(i,0) = initial + (((double)i/(double)step) * add);  
        
    }
    return q;
}





/*
MatrixXd kinematic::multiarm_kinematic(MatrixXd po_or, double l1, double l2, 
                                       double l1_radius, double l2_radius, double l3_radius, int iknum)
{
    
    MatrixXd T_W_o(4,4);

    MatrixXd T_w_b1(4,4);
    MatrixXd T_w_b2(4,4);
    MatrixXd T_w_b3(4,4);

    MatrixXd T_o_e1(4,4);
    MatrixXd T_o_e2(4,4);
    MatrixXd T_o_e3(4,4);

    MatrixXd T_b1_e1(4,4);
    MatrixXd T_b2_e2(4,4);
    MatrixXd T_b3_e3(4,4);

    MatrixXd out(1,18);

    T_W_o.block(0,0,3,3) = Rx(po_or(0,3)) * Ry(po_or(0,4)) * Rz(po_or(0,5));
    T_W_o.block(0,3,3,1) << po_or(0,0), po_or(0,1), po_or(0,2);
    T_W_o.block(3,0,1,4) << 0, 0, 0, 1;


    T_w_b1.block(0,0,3,3) = MatrixXd::Identity(3,3);
    T_w_b1.block(0,3,3,1) << -l2/2, l1/2, 0;
    T_w_b1.block(3,0,1,4) << 0, 0, 0, 1;

    T_w_b2.block(0,0,3,3) = MatrixXd::Identity(3,3);
    T_w_b2.block(0,3,3,1) << -l2/2, -l1/2, 0;
    T_w_b2.block(3,0,1,4) << 0, 0, 0, 1;

    T_w_b3.block(0,0,3,3) = Rz(pi);
    T_w_b3.block(0,3,3,1) << l2/2, 0, 0;
    T_w_b3.block(3,0,1,4) << 0, 0, 0, 1;


    double angle = (45.0/180.0) * pi;
    // T_o_e1.block(0,0,3,3) = Rx(pi/2) * Ry(pi/6);
    // T_o_e1.block(0,3,3,1) << -l1_radius*sin(pi/6), l1_radius*cos(pi/6), 0;
    T_o_e1.block(0,0,3,3) = Rx(pi/2) * Ry(angle);    
    T_o_e1.block(0,3,3,1) << -l1_radius*sin(angle), l1_radius*cos(angle), 0;
    T_o_e1.block(3,0,1,4) << 0, 0, 0, 1;

    // T_o_e2.block(0,0,3,3) = Rx(-pi/2) * Ry(pi/6);
    // T_o_e2.block(0,3,3,1) << -l2_radius*sin(pi/6), -l2_radius*cos(pi/6), 0;
    T_o_e2.block(0,0,3,3) = Rx(-pi/2) * Ry(angle);    
    T_o_e2.block(0,3,3,1) << -l2_radius*sin(angle), -l2_radius*cos(angle), 0;
    T_o_e2.block(3,0,1,4) << 0, 0, 0, 1;

    T_o_e3.block(0,0,3,3) = Ry(-pi/2);
    T_o_e3.block(0,3,3,1) << l3_radius, -50, 0;      /////////???????
    T_o_e3.block(3,0,1,4) << 0, 0, 0, 1;

    out.block(0,0,1,6) = ik( T_w_b1.inverse() * T_W_o * T_o_e1, ER20, iknum);
    out.block(0,6,1,6) = ik( T_w_b2.inverse() * T_W_o * T_o_e2, ER20, iknum);
    out.block(0,12,1,6) = ik( T_w_b3.inverse() * T_W_o * T_o_e3, ER10, iknum);
    
    return out;

}
*/


bool kinematic::robot_safe(MatrixXd last_step,MatrixXd current_step)
{
    bool flag = false;
    MatrixXd substraction(1,6);
    substraction << abs( current_step(0,0)-last_step(0,0) ),  abs( current_step(0,1)-last_step(0,1) ),  abs( current_step(0,2)-last_step(0,2) ),
                    abs( current_step(0,3)-last_step(0,3) ),  abs( current_step(0,4)-last_step(0,4) ),  abs( current_step(0,5)-last_step(0,5) );

    if ( substraction(0,0)>0.15 || substraction(0,1)>0.15 || substraction(0,2)>0.15 || substraction(0,3)>0.15 || substraction(0,4)>0.15 || substraction(0,5)>0.15)
    {
        flag = true;
    }

    return flag;
}

/*
* @brief workspace
* @param
* @return
*/
bool kinematic::ik_erro_detection(MatrixXd Q)
{
    //the function of this function is to detect whether the data from ik is NAN or INF
    bool erro_flag = false;
    MatrixXd jointScope(6,2);
    jointScope << (170.0/180.0) * pi, (-170.0/180.0) * pi,
                  (90.0/180.0) * pi, (-90.0/180.0) * pi,
                  (160.0/180) * pi, (-80.0/180.0) * pi,
                  (360.0/180.0) * pi, (-360.0/180.0) * pi,
                  (120.0/180.0) * pi, (-120.0/180.0) * pi,
                  (360.0/180.0) * pi, (-360.0/180.0) * pi;
    for (int i = 0; i < 6; i++)
    {
        bool IS_NAN = false;
        bool IS_INF = false;
        bool IS_beyoundJointScope = false;
        IS_NAN = isnan(Q(0,i));
        IS_INF = isinf(Q(0,i));

        if (IS_NAN == true  || IS_INF == true)
        {
            erro_flag = true;
            cout<<"ERRO in inverse kinematic"<<'\n'<<Q<<endl;
            break;
        }   
        if(Q(0,i) > jointScope(i,0) || Q(0,i) < jointScope(i,1))
        {
            erro_flag = true;
            cout<<"joint is beyound the joint workspace in inverse kinematic"<<'\n'<<"desired joint"<<i<<" "<<Q(0,i);
            cout<<" the joint"<<i<<" socpe is "<< jointScope(i,1)<<" ~ "<< jointScope(i,0)<<endl;
            break;
        }     
    }
    
    return erro_flag;
}

