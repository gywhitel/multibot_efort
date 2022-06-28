#include "kinematic.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <string>
#include <cmath>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include "color_print.h"
#include <ros/ros.h>

const int PI = 3.14159265358979323;

using namespace Eigen;
using namespace std;

double square_sum(RowVectorXd vectorA, RowVectorXd vectorB)
{
    assert (vectorA.cols() == vectorB.cols());
    double sum = 0;
    for (int i = 0; i < vectorA.cols(); i++)
    {
        sum += pow(vectorA(i)-vectorB(i), 2);
    }
    return sum;
}



int main()
{
    RowVector6d jointAngle;
    jointAngle<<0,0,0,0,0,0;
    Matrix4d T, T_;
    MatrixXd ik(8,6);
    kinematic efort;
    int i, j, k;
    double error;
    bool correctIK, IK_FK;
    for (i = 0; i < 10; i++)
    {
        for (j = 0; j < 6; j++)
        {
            jointAngle(j) = (double(rand())/RAND_MAX - 0.5) * 2 *PI;
        }
        cout<<_YELLOW_<<"Randomly generated joint angles: "<<jointAngle<<_RESET_<<endl;
        T = efort.fk(jointAngle, ER10);
        ik = efort.ik(T, ER10);
        correctIK = false;
        IK_FK = true;
        // verify inverse kinematics fk0 -> 8 iks -> 8 fks == fk0 ?
        for (j = 0; j < ik.rows(); j++)
        {
            T_ = efort.fk(ik.row(j), ER10);
            error = (T-T_).norm();  // the difference between FK(IK) and real pose
            if (error > 0.1)
            {
                cout<<_BOLDRED_<<"Incorrect IK ! "<<error<<_RESET_<<endl;
                GREEN_BOLD_PRINT("End-effector pose:\n");
                cout<<T<<endl;
                YELLOW_PRINT("Computed forward kinematics:\n");
                cout<<T_<<endl;
                // cout<<ik.row(j)<<endl;
                IK_FK = false;
                continue;
            }
            error = square_sum(ik.row(j), jointAngle);  // the difference between joint angle and IK
            if (error < 0.1)
            {
                GREEN_BOLD_PRINT("Correct IK\n");
                correctIK = true;
                
            }
        }
            if (IK_FK == true & correctIK == false)
            {
                GREEN_PRINT("FK derived from IK is right.\n");
                RED_PRINT("Correct computed FK but incorrect joint angles\n");
                cout<<ik<<endl;
            }
    }
    /*
    for (i = 0; i < 10; i++)
    {
        for (j = 0; j < 6; j++)
            jointAngle(i) = (double(rand())/RAND_MAX - 0.5) * 2 *PI;
        
        T = efort.fk(jointAngle, ER20);
        ik = efort.ik(T, ER20);
        for (j = 0; j < 8; j++)
        {
            error = square_sum(ik.block(j,0,1,6), jointAngle);
            if (error > 0.1)
            {
                cout<<"Incorrect FK or IK for ER20 ! Joint angle error: "<<error<<" rad^2\n";
            }
        }
    }
    */
}