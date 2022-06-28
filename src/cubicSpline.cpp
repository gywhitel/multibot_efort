#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include "cubicSpline.h"
#include "eigen3/Eigen/Core"
#include <vector>
#include <iostream>


const double PI = 3.14159265358979;

cubicSpline::cubicSpline()
{
}

cubicSpline::~cubicSpline()
{
    releaseMem();
}

void cubicSpline::initParam()
{
    x_sample_ = y_sample_ = M_ = NULL;
    sample_count_ = 0;
    bound1_ = bound2_ = 0;
}

void cubicSpline::releaseMem()
{
    delete x_sample_;
    delete y_sample_;
    delete M_;

    initParam();
}

bool cubicSpline::loadData(double *x_data, double *y_data, int count, double bound1, double bound2, BoundType type)
{
    if ((NULL == x_data) || (NULL == y_data) || (count < 3)
        || (type > BoundType_Second_Derivative) || (type < BoundType_First_Derivative))
    {
        throw("Invalid condition !");
        return false;
    }

    initParam();

    x_sample_ = new double[count];
    y_sample_ = new double[count];
    M_        = new double[count];
    sample_count_ = count;

    memcpy(x_sample_, x_data, sample_count_*sizeof(double));
    memcpy(y_sample_, y_data, sample_count_*sizeof(double));

    // using namespace std;
    // for (int i = 0; i < count; i++)
    // {
    //     cout<<"("<<x_sample_[i]<<", "<<y_sample_[i]<<") ";
    // }

    bound1_ = bound1;
    bound2_ = bound2;

    return spline(type);
}

void cubicSpline::load_sample_point(vector<double>& x, vector<double>& y, BoundType boundaryCondition, double boundary_condition1, double boundary_condition2)
{
    assert (x.size() == y.size());
    x_sample = x; y_sample = y;
    bound1_ = boundary_condition1;
    bound2_ = boundary_condition2;
    sample_count_ = x.size();
}

bool cubicSpline::spline(BoundType type)
{
    // TODO reconstruct this function using Eigen sparse
    if ((type < BoundType_First_Derivative) || (type > BoundType_Second_Derivative))
        return false;

    //  追赶法解方程求二阶偏导数
    double f1=bound1_, f2=bound2_;

    double *a=new double[sample_count_];                //  a:稀疏矩阵最下边一串数
    double *b=new double[sample_count_];                //  b:稀疏矩阵最中间一串数
    double *c=new double[sample_count_];                //  c:稀疏矩阵最上边一串数
    double *d=new double[sample_count_];

    double *f=new double[sample_count_];

    double *bt=new double[sample_count_];
    double *gm=new double[sample_count_];

    double *h=new double[sample_count_];

    for(int i=0;i<sample_count_;i++)
        b[i]=2;                                //  中间一串数为2
    for(int i=0;i<sample_count_-1;i++)
        h[i]=x_sample_[i+1]-x_sample_[i];      // 各段步长
    for(int i=1;i<sample_count_-1;i++)
        a[i]=h[i-1]/(h[i-1]+h[i]);
    a[sample_count_-1]=1;

    c[0]=1;
    for(int i=1;i<sample_count_-1;i++)
        c[i]=h[i]/(h[i-1]+h[i]);

    for(int i=0;i<sample_count_-1;i++)
        f[i]=(y_sample_[i+1]-y_sample_[i])/(x_sample_[i+1]-x_sample_[i]);

    for(int i=1;i<sample_count_-1;i++)
        d[i]=6*(f[i]-f[i-1])/(h[i-1]+h[i]);

    //  追赶法求解方程
    if(BoundType_First_Derivative == type)
    {
        d[0]=6*(f[0]-f1)/h[0];
        d[sample_count_-1]=6*(f2-f[sample_count_-2])/h[sample_count_-2];

        bt[0]=c[0]/b[0];
        for(int i=1;i<sample_count_-1;i++)
            bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

        gm[0]=d[0]/b[0];
        for(int i=1;i<=sample_count_-1;i++)
            gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

        M_[sample_count_-1]=gm[sample_count_-1];
        for(int i=sample_count_-2;i>=0;i--)
        {
            M_[i]=gm[i]-bt[i]*M_[i+1];
            // std::cout<<M_[i]<<", ";
        }
    }
    else if(BoundType_Second_Derivative == type)
    {
        d[1]=d[1]-a[1]*f1;
        d[sample_count_-2]=d[sample_count_-2]-c[sample_count_-2]*f2;

        bt[1]=c[1]/b[1];
        for(int i=2;i<sample_count_-2;i++)
            bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

        gm[1]=d[1]/b[1];
        for(int i=2;i<=sample_count_-2;i++)
            gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

        M_[sample_count_-2]=gm[sample_count_-2];
        for(int i=sample_count_-3;i>=1;i--)
            M_[i]=gm[i]-bt[i]*M_[i+1];

        M_[0]=f1;
        M_[sample_count_-1]=f2;
    }
    else
        return false;

    delete a;
    delete b;
    delete c;
    delete d;
    delete gm;
    delete bt;
    delete f;
    delete h;

    return true;
}

bool cubicSpline::getYbyX(double &x_in, double &y_out)
{
    int klo,khi,k;
    klo=0;
    khi=sample_count_-1;
    double hh,bb,aa;

    //  二分法查找x所在区间段
    while(khi-klo>1)
    {
        k=(khi+klo)>>1;
        if(x_sample_[k]>x_in)
            khi=k;
        else
            klo=k;
    }
    hh=x_sample_[khi]-x_sample_[klo];

    aa=(x_sample_[khi]-x_in)/hh;
    bb=(x_in-x_sample_[klo])/hh;

    y_out=aa*y_sample_[klo]+bb*y_sample_[khi]+((aa*aa*aa-aa)*M_[klo]+(bb*bb*bb-bb)*M_[khi])*hh*hh/6.0;

    //////test
    double acc = 0, vel = 0;
    acc = (M_[klo]*(x_sample_[khi]-x_in) + M_[khi]*(x_in - x_sample_[klo])) / hh;
    vel = M_[khi]*(x_in - x_sample_[klo]) * (x_in - x_sample_[klo]) / (2 * hh)
        - M_[klo]*(x_sample_[khi]-x_in) * (x_sample_[khi]-x_in) / (2 * hh)
        + (y_sample_[khi] - y_sample_[klo])/hh
        - hh*(M_[khi] - M_[klo])/6;
    printf("%0.9f, %0.9f, %0.9f\n",y_out, vel, acc);
    //////test end

    return true;
}

double cubicSpline::single_point_interpolate(double time)
{
    int klo,khi,k;
    klo=0;
    khi=sample_count_-1;
    double hh,bb,aa;

    //  二分法查找x所在区间段
    while(khi-klo>1)
    {
        k=(khi+klo)>>1;     // divided by 2
        if(x_sample_[k]>time)
            khi=k;          // upper bound
        else
            klo=k;          // lower bound
    }
    hh=x_sample_[khi]-x_sample_[klo];

    aa=(x_sample_[khi]-time)/hh;
    bb=(time-x_sample_[klo])/hh;

    // double q = aa*y_sample_[klo]+bb*y_sample_[khi]+((aa*aa*aa-aa)*M_[klo]+(bb*bb*bb-bb)*M_[khi])*hh*hh/6.0;
    // std::cout<<q<<",";      // DEBUG nan
    return aa*y_sample_[klo]+bb*y_sample_[khi]+((aa*aa*aa-aa)*M_[klo]+(bb*bb*bb-bb)*M_[khi])*hh*hh/6.0;
}

/*
* @brief
* @param
* @return
*/
std::vector<double> cubicSpline::interpolate(double* jointPosition, double* timeScaling, int size, double timeInterval)
{
    using namespace std;
    // boundary condition is zero first order derivation (joint velocity)
    loadData(timeScaling, jointPosition, size, 0, 0, BoundType_First_Derivative);
    std::vector<double> interpolated_joint_position;
    for (double time = timeScaling[0]; time < timeScaling[size-1]; time += timeInterval)
    {
        double interpolatedPosition = single_point_interpolate(time);   // DEBUG nan
        interpolated_joint_position.push_back(interpolatedPosition);
    }
    return interpolated_joint_position;
}

/*
* @brief cubic spline interpolation of 6 axes trajectory from initial position to final position with some viapoints. Even time interval
* @param initial 1x6 joint position vector. This denotes the start position, and it is usually current joint position.
* @param final 1x6 joint position vector. This is where the motion ends.
* @param viapoints vector of *jointTrajectory* struct (jointPosition and timeSequence)
* @param step how many steps you want to interpolate
* @param time total time the motion lasts
* @return nx6 joint position vector (including intial and final position)
*/
vector<RowVector6d> cubicSplinePlan(RowVector6d initial, RowVector6d final, vector<jointTrajectory> viapoints, int step, double time)
{
    if (time <= viapoints.back().time)
    {
        throw("Invalid time scaling !");
    }
    std::size_t axis, i, j;
    int SIZE = viapoints.size() + 2;    // size of viapoints + initial & final
    double jointPosition[SIZE], timeScaling[SIZE];
    double timeInterval = time / step;

    // vector<double> singleJoint;
    vector<RowVector6d> all_axes_position;  // (step+1) x 6
    // all_axes_position.resize(step+1);
    bool INITIAL = true;
    for (axis = 0; axis < 6; axis++)
    {   // loop for 6 axes
        jointPosition[0] = initial(axis);
        timeScaling[0] = 0;
        jointPosition[SIZE-1] = final(axis);
        timeScaling[SIZE-1] = time;
        for (i = 1; i < SIZE-1; i++)
        {
            jointPosition[i] = viapoints[i-1].jointPosition(axis);
            timeScaling[i] = viapoints[i-1].time;
        }
        cubicSpline splineInterpolator;
        // cubic spline interpolation of a joint
        vector<double> singleJoint = splineInterpolator.interpolate(jointPosition, timeScaling, SIZE, timeInterval);
        // write interpolated joint position into a vector 
        for (j = 0; j < singleJoint.size(); j++)
        {   // each step
            if (INITIAL)    // append the joint variable into vector<RowVector6d>
            {
                RowVector6d six_axes_joint_position = RowVector6d::Zero();
                // RowVector6d six_axes_joint_position = {singleJoint[j],0.0,0.0,0.0,0.0,0.0};
                all_axes_position.push_back(six_axes_joint_position);
            }
            // else
            all_axes_position[j](axis) = singleJoint[j];     // include start and end ? Y
        }
        std::cout<<"\n";
        INITIAL = false;
    }
    return all_axes_position;
}