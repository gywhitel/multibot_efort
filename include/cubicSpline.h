#ifndef _CUBIC_SPLINE_H_
#define _CUBIC_SPLINE_H_

#include <vector>
#include <eigen3/Eigen/Core>

using std::vector;
using std::array;
using namespace Eigen;

typedef Matrix<double, 1, 6> RowVector6d;
// typedef array<double, 6> timeSequence;

struct jointTrajectory
{
    RowVector6d jointPosition;
    double time;

    jointTrajectory(): jointPosition(), time() {}      // non-parameter construction
    /*
    * @brief
    * @param time 1x6 vector of time of each joint
    */
    jointTrajectory(RowVector6d jointAngle, double passTime): jointPosition(jointAngle), time(passTime){}
};

class cubicSpline
{
protected:
    typedef enum _BoundType
    {
        BoundType_First_Derivative,     //
        BoundType_Second_Derivative     //
    }BoundType;

    vector<double> x_sample, y_sample;

public:
    cubicSpline();
    ~cubicSpline();

    void initParam();
    void releaseMem();
    /*
    * @brief
    * @param
    * @return
    */
    bool loadData(double *x_data, double *y_data, int count, double bound1, double bound2, BoundType type);
    
    void load_sample_point(vector<double>& x, vector<double>& y, BoundType boundaryCondition, double boundary_condition1, double boundary_condition2);
    
    bool getYbyX(double &x_in, double &y_out);
    
    double single_point_interpolate(double time);

    std::vector<double> interpolate(double* jointPosition, double* timeScaling, int size, double timeInterval);

protected:
    bool spline(BoundType type);

protected:
    double *x_sample_, *y_sample_;
    double *M_;
    int sample_count_;
    double bound1_, bound2_;
};

vector<RowVector6d> cubicSplinePlan(RowVector6d initial, RowVector6d final, vector<jointTrajectory> viapoints, int step, double time);


#endif /* _CUBIC_SPLINE_H_ */