#include "cubicSpline.h"
#include <iostream>
#include <vector>

using namespace std;

const int SIZE = 7;

int main()
{
    double time[] = {0,1,2,3,4,5,6};
    double q[] = {0, 0.1, 0.4, 0.2, 0.3, 0.6, 0.5};
    cubicSpline cubic_spline_interpolator;
    // cubic_spline_interpolator.loadData(time, q, SIZE, 0, 0, cubicSpline::BoundType_First_Derivative);
    // double t;
    // for (t=0; t < time[SIZE-1]; t+=0.1)
    // {
    //     cout<<cubic_spline_interpolator.single_point_interpolate(t)<<",";
    // }
    vector<double> jointAngles = cubic_spline_interpolator.interpolate(q, time, SIZE, 0.1);
    for (size_t i = 0; i < jointAngles.size(); i++)
    {
        cout<<jointAngles[i]<<",";
    }
    cout<<"\n";
    return 0;
}