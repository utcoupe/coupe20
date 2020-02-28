#include "pathfinder/curvetrajectory.h"
#include <cmath>
#include "Eigen/Dense"

// Author : Théo Villanueva
// Modifications by Paul Constant

CurveTrajectory::CurveTrajectory(std::vector<Point> points)
{
    _points = points;
}

std::vector<Point> CurveTrajectory::compute(int nbPoints)
{
    // From:
    // https://www.math.ucla.edu/~baker/149.1.02w/handouts/dd_splines.pdf

    if (_points.size() <= 2) 
    {
        return _points;
    }

    int curveNb;

    if (_points.size() == 3)
    {
        // TODO find a way to do it with 3 points only
        // Add dummy point at the end to get 4 points
        _points.push_back(_points[2]);
        // Still want only two curves though
        curveNb = 2;
    }
    else {
        curveNb = _points.size() - 1;
    }

    std::vector<CubicBezier> bezierCurves;

    int n = _points.size() - 2;

    _pathPoints.clear();

    Eigen::MatrixXd M(n, n);
    M.setZero();

    for(int i = 0; i < n; i++)
    {
        M(i, i) = 4;
        if(i != 0) M(i, i - 1) = 1;
        if(i != n - 1) M(i, i + 1) = 1;
    }

    Eigen::MatrixXd C(n, 2);

    for (int i = 0; i < n; i++)
    {
        if(i == 0)
        {
            C(i, 0) = 6 * _points[i+1].getX() - _points[i].getX();
            C(i, 1) = 6 * _points[i+1].getY() - _points[i].getY();
        }
        else if(i == n-1)
        {
            C(i, 0) = 6 * _points[i+1].getX() - _points[i+2].getX();
            C(i, 1) = 6 * _points[i+1].getY() - _points[i+2].getY();
        }
        else
        {
            C(i, 0) = 6 * _points[i+1].getX();
            C(i, 1) = 6 * _points[i+1].getY();
        }
    }

    Eigen::MatrixXd B_temp = M.colPivHouseholderQr().solve(C);

    Eigen::MatrixXd B(n+2, 2);

    B(0, 0) = _points[0].getX();
    B(0, 1) = _points[0].getY();
    B(n+1, 0) = _points[n+1].getX();
    B(n+1, 1) = _points[n+1].getY();

    for(int i = 0; i < n; i++)
        for(int j = 0; j < 2; j++)
            B(i + 1, j) = B_temp(i, j);

    bezierCurves.resize(curveNb);

    for (int i = 0; i < curveNb; i++)
    {
        bezierCurves[i].setP0(_points[i]);

        bezierCurves[i].setP1({B(i, 0) + (B(i + 1, 0) - B(i, 0)) / 3.0,
                                B(i, 1) + (B(i + 1, 1) - B(i, 1)) / 3.0});

        bezierCurves[i].setP2({B(i, 0) + 2.0 * (B(i + 1, 0) - B(i, 0)) / 3.0,
                                B(i, 1) + 2.0 * (B(i + 1, 1) - B(i, 1)) / 3.0});

        bezierCurves[i].setP3(_points[i+1]);
    }
    
    nbPoints = nbPoints / curveNb;

    for (int i = 0; i < curveNb; i++)
    {
        std::vector<Point> bezierCurve = bezierCurves[i].compute(nbPoints);
        _pathPoints.insert(_pathPoints.end(), bezierCurve.begin(), bezierCurve.end());
    }


    return _pathPoints;
}