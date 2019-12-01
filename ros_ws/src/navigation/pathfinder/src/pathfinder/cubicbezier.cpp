#include "pathfinder/cubicbezier.h"

CubicBezier::CubicBezier()
{
    _P0 = {0, 0};
    _P1 = {0, 0};
    _P2 = {0, 0};
    _P3 = {0, 0};
}

CubicBezier::CubicBezier(Point P0, Point P1, Point P2, Point P3)
{
    _P0 = P0;
    _P1 = P1;
    _P2 = P2;
    _P3 = P3;
}

std::vector<Point> CubicBezier::compute(double tStep)
{
    double x, y;
    double t, tt, ttt, t1, tt1, ttt1;

    int nbPoints = 1 / tStep + 1;

    _points.clear();

    // Explicit form of the curve:
    // B(t) = P0 * (1 - t)^3 + 3 * P1 * t * (t - 1)^2 + 3 * P2 * t^2 * (1 * t) + P3 * t^3
    // with 0 <= t <= 1
    for (int i = 0; i <= nbPoints; i++)
    {
        t = tStep * i;      // t
        tt = t * t;         // t^2
        ttt = tt * t;       // t^3
        t1 = 1 - t;         // (1 - t)
        tt1 = t1 * t1;      // (1 - t)^2
        ttt1 = tt1 * t1;    // (1 - t)^3

        x = _P0.getX() * ttt1 + 3 * _P1.getX() * t * tt1 + 3 * _P2.getX() * tt * t1 + _P3.getX() * ttt;
        y = _P0.getY() * ttt1 + 3 * _P1.getY() * t * tt1 + 3 * _P2.getY() * tt * t1 + _P3.getY() * ttt;

        _points.push_back({x, y});
    }

    return _points;
}

std::vector<Point> CubicBezier::compute(int nbPoints)
{
    double x, y;
    double t, tt, ttt, t1, tt1, ttt1;

    double tStep = 1.0 / (double)nbPoints;

    _points.clear();

    // Explicit form of the curve:
    // B(t) = P0 * (1 - t)^3 + 3 * P1 * t * (t - 1)^2 + 3 * P2 * t^2 * (1 * t) + P3 * t^3
    // with 0 <= t <= 1
    for (int i = 0; i <= nbPoints; i++)
    {
        t = tStep * i;      // t
        tt = t * t;         // t^2
        ttt = tt * t;       // t^3
        t1 = 1 - t;         // (1 - t)
        tt1 = t1 * t1;      // (1 - t)^2
        ttt1 = tt1 * t1;    // (1 - t)^3

        x = _P0.getX() * ttt1 + 3 * _P1.getX() * t * tt1 + 3 * _P2.getX() * tt * t1 + _P3.getX() * ttt;
        y = _P0.getY() * ttt1 + 3 * _P1.getY() * t * tt1 + 3 * _P2.getY() * tt * t1 + _P3.getY() * ttt;

        _points.push_back({x, y});
    }

    return _points;
}

void CubicBezier::setP0(Point P)
{
    _P0 = P;
}

void CubicBezier::setP1(Point P)
{
    _P1 = P;
}

void CubicBezier::setP2(Point P)
{
    _P2 = P;
}

void CubicBezier::setP3(Point P)
{
    _P3 = P;
}

void CubicBezier::setParameters(Point P0, Point P1, Point P2, Point P3)
{
    _P0 = P0;
    _P1 = P1;
    _P2 = P2;
    _P3 = P3;
}

Point CubicBezier::getP0()
{
    return _P0;
}

Point CubicBezier::getP1()
{
    return _P1;
}

Point CubicBezier::getP2()
{
    return _P2;
}

Point CubicBezier::getP3()
{
    return _P3;
}

std::vector<Point> CubicBezier::getPoints()
{
    return _points;
}
