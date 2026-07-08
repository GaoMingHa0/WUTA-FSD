#pragma once
#include<vector>
#include "MidPoint.h"
class MidPoint;
class Triangle {
public:
    std::vector<std::shared_ptr<MidPoint>> tri2midp;
    Point2d p1, p2, p3;
    Point2d center;
    double r;
    bool visited;
    //初始化三角形
    Triangle(Point2d a, Point2d b, Point2d c);
    Triangle();
};