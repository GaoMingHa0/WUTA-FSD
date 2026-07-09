#pragma once
#include "DelaunayTriangle.h"
class Line;
//分治法待定
struct P_A {
    Point2d p;
    double angle;
};
class Divide : public DelaunayTriangle
{
public:
    //virtual std::vector<Triangle*> delaunay(std::vector<Point2d> points);
private:
    std::vector<Line> div(std::vector<Point2d> points);
    double crossProduct(Point2d a, Point2d b);
    bool checkIntersect(Line e1,Line e2);
    static bool comAngle(P_A& p1, P_A& p2);
     //计算角度,从AB按顺时针方向转到BC
    double angleBetween(Point2d A, Point2d B, Point2d C);
    static bool compareY(Point2d& p1,Point2d& p2);
    static bool xy(Point2d& a, Point2d& b);
    std::vector<Point2d> _points;
    std::vector<Line> _lines;
};