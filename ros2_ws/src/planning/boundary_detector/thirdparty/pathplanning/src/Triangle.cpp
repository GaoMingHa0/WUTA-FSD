#include <cmath>
#include "pathplanning/structs/Triangle.h"

Triangle::Triangle(Point2d a, Point2d b, Point2d c) : tri2midp(3, nullptr), visited(false)
{
    p1 = a; p2 = b; p3 = c;
    //计算外接圆圆心和半径
    double A = pow(p1.x, 2) + pow(p1.y, 2);
    double B = pow(p2.x, 2) + pow(p2.y, 2);
    double C = pow(p3.x, 2) + pow(p3.y, 2);
    double G = (p3.y - p2.y) * p1.x + (p1.y - p3.y) * p2.x + (p2.y - p1.y) * p3.x;
    center.x = ((B - C) * p1.y + (C - A) * p2.y + (A - B) * p3.y) / (2 * G);
    center.y = ((C - B) * p1.x + (A - C) * p2.x + (B - A) * p3.x) / (2 * G);
    r = sqrt(pow(p1.x - center.x, 2) + pow(p1.y - center.y, 2));
}

Triangle::Triangle() : tri2midp(3, nullptr), visited(false)
{
    p1.x = 0; p1.y = 0; p2.x = 0;
    p2.y = 0; p3.x = 0; p3.y = 0;
    r = 0;
}