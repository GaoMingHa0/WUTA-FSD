#pragma once
class Point2d {
public:
    double x, y;
    int color;
    Point2d();
    Point2d(double x, double y);
    Point2d(double x, double y,int color);
    bool operator==(const Point2d& other);
    Point2d operator+(const Point2d& other);
    Point2d operator/(int a);
    bool IsNeededPoint();
};