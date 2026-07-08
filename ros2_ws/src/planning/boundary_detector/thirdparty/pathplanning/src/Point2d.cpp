#include "pathplanning/structs/Point2d.h"
Point2d::Point2d() {
    x = 0;
    y = 0;
    color = -1;
}
Point2d::Point2d(double x, double y) : x(x), y(y) {
    color = -1;
}
Point2d::Point2d(double x, double y,int color) : x(x), y(y),color(color) {
}
bool Point2d::operator==(const Point2d& other) {
    return (this->x == other.x && this->y == other.y);
}
Point2d Point2d::operator+(const Point2d& other) {
    Point2d temp;
    temp.x = this->x + other.x;
    temp.y = this->y + other.y;
    return temp;
}
Point2d Point2d::operator/(int a) {
    Point2d temp;
    temp.x = this->x / a;
    temp.y = this->y / a;
    return temp;
}
bool Point2d::IsNeededPoint()
{
    bool flag = false;
    if (this->color != -1) {
        flag = true;
    }
    return flag;
}