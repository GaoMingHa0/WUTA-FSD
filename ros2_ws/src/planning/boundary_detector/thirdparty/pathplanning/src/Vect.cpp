#include "pathplanning/structs/Vect.h"

Vect::Vect(double x,double y):x(x),y(y){}
Vect::Vect():x(-10000),y(-10000){}
Vect::Vect(Point2d start, Point2d end) {
    x = end.x - start.x;
    y = end.y - start.y;
}