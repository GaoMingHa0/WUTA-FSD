#pragma once
#include "pathplanning/structs/Point2d.h"
class Vect
{
public:
    double x;
    double y;
    Vect(double x,double y);
    Vect();
     //计算两点间的线段向量
    Vect(Point2d start, Point2d end) ;
};