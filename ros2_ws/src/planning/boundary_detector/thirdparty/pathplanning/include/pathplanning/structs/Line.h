#pragma once
#include "pathplanning/structs/Point2d.h"
class Line {
public:
    Point2d p1, p2;
    Line();
    Line(Point2d a, Point2d b);
    bool operator==(const Line& other);
};