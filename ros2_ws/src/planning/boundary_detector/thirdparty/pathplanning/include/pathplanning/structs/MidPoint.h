#pragma once
#include <vector>
#include <memory>
#include "Point2d.h"

class Triangle;
class MidPoint {
public:
    Point2d mid;
    bool visited;
    std::vector <std::shared_ptr<Triangle>> midp2tri;
    double width;
    MidPoint();
};
