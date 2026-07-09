#pragma once
#include <map>
#include "DelaunayTriangle.h"

class BowyerWatson : public DelaunayTriangle
{
public:
    virtual std::vector<std::shared_ptr<Triangle>> Delaunay(std::vector<Point2d>& points);
    virtual void Clear();
    std::shared_ptr<Triangle> MaxTriangle(std::vector<Point2d>& points);
};