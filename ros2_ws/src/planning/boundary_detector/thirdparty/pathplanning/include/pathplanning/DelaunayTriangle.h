#pragma once
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
class Triangle;
class Point2d;
class GlobalVariables;
class DelaunayTriangle
{
public:
    DelaunayTriangle();
    virtual std::vector<std::shared_ptr<Triangle>> Delaunay(std::vector<Point2d>& points) = 0;
    virtual void Clear() = 0;
protected:
    void IsNeededTraingle(std::vector<std::shared_ptr<Triangle>>& triangles);
    GlobalVariables* single;
    double triMaxLen;
    double triMinLen;
    double maxAngleTheshold;
    double minAngleTheshold;
};
