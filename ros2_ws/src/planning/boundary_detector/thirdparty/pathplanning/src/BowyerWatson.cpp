#include "pathplanning/structs/Triangle.h"
#include "pathplanning/structs/Line.h"
#include "pathplanning/Tool.h"
#include "pathplanning/BowyerWatson.h"
std::vector<std::shared_ptr<Triangle>> BowyerWatson::Delaunay(std::vector<Point2d>& points) {
    std::vector<std::shared_ptr<Triangle>> triangles;
    std::vector<std::shared_ptr<Triangle>> temp_triangles;
    std::map<int, Line> Lines;
    std::vector<int> delLines;
    auto maxTriangle = MaxTriangle(points);
    temp_triangles.emplace_back(maxTriangle);
    triangles.emplace_back(maxTriangle);
    int cont = 0;
    sort(points.begin(), points.end(), [](Point2d& a, Point2d& b) ->bool {
        if (a.x == b.x) {
            return a.y < b.y;
        }
        else {
            return a.x < b.x;
        }
       });
    for (auto it_point = points.begin(); it_point != points.end(); it_point++)
    {
        delLines.clear();
        Lines.clear();
        //遍历temp_triangles
        for (auto it_temp_triangles = temp_triangles.begin(); it_temp_triangles != temp_triangles.end(); )
        {
            double length = sqrt(pow((*it_temp_triangles)->center.x - it_point->x, 2)
                + pow((*it_temp_triangles)->center.y - it_point->y, 2));
            if (it_point->x - (*it_temp_triangles)->center.x > (*it_temp_triangles)->r) {
                triangles.emplace_back(new Triangle((*it_temp_triangles)->p1, (*it_temp_triangles)->p2
                    , (*it_temp_triangles)->p3));
                it_temp_triangles = temp_triangles.erase(it_temp_triangles);
            }
            else if (length <= (*it_temp_triangles)->r) {
                Lines[cont++] = Line((*it_temp_triangles)->p1, (*it_temp_triangles)->p2);
                Lines[cont++] = Line((*it_temp_triangles)->p3, (*it_temp_triangles)->p2);
                Lines[cont++] = Line((*it_temp_triangles)->p1, (*it_temp_triangles)->p3);
                it_temp_triangles = temp_triangles.erase(it_temp_triangles);
            }
            else ++it_temp_triangles;
        }
        //去重边，并生成三角形
        for (auto it = Lines.begin(); it != Lines.end(); it++) {
            auto it2 = it;
            while (1) {
                advance(it2, 1);
                if (it2 != Lines.end()) {
                    if (it->second == it2->second) {
                        delLines.emplace_back(it->first);
                        delLines.emplace_back(it2->first);
                        break;
                    }
                }
                else break;
            }
        }
        sort(delLines.begin(), delLines.end());
        auto del = unique(delLines.begin(), delLines.end());
        delLines.erase(del, delLines.end());
        for (auto it : delLines) {
            Lines.erase(it);
        }
        for (auto& it : Lines)
        {
            temp_triangles.emplace_back(new Triangle(
                Point2d(it_point->x, it_point->y),
                Point2d(it.second.p1.x, it.second.p1.y),
                Point2d(it.second.p2.x, it.second.p2.y)
            ));

        }
    }
    //合并到triangles中
    triangles.insert(triangles.end(), temp_triangles.begin(), temp_triangles.end());
    //去除与超级三角形有关的三角形
    for (auto it_triangles = triangles.begin(); it_triangles != triangles.end();) {
        if ((*it_triangles)->p1 == maxTriangle->p1 || (*it_triangles)->p1 == maxTriangle->p2
            || (*it_triangles)->p1 == maxTriangle->p3 || (*it_triangles)->p2 == maxTriangle->p1
            || (*it_triangles)->p2 == maxTriangle->p2 || (*it_triangles)->p2 == maxTriangle->p3
            || (*it_triangles)->p3 == maxTriangle->p1 || (*it_triangles)->p3 == maxTriangle->p2
            || (*it_triangles)->p3 == maxTriangle->p3) {
            it_triangles = triangles.erase(it_triangles);
        }
        else ++it_triangles;
    }
    IsNeededTraingle(triangles);
    return triangles;
}

void BowyerWatson::Clear()
{
    //temp_triangles.clear();
}

std::shared_ptr<Triangle> BowyerWatson::MaxTriangle(std::vector<Point2d>& points)
{
    double rect_x_min, rect_x_max, rect_y_min, rect_y_max;
    sort(points.begin(), points.end(), [](Point2d& a, Point2d& b)->bool {
        return a.y < b.y;
        });
    rect_y_min = points.begin()->y;
    rect_y_max = (points.end() - 1)->y;
    sort(points.begin(), points.end(), [](Point2d& a, Point2d& b)->bool {
        return a.x < b.x;
        }); //注意排序先后，当前为按x从小到大排序
    rect_x_min = points.begin()->x;
    rect_x_max = (points.end() - 1)->x;

    double m;
    Point2d center;
    if (rect_y_max - rect_y_min > rect_x_max - rect_x_min)
        m = rect_y_max - rect_y_min;
    else
        m = rect_x_max - rect_x_min;
    center.x = (rect_x_max + rect_x_min) / 2;
    center.y = (rect_y_max + rect_y_min) / 2;
    m *= 1.1;

    Point2d left, top, right;
    left.x = center.x - m;
    left.y = center.y - m / 2;
    top.x = center.x;
    top.y = center.y + 1.5 * m;
    right.x = center.x + m;
    right.y = center.y - m / 2;
    return  std::make_shared<Triangle>(left, top, right);

}