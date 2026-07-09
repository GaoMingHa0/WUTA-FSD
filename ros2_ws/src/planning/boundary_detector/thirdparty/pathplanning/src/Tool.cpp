#include "pathplanning/structs/MidPoint.h"
#include "pathplanning/structs/Triangle.h"
#include "pathplanning/structs/Vect.h"
#include "pathplanning/Spline.h"
#include "pathplanning/GlobalVariables.h"
#include "pathplanning/Tool.h"
bool TOOL::IsPointOnLine(Point2d startPoint, Point2d lineP1 , Point2d lineP2) {
    double crossProduct = (startPoint.y - lineP1.y) * (lineP2.x - lineP1.x) - (startPoint.x - lineP1.x) * (lineP2.y - lineP1.y);
    return crossProduct == 0;
}
double TOOL::CrossProduct(const Point2d& tri_p1, const Point2d& tri_p2, const Point2d& target_p) {
    return (tri_p2.x - tri_p1.x) * (target_p.y - tri_p1.y) - (tri_p2.y - tri_p1.y) * (target_p.x - tri_p1.x);
}

int TOOL::IsPointInTriangle(std::shared_ptr<MidPoint> start, std::shared_ptr<Triangle> triangle)
{
    int res = -1;
    // 计算叉乘
    double cp1 = CrossProduct(triangle->p1, triangle->p2, start->mid);
    double cp2 = CrossProduct(triangle->p2, triangle->p3, start->mid);
    double cp3 = CrossProduct(triangle->p3, triangle->p1, start->mid);

    // 判断叉乘结果的符号，如果三个结果符号相同，则点在三角形内
    if (cp1 > 0 && cp2 > 0 && cp3 > 0) res = 1;
    if (cp1 < 0 && cp2 < 0 && cp3 < 0) res = 1;
    //全为0在边上
    if (cp1 == 0 && cp2 == 0 && cp3 == 0) res = 0;
    // 点在三角形外部
    return res;
}

double TOOL::Len(Point2d a, Point2d b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

double TOOL::LineLen(const std::vector<std::shared_ptr<MidPoint>>& points)
{
    double all = 0;
    for (int i = 0; i < points.size() - 1; i++) {
        all += Len(points[i]->mid, points[i + 1]->mid);
    }
    return all;
}

double TOOL::Angle(Vect a, Vect b) {
    double angleA = atan2(a.y, a.x); // 计算向量a的角度
    double angleB = atan2(b.y, b.x); // 计算向量b的角度
    double diff = angleB - angleA;   // 计算角度差
    // 将角度差转换为 -180 到 180 度的范围
    if (diff > PI) {
        diff -= 2 * PI;
    }
    else if (diff < -PI) {
        diff += 2 * PI;
    }

    return diff * 180.0 / PI; // 返回角度差（弧度）
}

std::vector<Point2d>  TOOL::SplineWay(const std::vector<std::shared_ptr<MidPoint>>& midps, double size) {
    //std::cout << "midps.size:" << midps.size() << std::endl;
    std::vector<Point2d> res;
    int n = midps.size();
    std::vector<double> t(n), xlist(n), ylist(n);
    for (int i = 0; i < n; ++i) {
        t[i] = i;
        xlist[i] = midps[i]->mid.x;
        ylist[i] = midps[i]->mid.y;
    }
    tk::spline x_spline, y_spline;
    x_spline.set_points(t, xlist);
    y_spline.set_points(t, ylist);
    /* for (int i = 0; i < n - 1; ++i) {
         double length = Len(midps[i]->mid, midps[i + 1]->mid);
         std::cout << midps[i]->mid.x << "," << midps[i]->mid.y << std::endl;
         std::cout << midps[i+1]->mid.x << "," << midps[i+1]->mid.y << std::endl;
         double Slice = 1.0 / (length / size);
         std::cout << "length:" << length << std::endl;
         std::cout << "Slice:" << Slice << std::endl;
         for (double start = t[i]; start < t[i + 1]; start += Slice) {
             Point2d temp(x_spline(start), y_spline(start));
             res.emplace_back(temp);
         }
     }*/
    double step = 1.0 / size;
    for (int i = 0; i < n - 1; ++i) {
        for (double start = t[i]; start < t[i + 1]; start += step) {
            Point2d temp(x_spline(start), y_spline(start));
            res.emplace_back(temp);
        }
    }
    //std::cout << "res:" << res.size() << std::endl;
    return res;
}

std::vector<Point2d>  TOOL::SplineWay(const std::vector<Point2d>& points, double size) {
    std::vector<Point2d> res;
    int n = points.size();
    std::vector<double> t(n), xlist(n), ylist(n);
    for (int i = 0; i < n; ++i) {
        t[i] = i;
        xlist[i] = points[i].x;
        ylist[i] = points[i].y;
    }
    tk::spline x_spline, y_spline;
    x_spline.set_points(t, xlist);
    y_spline.set_points(t, ylist);
    double step = 1.0 / size;
    for (int i = 0; i < n - 1; ++i) {
        for (double start = t[i]; start < t[i + 1]; start += step) {
            Point2d temp(x_spline(start), y_spline(start));
            res.emplace_back(temp);
        }
    }
    return res;
}

double TOOL::GetCurvature(Point2d a, Point2d b, Point2d c)
{
    if ((fabs(a.x-b.x)<EPS&&fabs(a.x-c.x)<EPS)||(fabs(a.y-b.y)<EPS&&fabs(a.y-c.y)<EPS))
        return 0;
    double curvity;
    double dis1, dis2, dis3;

    double cosB, sinB, dis;
    dis1 = Len(a, b);
    dis2 = Len(a, c);
    dis3 = Len(b, c);
    //余弦定理
    dis = dis1 * dis1 + dis3 * dis3 - dis2 * dis2;
    cosB = dis / (2 * dis1 * dis3);
    sinB = sqrt(1 - cosB * cosB);
    curvity = 0.5 * dis2 / sinB;
    curvity = 1 / curvity;
    return curvity;
}

double TOOL::VectDotProduct(Vect a, Vect b)
{
    return a.x * b.x + a.y * b.y;
}
Vect TOOL::angleBisector(Point2d a, Point2d b, Point2d c) {
    Vect res;
    Vect ba = Vect(b, a);
    Vect bc = Vect(b, c);
    res = Vect(ba.x + bc.x, ba.y + bc.x);
    return res;
}
Vect TOOL::verticalOfBisector(Point2d a, Point2d b, Point2d c)
{
    Vect bisector = angleBisector(a, b, c);
    Vect res;
    if (bisector.x == 0)
    {
        res.y = 1;
        if (VectDotProduct(res, bisector) < 0)
        {
            res.y = -1;
        }
    }
    else if (bisector.y == 0)
    {
        res.x = 1;
        if (VectDotProduct(res, bisector) < 0)
        {
            res.x = -1;
        }
    }
    else
    {
        res.x = 1;
        res.y = -bisector.x / bisector.y;
        if (VectDotProduct(res, bisector) < 0)
        {
            res.x = -1;
            res.y = bisector.x / bisector.y;
        }
    }
    return res;
}
double TOOL::vectToAnglex(Vect vec)
{
    double ang = atan2(vec.y, vec.x) * 180.0 / PI;
    return ang;
}

double TOOL::GetHeading(Point2d a, Point2d b, Point2d c)
{
    Vect temp = verticalOfBisector(a, b, c);
    double x = vectToAnglex(temp);
    return x;
}

std::vector<Point2d> TOOL::Transeform(const std::vector<std::shared_ptr<MidPoint>>& midps)
{
    std::vector<Point2d> res;
    for(auto it : midps){
        res.push_back(it->mid);
    }
    return res;
}
void TOOL::CyclicCopy(const std::vector<Point2d>& source, std::vector<Point2d>& destination, int start, int length){
    destination.clear();
    int n = source.size();

    if (n == 0) return;

    for (int i = 0; i < length; ++i) {
        destination.push_back(source[(start + i) % n]);
    }
}

   