#pragma once
#include <vector>
#include <memory>
#include "pathplanning/structs/Point2d.h"
#include "pathplanning/structs/MidPoint.h"
#include "pathplanning/structs/Vect.h"
#include "pathplanning/structs/Triangle.h"
namespace TOOL {
    // 计算向量 p1p2 和 p1p3 的叉乘
    bool IsPointOnLine(Point2d startPoint, Point2d lineP1 , Point2d lineP2) ;
    double CrossProduct(const Point2d& tri_p1, const Point2d& tri_p2, const Point2d& target_p) ;
    int IsPointInTriangle(std::shared_ptr<MidPoint> start, std::shared_ptr<Triangle> triangle);
    double Len(Point2d a, Point2d b) ;
    double LineLen(const std::vector<std::shared_ptr<MidPoint>>& points);
    //两个向量间的角度
    double Angle(Vect a, Vect b) ;
    std::vector<Point2d>  SplineWay(const std::vector<std::shared_ptr<MidPoint>>& midps, double size) ;
    std::vector<Point2d>  SplineWay(const std::vector<Point2d>& points, double size) ;

    // 三点法求曲率
    double GetCurvature(Point2d a, Point2d b, Point2d c);
    
     // 计算两个向量的点积
    double VectDotProduct(Vect a, Vect b);
   
     // 计算角abc角平分线的向量
    Vect angleBisector(Point2d a, Point2d b, Point2d c);
    // 角平分线的垂直线，且向量与bc的夹角为锐角
    Vect verticalOfBisector (Point2d a, Point2d b, Point2d c);
     // 向量与x轴正方向的夹角
    double vectToAnglex(Vect vec);
  
    double GetHeading(Point2d a, Point2d b, Point2d c);

    std::vector<Point2d> Transeform(const std::vector<std::shared_ptr<MidPoint>>& midps);
    void CyclicCopy(const std::vector<Point2d>& source, std::vector<Point2d>& destination, int start, int length);
}
