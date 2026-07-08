#pragma once
#include <memory>
#include "pathplanning/structs/Point2d.h"
#include "pathplanning/structs/MidPoint.h"
class PathSearch;
class GlobalVariables;
class Path {
public:
    Path();
    ~Path();

    std::vector<Point2d> GetPathPoints();
    std::vector<Point2d> GetFinialPoints();
    std::vector<Point2d> GetFinialPointsPro();
    void Init();
    //设置起点
    void SetStartP(double x, double y);
    void SetStartP(Point2d p);
    //传入点云
    void SetPoints(std::vector<Point2d>& points);
    //设置搜索起点
    void SetPathSearchStartPoint(double x, double y);
    void SetPathSearchStartPoint(Point2d p);
    //清除数据
    void Clear();
    //取出是否回环信息
    bool GetIsLoop();
    bool GetIsPlanningPathLoop();
    bool IsInit();
    bool GetIsSetStartPoint();
    //设置方向向量
    void SetDirectionVector(double x, double y);
    void SetDirectionVector(double yaw); //输入yaw角
    void PushPoints(Point2d p);
private:
    void IsLoop(Point2d Point, double Redundancy);
    void IsPlanningPathLoop(std::shared_ptr<MidPoint> p);
    std::shared_ptr<PathSearch> pathSearch;
    std::shared_ptr<MidPoint> startPoint;
    std::shared_ptr<MidPoint> pathSearchStartPoint;
    std::vector<Point2d> finalPoints;
    std::vector<Point2d> finalPointsPro;
    std::vector<std::shared_ptr<MidPoint>> lastMidps;
    bool isSetStartPoint;
    bool isLoop;
    bool isSetPathSearchStartPoint;
    bool isSetPoints;
    bool isSetDirectionVector;
    bool isPlanningPathLoop;
    int splineSize;
    double startPointRedundancy;
    GlobalVariables* single;
};