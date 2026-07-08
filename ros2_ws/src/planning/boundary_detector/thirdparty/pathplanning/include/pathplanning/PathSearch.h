#pragma once
#include <iostream>
#include <vector>
#include "pathplanning/structs/Point2d.h"
#include "pathplanning/structs/MidPoint.h"
#include "pathplanning/structs/Vect.h"
#include "pathplanning/structs/Triangle.h"
class Evaluation;
class DelaunayTriangle;
class GlobalVariables;
class PathSearch {
    std::vector<std::shared_ptr<MidPoint>> midPoints;
    std::vector<Point2d> points;
    std::vector<std::shared_ptr<Triangle>> triangles;
    std::shared_ptr<DelaunayTriangle> delaunay;
    //搜索起点
    std::shared_ptr<MidPoint> startPoint;
    std::shared_ptr<MidPoint> startMidPoint;
    //记录初始化是否完成
    bool isSetStartPoint;
    bool isSetPoints;
    bool isSetPathStartPoint;
    bool isSetLastMidps;
    bool isSetFormer;
    //记录搜索出的所有路径
    std::vector<std::vector< std::shared_ptr<MidPoint>>> midpsList;
    //存储上一次的路径
    std::vector<std::shared_ptr<MidPoint>> lastMidps;
    std::shared_ptr<Evaluation> evaluation;
    //存储路径起点
    std::shared_ptr<MidPoint> pathStartPoint;
    Vect* former;
    std::vector<std::shared_ptr<MidPoint>> dfsMidps;
public:
    PathSearch();
    //设置搜索起点
    void SetStartPoint(std::shared_ptr<MidPoint> start);
    //初始化数据
    void SetPoints(std::vector<Point2d> points_arg);
    //清除数据
    void Clear();
    //得到最优路径
    std::vector<std::shared_ptr<MidPoint>> GetBestMidps();
    //设置路径起点
    void SetPathStartPoint(std::shared_ptr<MidPoint> PathStartPoint);
    //设置上一次的路径
    void SetLastMidps(const std::vector<std::shared_ptr<MidPoint>>& LastMidps);
    void SetFormer(Vect p);
    bool IsInit();
private:
    void Dfs(std::shared_ptr<MidPoint> Startp, Vect former, std::vector<std::shared_ptr<MidPoint>>& _midps, int depth,bool f);
    void DelMidp();
    void Point2each();

    int maxDepth;
    int depthStep;
    int minDepth;
    double angleThreshold;
    GlobalVariables* single;

};
