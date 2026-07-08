#include "pathplanning/structs/MidPoint.h"
#include "pathplanning/structs/Triangle.h"
#include "pathplanning/structs/Vect.h"
#include "pathplanning/Tool.h"
#include "pathplanning/PathSearch.h"
#include "pathplanning/Path.h"
#include "pathplanning/GlobalVariables.h"
#include <cmath>
#include <cassert>

Path::Path(){
    isSetStartPoint = false;
    isSetPoints = false;
    isSetPathSearchStartPoint = false;
    isLoop = false;
    isSetDirectionVector = false;
    startPoint = std::make_shared<MidPoint>();
    pathSearch = std::make_shared<PathSearch>();
    pathSearchStartPoint = std::make_shared<MidPoint>();
    single = GlobalVariables::getinstance();
    splineSize = single->GetSplineSize();
    startPointRedundancy = single->GetStartPointRedundancy();
    isPlanningPathLoop = false;
}
void Path::SetStartP(double x, double y) {
    if (isSetStartPoint) {
        std::cout << "Path : already set start point" << std::endl;
        return;
    }
    startPoint->mid.x = x;
    startPoint->mid.y = y;
    isSetStartPoint = true;
}
void Path::SetStartP(Point2d p)
{
    if (isSetStartPoint) {
        std::cout << "Path : already set start point" << std::endl;
        return;
    }
    startPoint->mid.x = p.x;
    startPoint->mid.y = p.y;
    isSetStartPoint = true;
}
void Path::IsLoop(Point2d Point, double Redundancy) {
    if (TOOL::Len(startPoint->mid, Point) <= Redundancy) {
        isLoop = true;
    }
}
void Path::SetPoints(std::vector<Point2d>& points) {
    if (isSetPoints) {
        std::cout << "Path : already set points" << std::endl;
        return;
    }
    isSetPoints = true;
    pathSearch->SetPoints(points);

    //pathSearch->SetLastMidps(lastMidps);
}
void Path::SetPathSearchStartPoint(double x, double y)
{
    if (isSetPathSearchStartPoint) {
        std::cout << "Path : already set path search start point" << std::endl;
        return;
    }
    pathSearchStartPoint->mid.x = x;
    pathSearchStartPoint->mid.y = y;
    pathSearch->SetStartPoint(pathSearchStartPoint);
}
void Path::SetPathSearchStartPoint(Point2d p){
    if (isSetPathSearchStartPoint) {
        std::cout << "Path : already set path search start point" << std::endl;
        return;
    }
    pathSearchStartPoint->mid = p;
    isSetPathSearchStartPoint = true;
    //pathSearch->SetLastMidps(lastMidps);
    pathSearch->SetStartPoint(pathSearchStartPoint);
}
void Path::Clear()
{
    pathSearch->Clear();
    isSetPoints = false;
    isSetDirectionVector = false;
    isSetPathSearchStartPoint = false;
}
bool Path::GetIsLoop()
{
    return isLoop;
}
void Path::SetDirectionVector(double x, double y)
{
    if (isSetDirectionVector) {
        std::cout << "Path : already set direction vector" << std::endl;
        return;
    }
    isSetDirectionVector = true;
    pathSearch->SetFormer(Vect(x,y));
}
void Path::SetDirectionVector(double yaw)
{
    if (isSetDirectionVector) {
        std::cout << "Path : already set direction vector" << std::endl;
        return;
    }
    double x = cos(yaw);
    double y = sin(yaw);
    this->SetDirectionVector(x, y);
}
void Path::Init(){
    if (!isSetPoints) {
        std::cout << "Path : not set points" << std::endl;
        assert(!isSetPoints);
    }
    if (!isSetStartPoint) {
        std::cout << "Path : not set start point" << std::endl;
        assert(!isSetStartPoint);
    }
    if (!isSetDirectionVector) {
        std::cout << "Path : not set direction vector" << std::endl;
        assert(!isSetDirectionVector);
    }
    if (!isSetPathSearchStartPoint) {
        std::cout << "Path : not set path search start point" << std::endl;
        assert(!isSetPathSearchStartPoint);
    }
    //pathSearch->Clear();
    pathSearch->SetPathStartPoint(startPoint);
}
std::vector<Point2d> Path::GetPathPoints() {
    std::vector<Point2d> pathSearchPoints;
    lastMidps.clear();
    lastMidps = pathSearch->GetBestMidps();
    int n = lastMidps.size();
    // for (int i = 0; i < n; i++) {
    //     std::cout << "lastMidps:"<<lastMidps[i]->mid.x << "," << lastMidps[i]->mid.y << std::endl;
    // }
    
    //std::cout << "size:" << n << std::endl;
    if (n > 1) {
        for(auto it : lastMidps) {
            IsPlanningPathLoop(it);
        }
        pathSearchPoints = TOOL::SplineWay(lastMidps, splineSize);
    }
    return pathSearchPoints;
}
std::vector<Point2d> Path::GetFinialPoints() {
    if (finalPoints.size() > 2) {
        return finalPoints;
    }
    std::cout << "Path:GetFinialPoints Error" << std::endl;
    return finalPoints;
}


bool Path::IsInit(){
    bool flag = false;
    // if(pathSearch->IsInit()){
    //     printf("pathsearch init\n");
    // }
    if(isSetStartPoint && isSetPathSearchStartPoint
    && isSetPoints && isSetDirectionVector&&pathSearch->IsInit()){
        flag = true;
    }
    // printf("isSetStartPoint:%d,isSetPathSearchStartPoint:%d,isSetPoints:%d,isSetDirectionVector:%d,pathSearch_init:%d\n",
    // isSetStartPoint,isSetPathSearchStartPoint,isSetPoints,
    // isSetDirectionVector,pathSearch->IsInit());
    return flag;
}
bool Path::GetIsSetStartPoint(){
    return isSetStartPoint;
}
void Path::PushPoints(Point2d p) {
    bool flag = true;
    if(finalPoints.size() < 1){
        finalPoints.push_back(p);
        return;
    }
    Point2d temp = finalPoints.back();
    if(TOOL::Len(temp,p) < 0.1){
        flag = false;
    }
    if(flag){
        finalPoints.push_back(p);
        if(!isLoop && finalPoints.size() > 50){
            IsLoop(p,startPointRedundancy);
        }
    }
}
Path::~Path() {
    delete single;
}
bool Path::GetIsPlanningPathLoop() {
    return isPlanningPathLoop;
}
void Path::IsPlanningPathLoop(std::shared_ptr<MidPoint> p) {
    if (TOOL::Len(p->mid,startPoint->mid) < 0.1){
        isPlanningPathLoop = true;
    }
}