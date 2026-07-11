#include "pathplanning/structs/MidPoint.h"
#include "pathplanning/structs/Triangle.h"
#include "pathplanning/structs/Vect.h"
#include "pathplanning/BowyerWatson.h"
#include "pathplanning/Evaluation.h"
#include "pathplanning/Tool.h"
#include "pathplanning/PathSearch.h"
#include "pathplanning/GlobalVariables.h"


PathSearch::PathSearch() :isSetStartPoint(false), isSetPoints(false),
                          isSetPathStartPoint(false), isSetLastMidps(false), isSetFormer(false) {
    delaunay = std::make_shared<BowyerWatson>();
    lastMidps.resize(0);
    evaluation = std::make_shared<Evaluation>();
    pathStartPoint = std::make_shared<MidPoint>();
    startMidPoint = std::make_shared<MidPoint>();
    former = new Vect();
    single = GlobalVariables::getinstance();
    maxDepth = single->GetMaxDepth();
    depthStep = single->GetDepthStep();
    minDepth = single->GetMinDepth();
    angleThreshold = single->GetAngleThreshold();
}

void PathSearch::SetStartPoint(std::shared_ptr<MidPoint> start) {
    //std::cout<<lastMidps.size()<<std::endl;
    if (!isSetPoints) {
        std::cout << "PathSearch_SetStartPoint:no set points" << std::endl;
        return;
    }
    if (!isSetFormer) {
        std::cout << "PathSearch_SetStartPoint:no set former" << std::endl;
        return;
    }
    bool flag = false;
//    if(lastMidps.size() != 0){
//        double minD = 10000;
//        double minAngle = 10000;
//        for(int i = 0 ; i < lastMidps.size() ; i++){
//            double dis = TOOL::Len(start->mid,lastMidps[i]->mid);
//            double angle = abs(TOOL::Angle(former,Vect(start->mid,midPoints[i]->mid)));
//            if(angle > 45){
//                continue;
//            }
//            if(dis < minD){
//                minD = dis;
//                startMidPoint = lastMidps[i];
//                flag = true;
//                isSetStartPoint = true;
//            }
//        }
//    }else{
//        std::cout<<"PathSearch-SetStartPoint:no use lastmidps!"<<std::endl;
//    }
    if(flag)
        return;
    double minDis = 100000;
    for (int i = 0; i < midPoints.size(); i++){
        double dis = TOOL::Len(start->mid,midPoints[i]->mid);
        double angle = abs(TOOL::Angle(*former,Vect(start->mid,midPoints[i]->mid)));
        if(angle > 40){
            continue;
        }
        if(dis < minDis){
            minDis = dis;
            startMidPoint = midPoints[i];
        }
    }
    //former = Vect(start->mid,startMidPoint->mid);
    isSetStartPoint = true;
    return;
    // auto x = former.x;
    // auto y = former.y;
    // auto new_x = x/sqrt(x*x+y*y);
    // auto new_y = y/sqrt(x*x+y*y);
    // printf("new former:x:%lf\ty:%lf\n",new_x,new_y);
    // for (int i = 0; i < triangles.size(); i++) {
    //     int flag = TOOL::IsPointInTriangle(start, triangles[i]);
    //     if(flag == -1){
    //         continue;
    //     }
    //     else if (flag == 1) {
    //         std::vector<double> angles = { TOOL::Angle(former, Vect(start->mid, triangles[i]->tri2midp[0]->mid)),
    //          TOOL::Angle(former, Vect(start->mid, triangles[i]->tri2midp[1]->mid)),
    //          TOOL::Angle(former, Vect(start->mid, triangles[i]->tri2midp[2]->mid)) };
    //         int res = 0;
    //         double min = 1000;
    //         for (int j = 0; j < angles.size();j++) {
    //             if (min > angles[j]) {
    //                 min = angles[j];
    //                 res = j;
    //             }
    //        }
    //         triangles[i]->visited = true;
    //         startMidPoint = triangles[i]->tri2midp[res];

    //         former = Vect(start->mid, triangles[i]->tri2midp[res]->mid);
    //         isSetStartPoint = true;
    //         break;
    //     }else{
    //         int resLine = 0;
    //         for(int j = 0 ; j < 3 ; j++){
    //             if(TOOL::IsPointOnLine(start->mid, triangles[i]->tri2midp[j]->p1
    //             , triangles[i]->tri2midp[j]->p2)){
    //                 resLine = j;
    //                 break;
    //             }
    //         }
    //         double angleSum = 0;
    //         int res = 0;
    //         double angleMin = 10000;
    //         //暂定，后续可以尝试结合lastMidps;
    //         for(int j = 0 ; j < 3 ; j++){
    //             if(resLine != j){
    //                 Vect after = Vect(triangles[i]->tri2midp[resLine]->mid,triangles[i]->tri2midp[j]->mid);
    //                 double angle = TOOL::Angle(former, after);
    //                 angleSum += angle;
    //                 if(angleMin > angle){
    //                     angleMin = angle;
    //                     res = j;
    //                 }
    //             }
    //         }
    //         if(angleSum > 150){
    //             continue;
    //         }
    //         triangles[i]->visited = true;
    //         startMidPoint = triangles[i]->tri2midp[res];
    //         former = Vect(start->mid, triangles[i]->tri2midp[res]->mid);
    //         isSetStartPoint = true;
    //         break;
    //     }
    // }
    // if (!isSetStartPoint) {
    //     std::cout << "PathSearch:no find start point" << std::endl;
    //     double finalMinDis = 100000;
    //     for (int i = 0; i < triangles.size(); i++){
    //         double minDis = 10000;
    //         int res = 0;
    //         for(int j = 0 ; j < 3;j++){
    //             double dis = TOOL::Len(start->mid,triangles[i]->tri2midp[j]->mid);
    //             if(TOOL::Angle(former,Vect(start->mid,triangles[i]->tri2midp[j]->mid)) > 0.7){
    //                 continue;
    //             }
    //             if(dis < minDis){
    //                 minDis = dis;
    //                 res = j;
    //             }
    //         }
    //         if(minDis < finalMinDis){
    //             finalMinDis = minDis;
    //             startMidPoint = triangles[i]->tri2midp[res];
    //         }
    //     }
    //     isSetStartPoint = true;
    // }
}

void PathSearch::SetPoints(std::vector<Point2d> points_arg)
{
    if (isSetPoints)
        return;
    isSetPoints = true;
    this->points = points_arg;
    triangles = delaunay->Delaunay(points_arg);
    Point2each();
    DelMidp();
    //std::cout << "PathSearch:already create" << std::endl;
}
void PathSearch::Clear()
{
    //printf("clear\n");
    midPoints.clear();
    points.clear();
    triangles.clear();
    dfsMidps.clear();
    isSetStartPoint = false;
    isSetPoints = false;
    //isSetPathStartPoint = false;
    isSetLastMidps = false;
    isSetFormer = false;
    midpsList.clear();
    //lastMidps.clear();
    delaunay->Clear();
}
std::vector<std::shared_ptr<MidPoint>> PathSearch::GetBestMidps() {
    if (!isSetStartPoint) {
        std::cout << "PathSearch : not set start point" << std::endl;
    }
    if (!isSetPoints) {
        std::cout << "PathSearch : not set points" << std::endl;
    }
    if (!isSetPathStartPoint) {
        std::cout << "PathSearch : not set path start point" << std::endl;
    }
    if (!isSetFormer) {
        std::cout << "PathSearch : not set direct" << std::endl;
    }
    //dfsMidps.emplace_back(startMidPoint);
    Dfs(startMidPoint,*former ,dfsMidps, 0,true);
    int n = midpsList.size();
    if (n == 0) {
        return lastMidps;
    }

    if (n == 1) {
        lastMidps = midpsList.front();
        midpsList.clear();
        return lastMidps;
    }

    evaluation->Clear();
    evaluation->Init(midpsList,lastMidps,pathStartPoint);
    int res = evaluation->Evaluate();
    auto bestMidps = midpsList[res];
    lastMidps = bestMidps;
    // if(BestMidps.size()>10){
    // printf("startMidPoint:%lf,%lf\n", startMidPoint->mid.x, startMidPoint->mid.y);
    // for (int i = 0; i < BestMidps.size(); i++) {


    //     printf("bestmidps:%lf,%lf\n", BestMidps[i]->mid.x, BestMidps[i]->mid.y);
    // }}
    //printf("bestmidpssize:%d\n",BestMidps.size());
    midpsList.clear();
    return bestMidps;
}

void PathSearch::Dfs(std::shared_ptr<MidPoint> Startp, Vect former, std::vector<std::shared_ptr<MidPoint>>& _midps, int depth,bool f) {
    if (depth == maxDepth) {
        midpsList.emplace_back(_midps);
        return;
    }
    //深度搜索
    bool flag = false;
    bool flag_two = false;
    for (int i = 0; i < 2; i++) {
        if (Startp->midp2tri[i] != nullptr && Startp->midp2tri[i]->visited == false) {
            for (int j = 0; j < 3; j++) {
                if (Startp->midp2tri[i]->tri2midp[j]->visited == false) {
                    //递归
                    Vect next = Vect(Startp->mid, Startp->midp2tri[i]->tri2midp[j]->mid);
                    if(f){
                        if (abs(TOOL::Angle(former, next)) > angleThreshold) {
                            continue;
                        }
                    }
                    Startp->midp2tri[i]->visited = true;
                    Startp->midp2tri[i]->tri2midp[j]->visited = true;
                    _midps.emplace_back(Startp->midp2tri[i]->tri2midp[j]);
                    Dfs(Startp->midp2tri[i]->tri2midp[j],next, _midps, depth+1,true);
                    //回溯
                    Startp->midp2tri[i]->visited = false;
                    Startp->midp2tri[i]->tri2midp[j]->visited = false;
                    _midps.pop_back();
                    flag = true;
                }
            }
        }
    }
    if (!flag&& depth >= minDepth) {
        midpsList.emplace_back(_midps);
        flag_two = true;
    }
   if(!flag_two && depth>= minDepth && (depth - minDepth) % depthStep == 0){
       midpsList.emplace_back(_midps);
   }

    return;
}
void PathSearch::SetPathStartPoint(std::shared_ptr<MidPoint> PathStartPoint)
{
    isSetPathStartPoint = true;
    this->pathStartPoint = PathStartPoint;
}
void PathSearch::SetLastMidps(const std::vector<std::shared_ptr<MidPoint>>& LastMidps)
{
    isSetLastMidps = true;
    this->lastMidps = LastMidps;
}

void PathSearch::SetFormer(Vect p)
{
    former->x = p.x;
    former->y = p.y;
    isSetFormer = true;
}

void PathSearch::DelMidp() {
    for (auto it = midPoints.begin(); it != midPoints.end(); it++) {
        auto it2 = it;
        while (1) {
            advance(it2, 1);
            if (it2 != midPoints.end()) {
                if ((*it)->mid == (*it2)->mid) {
                    (*it)->midp2tri[1] = (*it2)->midp2tri[0];
                    for (auto& it_tri : (*it2)->midp2tri[0]->tri2midp) {
                        if ((*it2)->mid == it_tri->mid) {
                            it_tri = *it;
                            break;
                        }
                    }
                    it2 = midPoints.erase(it2);
                    break;
                }
            }
            else break;
        }
    }

}
void PathSearch::Point2each() {
    for (auto& it : triangles) {
        std::shared_ptr<MidPoint> midp1 = std::make_shared<MidPoint>();
        midp1->mid = (it->p1 + it->p2) / 2;
        midp1->midp2tri[0] = it;
        it->tri2midp[0] = midp1;
        midp1->width = TOOL::Len(it->p1, it->p2);
        midPoints.emplace_back(midp1);
        std::shared_ptr<MidPoint> midp2 = std::make_shared<MidPoint>();
        midp2->mid = (it->p1 + it->p3) / 2;
        midp2->midp2tri[0] = it;
        it->tri2midp[1] = midp2;
        midp2->width = TOOL::Len(it->p1, it->p3);
        midPoints.emplace_back(midp2);
        std::shared_ptr<MidPoint> midp3 = std::make_shared<MidPoint>();
        midp3->mid = (it->p3 + it->p2) / 2;
        midp3->midp2tri[0] = it;
        it->tri2midp[2] = midp3;
        midp3->width = TOOL::Len(it->p3, it->p2);
        midPoints.emplace_back(midp3);
    }
}

bool PathSearch::IsInit(){
    bool flag = false;
    if( isSetStartPoint&&
        isSetPoints&&
        isSetPathStartPoint&&
        isSetFormer){
        flag = true;
    }
    //printf("isSetStartPoint:%d,isSetPoints:%d,isSetPathStartPoint:%d,isSetLastMidps:%d,isSetFormer:%d\n",
    //isSetStartPoint,isSetPoints,isSetPathStartPoint,isSetLastMidps,isSetFormer);
    //printf("isSetStartPoint:%d,isSetPoints:%d,isSetFormer:%d\n",
    //isSetStartPoint,isSetPoints,isSetFormer);
    return flag;
}
