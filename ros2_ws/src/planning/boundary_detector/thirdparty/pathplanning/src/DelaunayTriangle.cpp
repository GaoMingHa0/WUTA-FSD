#include "pathplanning/structs/Triangle.h"
#include "pathplanning/Tool.h"
#include "pathplanning/GlobalVariables.h"
#include "pathplanning/DelaunayTriangle.h"

DelaunayTriangle::DelaunayTriangle() {
    single = GlobalVariables::getinstance();
    triMaxLen = single->GetTriMaxLen();
    triMinLen = single->GetTriMinLen();
    maxAngleTheshold = single->GetMaxAngleTheshold();
    minAngleTheshold = single->GetMinAngleTheshold();
}
void DelaunayTriangle::IsNeededTraingle(std::vector<std::shared_ptr<Triangle>>& triangles)
{
    double L[3];
    double angle[3];
    for (auto it = triangles.begin(); it != triangles.end();) {
        L[0] = TOOL::Len((*it)->p1, (*it)->p2);
        L[1] = TOOL::Len((*it)->p3, (*it)->p2);
        L[2] = TOOL::Len((*it)->p1, (*it)->p3);
        std::sort(L, L + 3);
        if (L[2] > triMaxLen) {
            triangles.erase(it);
            continue;
        }
        if (L[0] < triMinLen) {
            triangles.erase(it);
            continue;
        }
        // if (L[0] < TRIMINLEN || L[1] < TRIMINLEN || L[2] < TRIMINLEN) {
        //     triangles.erase(it);
        //     continue;
        // }
        //设置最小角阈值
        angle[0] = acos((L[1] * L[1] + L[2] * L[2] - L[0] * L[0]) / (2 * L[1] * L[2])) * 180 / M_PI;
        /*angle[1] = acos((L[0] * L[0] + L[2] * L[2] - L[1] * L[1]) / (2 * L[0] * L[2]));
        angle[2] = acos((L[1] * L[1] + L[0] * L[0] - L[2] * L[2]) / (2 * L[1] * L[0]));*/
        // if (angle[0] < MINANGLETHRESHOLD) {
        //     triangles.erase(it);
        //     continue;
        // }
        angle[2] = acos((L[1] * L[1] + L[0] * L[0] - L[2] * L[2]) / (2 * L[1] * L[0])) * 180 / M_PI;
        //double minAngleCos = (L[1] * L[1] + L[2] * L[2] - L[0] * L[0]) / (2 * L[1] * L[2]);
        //double maxAngleCos = (L[1] * L[1] + L[0] * L[0] - L[2] * L[2]) / (2 * L[1] * L[0]);

        //设置最大角阈值
        //老程序数据
        if (angle[2] > maxAngleTheshold) {
            triangles.erase(it);
            continue;
        }

        if(angle[0] < minAngleTheshold){
            triangles.erase(it);
            continue;
        }
        it++;
    }
    //printf("tris:%d\n",triangles.size());
}
