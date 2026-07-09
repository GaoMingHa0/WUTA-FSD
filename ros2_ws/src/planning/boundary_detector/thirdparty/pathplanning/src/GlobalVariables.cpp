#include "hpl/hpl_param.hpp"
#include "hpl/hpl_log.hpp"
#include "pathplanning/GlobalVariables.h"


GlobalVariables::GlobalVariables() {
    hpl::ParamNode param_node{"config"};
    param_node.declParam(MaxDepth, "MaxDepth");
    param_node.declParam(DepthStep, "DepthStep");
    param_node.declParam(MinDepth, "MinDepth");
    param_node.declParam(TriMaxLen, "TriMaxLen");
    param_node.declParam(TriMinLen, "TriMinLen");
    param_node.declParam(MinAngleTheshold, "MinAngleTheshold");
    param_node.declParam(MaxAngleTheshold, "MaxAngleTheshold");
    param_node.declParam(IsFindStartPoint, "IsFindStartPoint");
    param_node.declParam(CenterWeight, "CenterWeight");
    param_node.declParam(DepthWeight, "DepthWeight");
    param_node.declParam(DtwWeight, "DtwWeight");
    param_node.declParam(WidthWeigth, "WidthWeigth");
    param_node.declParam(LengthWeight, "LengthWeight");
    param_node.declParam(AngleWeight, "AngleWeight");
    param_node.declParam(AngleStandardDeviationWeight, "AngleStandardDeviationWeight");
    param_node.declParam(LengthStandardDeviationWeight, "LengthStandardDeviationWeight");
    param_node.declParam(AngleThreshold, "AngleThreshold");
    param_node.declParam(SplineSize, "SplineSize");
    param_node.declParam(Redundancy, "Redundancy");
    param_node.declParam(StartPointRedundancy, "StartPointRedundancy");
    param_node.declParam(PlugPointSize, "PlugPointSize");
    param_node.declParam(DistanceToBoundary, "DistanceToBoundary");
    param_node.declParam(IsUseFirstLapPoints, "IsUseFirstLapPoints");
    param_node.declParam(IsUseFirstLapPointsPro,"IsUseFirstLapPointsPro");


    CLOG("MaxDepth:%d",MaxDepth);
    CLOG("DepthStep:%d",DepthStep);
    CLOG("MinDepth:%d",MinDepth);
    CLOG("TriMaxLen:%lf",TriMaxLen);
    CLOG("TriMinLen:%lf",TriMinLen);
    CLOG("MinAngleTheshold:%lf",MinAngleTheshold);
    CLOG("MaxAngleTheshold:%lf",MaxAngleTheshold);
    CLOG("IsFindStartPoint:%lf",IsFindStartPoint);
    CLOG("CenterWeight:%lf",CenterWeight);
    CLOG("DepthWeight:%lf",DepthWeight);
    CLOG("DtwWeight:%lf",DtwWeight);
    CLOG("WidthWeigth:%lf",WidthWeigth);
    CLOG("LengthWeight:%lf",LengthWeight);
    CLOG("AngleWeight:%lf",AngleWeight);
    CLOG("AngleStandardDeviationWeight:%lf",AngleStandardDeviationWeight);
    CLOG("LengthStandardDeviationWeight:%lf",LengthStandardDeviationWeight);
    CLOG("AngleThreshold:%lf",AngleThreshold);
    CLOG("SplineSize:%d",SplineSize);
    CLOG("Redundancy:%lf",Redundancy);
    CLOG("StartPointRedundancy:%lf",StartPointRedundancy);
    CLOG("PlugPointSize:%d",PlugPointSize);
    CLOG("DistanceToBoundary:%lf",DistanceToBoundary);
    CLOG("IsUseFirstLapPoints:%d",IsUseFirstLapPoints);
    CLOG("IsUseFirstLapPointsPro:%d",IsUseFirstLapPointsPro);
}
