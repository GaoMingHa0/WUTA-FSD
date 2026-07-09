#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
class MidPoint;
class GlobalVariables;
class Evaluation
{
public:
    Evaluation();
    void Clear();

    void Init(const std::vector<std::vector< std::shared_ptr<MidPoint>>>& midPointsList,const std::vector<std::shared_ptr<MidPoint>>& lastMidPoints, std::shared_ptr<MidPoint>& StartPoint);
    int Evaluate();

private:
    double DepthScore(int i);
    double DtwScore(int i);
    double WidthScore(int i);
    double CenterScore(int i);
    double AnglesStandardDeviationScore(int i);
    double LengthScore(int i);
    double AnglesScore(int i);
    double LengthStandardDeviationScore(int i);
//权重
    double dtwWeigth;
    double widthWeigth;
    double lengthWeigth;
    double anglesWeigth;
    double isFindStartPoint;
    double angleStandardDeviationWeight;
    double depthWeight;
    double centerWeight;
    double lengthStandardDeviationWeight;
    int minDepth;
    int maxDepth;

    struct Score{
        double dtwScore;
        double widthScore;
        double depthScore;
        double centerScore;
        double lengthScore;
        double lengthStandardDeviationScore;
        double angleScore;
        double anglesStandardDeviationScore;
        Score(){
            dtwScore = 10000000000;
            depthScore = 100000000;
            widthScore = 10000000;
            centerScore = 100000000000;
            lengthScore = 100000000000;
            lengthStandardDeviationScore = 1000000000000;
            angleScore = 1000000000000;
            anglesStandardDeviationScore = 10000000000000;
        }
    };

    double dtwMax;
    double lengthMax;
    double anglesStandardDeviationMax;
    double angleMax;
    double lengthStandardDeviationMax;
    double widthMax;

    double dtwMin;
    double lengthMin;
    double anglesStandardDeviationMin;
    double angleMin;
    double lengthStandardDeviationMin;
    double widthMin;
    std::vector<std::vector< std::shared_ptr<MidPoint>>> midPointsList;
    std::vector<std::shared_ptr<MidPoint>> lastMidPoints;
    std::shared_ptr<MidPoint> StartPoint;
    std::vector<Score> midPointsScore;

    GlobalVariables* single;
};
