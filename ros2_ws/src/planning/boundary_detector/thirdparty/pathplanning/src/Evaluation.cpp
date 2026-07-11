#include "pathplanning/structs/Vect.h"
#include "pathplanning/Evaluation.h"
#include "pathplanning/structs/MidPoint.h"
#include "pathplanning/Tool.h"
#include "pathplanning/GlobalVariables.h"
GlobalVariables* GlobalVariables::instance = nullptr;
Evaluation::Evaluation() {
    single = GlobalVariables::getinstance();
    depthWeight = single->GetDepthWeight();
    widthWeigth = single->GetWidthWeigth();
    lengthWeigth = single->GetLengthWeight();
    dtwWeigth = single->GetDtwWeight();
    anglesWeigth = single->GetAngleWeight();
    isFindStartPoint = single->GetIsFindStartPoint();
    angleStandardDeviationWeight = single->GetAngleStandardDeviationWeight();
    centerWeight = single->GetCenterWeight();
    lengthStandardDeviationWeight = single->GetLengthStandardDeviationWeight();
    minDepth = single->GetMinDepth();
    maxDepth = single->GetMaxDepth();

    dtwMax = -1;
    lengthMax = -1;
    anglesStandardDeviationMax = -1;
    angleMax = -1;
    lengthStandardDeviationMax = -1;
    widthMax = -1;

    dtwMin = 100000000000;
    lengthMin = 100000000000;
    anglesStandardDeviationMin = 1000000000000;
    angleMin = 1000000000000;
    lengthStandardDeviationMin = 10000000000000;
    widthMin = 100000000000000;
}
double Evaluation::DepthScore(int i){
    int n = midPointsList[i].size();
    double score = 1- (n - minDepth)/(maxDepth - minDepth);
    return score;
}
double Evaluation::DtwScore(int i) {
    auto mps1 = midPointsList[i];
    auto mps2 = lastMidPoints;
    int m = mps1.size();
    int n = mps2.size();
    if (n == 0)return 0;
    std::vector<std::vector<double>> dp(m, std::vector<double>(n, 0.0));
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            dp[i][j] = TOOL::Len(mps1[i]->mid, mps2[j]->mid);
        }
    }
    for (int i = 1; i < m; i++) {
        dp[i][0] += dp[i - 1][0];
    }
    for (int i = 1; i < n; i++) {
        dp[0][i] += dp[0][i - 1];
    }
    for (int i = 1; i < m; i++) {
        for (int j = 1; j < n; j++) {
            dp[i][j] += std::min(dp[i - 1][j - 1], std::min(dp[i - 1][j], dp[i][j - 1]));
        }
    }
    double res = dp[m - 1][n - 1];
    if(dtwMax < res){
        dtwMax = res;
    }
    if(dtwMin > res){
        dtwMin = res;
    }
    return res;
}
double Evaluation::CenterScore(int i){
    int n = midPointsList[i].size();
    int count = 0;
    for(auto it : midPointsList[i]){
        if(!(it->midp2tri[0] != nullptr&&
             it->midp2tri[1] != nullptr)){
            ++count;
        }
    }
    double score = count*1.0/n;
    return score;
}

double Evaluation::WidthScore(int i) {
    auto mps = midPointsList[i];
    int n = mps.size();
    double widthscore = 0;
    double widths = 0;
    for (int i = 0; i < n; i++) {
        widths += mps[i]->width;
    }
    widthscore = widths/n;
    if(widthMax < widthscore){
        widthMax = widthscore;
    }
    if(widthMin > widthscore){
        widthMin = widthscore;
    }
    return widthscore;
}


double Evaluation::LengthScore(int i){
    auto points = TOOL::Transeform(midPointsList[i]);
    int n = points.size();
    double lengthscore;
    double lengths = 0;
    for (int i = 0; i < n - 1; i++) {
        double temp = TOOL::Len(points[i], points[i+1]);
        lengths += temp;
    }
    lengthscore = lengths/(n-1);
    if(lengthMax < lengthscore){
        lengthMax = lengthscore;
    }
    if(lengthMin > lengthscore){
        lengthMin = lengthscore;
    }
    return lengthscore;
}
double Evaluation::AnglesScore(int i){
    auto points = TOOL::Transeform(midPointsList[i]);
    int n = points.size();
    double anglesscore = 0;
    double angless = 0;
    for (int i = 0; i < n - 2; i++) {
        Vect l1 = Vect(points[i], points[i+1]);
        Vect l2 = Vect(points[i+1], points[i+2]);
        double angletemp = abs(TOOL::Angle(l1, l2));
        // if (angletemp > AngltThreshold) {
        //     //anglesscore += 1000;
        //     anglesscore += angletemp*100;
        // }
        angless += angletemp;
    }
    anglesscore += angless/(n-2);
    if(angleMax < anglesscore){
        angleMax = anglesscore;
    }
    if(angleMin > anglesscore){
        angleMin = anglesscore;
    }
  return anglesscore;
}
double Evaluation::AnglesStandardDeviationScore(int i){
    auto points = TOOL::Transeform(midPointsList[i]);
    int n = points.size();
    double anglesVarianceScore = 0;
    double average = 0;
    std::vector<double> angles;
    for (int i = 0; i < n - 2; i++) {
        Vect l1 = Vect(points[i], points[i+1]);
        Vect l2 = Vect(points[i+1], points[i+2]);
        //double angletemp = abs(TOOL::Angle(l1, l2));
        double angletemp = TOOL::Angle(l1, l2);
        average += angletemp;
        angles.emplace_back(angletemp);
    }
    int anglesSize = angles.size();
    average /= anglesSize;
    for(int i = 0 ; i < anglesSize;i++){
        anglesVarianceScore += (angles[i] - average)*(angles[i] - average);
    }
    double anglesStandardDeviationScore = sqrt(anglesVarianceScore / anglesSize);
    if(anglesStandardDeviationMax < anglesStandardDeviationScore){
        anglesStandardDeviationMax = anglesStandardDeviationScore;
    }
    if(anglesStandardDeviationMin > anglesStandardDeviationScore){
        anglesStandardDeviationMin = anglesStandardDeviationScore;
    }
    return anglesStandardDeviationScore;
}

double Evaluation::LengthStandardDeviationScore(int i){
    auto points = TOOL::Transeform(midPointsList[i]);
    int n = points.size();
    double lengthvariancescore = 0;
    double lengths = 0;
    std::vector<double> lens;
    for (int i = 0; i < n - 1; i++) {
        double temp = TOOL::Len(points[i], points[i+1]);
        lens.push_back(temp);
        lengths += temp;

    }
    int lensSize = lens.size();
    double average = lengths / lensSize;
    for(auto it: lens){
        lengthvariancescore += (it - average)*(it - average);
    }
    double lengthStandardDeviationScore = sqrt(lengthvariancescore / lensSize);
    if(lengthStandardDeviationMax < lengthStandardDeviationScore){
        lengthStandardDeviationMax = lengthStandardDeviationScore;
    }
    if(lengthStandardDeviationMin > lengthStandardDeviationScore){
        lengthStandardDeviationMin = lengthStandardDeviationScore;
    }
    return lengthStandardDeviationScore;
}

void Evaluation::Init(const std::vector<std::vector<std::shared_ptr<MidPoint>>> &midPointsList,
                      const std::vector<std::shared_ptr<MidPoint>> &lastMidPoints,
                      std::shared_ptr<MidPoint> &StartPoint)
{
    this->midPointsList = midPointsList;
    this->lastMidPoints = lastMidPoints;
    this->StartPoint = StartPoint;
    midPointsScore.resize(midPointsList.size());
}

void Evaluation::Clear()
{
    midPointsScore.clear();
    dtwMax = -1;
    lengthMax = -1;
    anglesStandardDeviationMax = -1;
    angleMax = -1;
    lengthStandardDeviationMax = -1;
    widthMax = -1;
}

int Evaluation::Evaluate()
{
    for(int i = 0 ; i < midPointsList.size();i++){
        midPointsScore[i].depthScore = DepthScore(i);
        midPointsScore[i].dtwScore = DtwScore(i);
        midPointsScore[i].widthScore = WidthScore(i);
        midPointsScore[i].centerScore = CenterScore(i);
        midPointsScore[i].lengthScore = LengthScore(i);
        midPointsScore[i].lengthStandardDeviationScore = LengthStandardDeviationScore(i);
        midPointsScore[i].angleScore = AnglesScore(i);
        midPointsScore[i].anglesStandardDeviationScore = AnglesStandardDeviationScore(i);
    }
    for(auto it : midPointsScore){
        it.dtwScore = (it.dtwScore - dtwMin)/(dtwMax - dtwMin);
        it.lengthScore = 1 - (it.lengthScore - lengthMin)/(lengthMax - lengthMin);
        it.anglesStandardDeviationScore =(it.anglesStandardDeviationScore - anglesStandardDeviationMin)
                /(anglesStandardDeviationMax - anglesStandardDeviationMin);
        it.angleScore = (it.angleScore - angleMin)/(angleMax - angleMin);
        it.lengthStandardDeviationScore = (it.lengthStandardDeviationScore - lengthStandardDeviationMin)
                /(lengthStandardDeviationMax - lengthStandardDeviationMin);
        it.widthScore = (it.widthScore - widthMin)/(widthMax - widthMin);
    }
    int res = 0;
    double res_score = 1000000;
    for(auto i = 0 ; i < midPointsScore.size();i++){
        double temp_score = midPointsScore[i].depthScore * depthWeight + midPointsScore[i].dtwScore * dtwWeigth +
                midPointsScore[i].widthScore * widthWeigth + midPointsScore[i].centerScore * centerWeight +
                midPointsScore[i].lengthScore * lengthWeigth + midPointsScore[i].lengthStandardDeviationScore * lengthStandardDeviationWeight +
                midPointsScore[i].angleScore * anglesWeigth + midPointsScore[i].anglesStandardDeviationScore * angleStandardDeviationWeight;
        if(res_score > temp_score){
            res_score = temp_score;
            res = i;
        }
    }
    return res;
}
