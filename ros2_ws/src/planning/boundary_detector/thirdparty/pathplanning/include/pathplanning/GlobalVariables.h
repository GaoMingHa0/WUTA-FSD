#pragma once

const double PI=3.1415926;
const double EPS = 1.0E-8;

class GlobalVariables {
public:
    static GlobalVariables* getinstance(){
        if(instance == nullptr){
            instance = new GlobalVariables();
        }
        return instance;
    }
    int GetMaxDepth(){
        return MaxDepth;
    }
    int GetDepthStep(){
        return DepthStep;
    }
    int GetMinDepth(){
        return MinDepth;
    }
    double GetTriMaxLen(){
        return TriMaxLen;
    }
    double GetTriMinLen(){
        return TriMinLen;
    }
    double GetMinAngleTheshold(){
        return MinAngleTheshold;
    }
    double GetMaxAngleTheshold(){
        return MaxAngleTheshold;
    }
    double GetIsFindStartPoint(){
        return IsFindStartPoint;
    }
    double GetCenterWeight(){
        return CenterWeight;
    }
    double GetDepthWeight(){
        return DepthWeight;
    }
    double GetDtwWeight(){
        return DtwWeight;
    }
    double GetWidthWeigth(){
        return WidthWeigth;
    }
    double GetLengthWeight(){
        return LengthWeight;
    }
    double GetAngleWeight(){
        return AngleWeight;
    }
    double GetAngleStandardDeviationWeight(){
        return AngleStandardDeviationWeight;
    }
    double GetLengthStandardDeviationWeight(){
        return LengthStandardDeviationWeight;
    }
    double GetAngleThreshold(){
        return AngleThreshold;
    }
    int GetSplineSize(){
        return SplineSize;
    }
    double GetRedundancy() { return Redundancy; }
    double GetStartPointRedundancy() { return StartPointRedundancy; }
    int GetPlugPointSize() { return PlugPointSize; }
    double GetDistanceToBoundary() { return DistanceToBoundary; }
    int GetIsUseFirstLapPoints() { return IsUseFirstLapPoints; }
    int GetIsUseFirstLapPointsPro() { return IsUseFirstLapPointsPro; }
private:
    static GlobalVariables* instance;
    GlobalVariables();
    //最大搜索深度
    int MaxDepth{5};
    int DepthStep{1};
    //最小搜索深度
    int MinDepth{2};
    //三角形最长边阈值
    double TriMaxLen{10.0};
    //三角形最短边阈值
    double TriMinLen{0.1};
    //三角形最小角阈值
    double MinAngleTheshold{5.0};
    //三角形最大角阈值
    double MaxAngleTheshold{175.0};

    //路径搜索，评价函数权重
    //路径搜索到起点时，得分
    double IsFindStartPoint{1.0};
    //搜索的路径点，路径点周围是否包含2个三角形，权重
    double CenterWeight{1.0};
    //搜索深度权重
    double DepthWeight{1.0};
    //dtw权重
    double DtwWeight{1.0};
    //宽度权重
    double WidthWeigth{1.0};
    //长度权重
    double LengthWeight{0.5};
    //角度权重
    double AngleWeight{1.0};
    //角度标准差权重
    double AngleStandardDeviationWeight{0.5};
    //长度标准差权重
    double LengthStandardDeviationWeight{0.5};

    //分支限界法，角度阈值
    double AngleThreshold{80.0};

    //三次样条插值，插入数量
    int SplineSize{5};
    double Redundancy{0.1};
    double StartPointRedundancy{1.0};
    int PlugPointSize{5};
    double DistanceToBoundary{2.0};
    int IsUseFirstLapPoints{0};
    int IsUseFirstLapPointsPro{0};
};
