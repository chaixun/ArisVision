#ifndef AVOIDMOVE_H
#define AVOIDMOVE_H

#include <thread>
#include <chrono>
#include "aris.h"
#include "Aris_Vision.h"
#include "Vision_ObstacleDetection.h"
#include "Vision_Gait0.h"
#include "Vision_Terrain0.h"
#include "Vision_AvoidControl.h"
#include "Vision_RobotPos.h"
#include "RobObsPose.h"

using namespace aris::core;
using namespace RobObsPose;

namespace VisionAvoid
{

class VisionAvoidWrapper
{

public:
    VisionAvoidWrapper();
    ~VisionAvoidWrapper();

    static aris::sensor::KINECT kinect1;
    static VISION_WALK_PARAM visionWalkParam;
    static aris::control::Pipe<int> visionPipe;
    static std::thread visionThread;

    static RobPose targetPos;
    static vector<ObsPose> obsPosesGCS;

    static TerrainAnalysis terrainAnalysisResult;
    static ObstacleDetection obstacleDetectionResult;
    static AvoidControl avoidControlResult;


    static void KinectStart();
    static auto visionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
    static auto visionWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int;
    static auto stopVisionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;

private:
    static atomic_bool isAvoidAnalysisFinished;
    static atomic_bool isSending;
    static atomic_bool isStop;
};

}

#endif // AVOIDMOVE_H
