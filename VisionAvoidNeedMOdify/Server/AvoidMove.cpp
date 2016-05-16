#include "AvoidMove.h"

namespace VisionAvoid
{

aris::sensor::KINECT VisionAvoidWrapper::kinect1;
VISION_WALK_PARAM VisionAvoidWrapper::visionWalkParam;
aris::control::Pipe<int> VisionAvoidWrapper::visionPipe(true);
std::thread VisionAvoidWrapper::visionThread;

RobPose VisionAvoidWrapper::targetPos = {0, 8, 0, 0, 0, 0};
vector<ObsPose> VisionAvoidWrapper::obsPosesGCS;

TerrainAnalysis VisionAvoidWrapper::terrainAnalysisResult;
ObstacleDetection VisionAvoidWrapper::obstacleDetectionResult;
AvoidControl VisionAvoidWrapper::avoidControlResult;

atomic_bool VisionAvoidWrapper::isAvoidAnalysisFinished(false);
atomic_bool VisionAvoidWrapper::isSending(false);
atomic_bool VisionAvoidWrapper::isStop(false);

VisionAvoidWrapper::VisionAvoidWrapper()
{
    ;
}

VisionAvoidWrapper::~VisionAvoidWrapper()
{
    ;
}

void VisionAvoidWrapper::KinectStart()
{
    kinect1.start();

    visionThread = std::thread([]()
    {
        while(true)
        {
            int a;
            visionPipe.recvInNrt(a);

            auto visiondata = kinect1.getSensorData();

            terrainAnalysisResult.TerrainAnalyze(visiondata.get().gridMap, visiondata.get().pointCloud);

            //cout<<"Curr Robot Pos: X:"<<robotPosResult.robotPoses.back().X<<" Y:"<<robotPosResult.robotPoses.back().Y<<endl;
            cout<<"Curr Robot Pos: x:"<<robPoses.back().x<<" y:"<<robPoses.back().y<<" alpha:"<<robPoses.back().gama<<endl;

            cout<<"Target Pos: X:"<<targetPos.x<<" Y:"<<targetPos.y<<endl;
            cout<<"abs X: "<<fabs(robPoses.back().x - targetPos.x)<<endl;
            cout<<"abs Y: "<<fabs(robPoses.back().y - targetPos.y)<<endl;

            if(sqrt(pow((robPoses.back().y - targetPos.y),2)+pow((robPoses.back().x - targetPos.x),2)) <= 0.35)
            {
                visionWalkParam.movetype = nomove;
            }
            else
            {
                obstacleDetectionResult.ObstacleDetecting(visiondata.get().obstacleMap, robPoses.back());

                if(obstacleDetectionResult.obsPoses.size() > 0)
                {
                    if(obsPosesGCS.size() == 0)
                    {
                        obsPosesGCS.push_back(obstacleDetectionResult.obsPoses[0]);
                    }
                    else if(fabs(obsPosesGCS.back().x - obstacleDetectionResult.obsPoses[0].x) > obsPosesGCS.back().r
                            ||fabs(obsPosesGCS.back().y - obstacleDetectionResult.obsPoses[0].y) > obsPosesGCS.back().r)
                    {
                        obsPosesGCS.push_back(obstacleDetectionResult.obsPoses[0]);
                    }
                }

                for(int i = 0; i < obsPosesGCS.size(); i++)
                {
                    cout<<"Obs "<<i<<" Pos: X:"<<obsPosesGCS[i].x<<" Y:"<<obsPosesGCS[i].y<<" Radius:"<<obsPosesGCS[i].r<<endl;
                }

                avoidControlResult.AvoidWalkControl(targetPos, robPoses.back(), obsPosesGCS);

                robPoses.push_back(avoidControlResult.nextRobotPos);

                cout<<"Walk Step Num: "<<avoidControlResult.avoidWalkParam.stepNum<<endl;
                cout<<"Walk Step Len: "<<avoidControlResult.avoidWalkParam.stepLength<<endl;
                cout<<"Walk Step Dir: "<<avoidControlResult.avoidWalkParam.walkDirection<<endl;
                cout<<"Next Robot Pos: X:"<<avoidControlResult.nextRobotPos.x<<" Y:"<<avoidControlResult.nextRobotPos.y<<endl;

                visionWalkParam.movetype = avoidmove;
                visionWalkParam.walkLength = avoidControlResult.avoidWalkParam.stepLength;
                visionWalkParam.walkDirection = avoidControlResult.avoidWalkParam.walkDirection;
                visionWalkParam.walkNum = avoidControlResult.avoidWalkParam.stepNum;
                if(visionWalkParam.walkNum == 1)
                {
                    visionWalkParam.totalCount = 1200;
                }
                else
                {
                    visionWalkParam.totalCount = 2400*(visionWalkParam.walkNum - 0.5);
                }
            }

            isAvoidAnalysisFinished = true;

            cout<<"avoidAnalysisFinished"<<endl;
        }
    });
}

auto VisionAvoidWrapper::visionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    aris::server::GaitParamBase param;
    msg_out.copyStruct(param);
}

auto VisionAvoidWrapper::visionWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int
{
    static bool isFirstTime = true;

    if (isAvoidAnalysisFinished)
    {
        if(isFirstTime)
        {
            visionWalkParam.count = 0;
            isFirstTime = false;
        }

        auto &robot = static_cast<Robots::RobotBase &>(model);

        switch(visionWalkParam.movetype)
        {
        case nomove:
        {
            isStop = false;
            isFirstTime = true;
            isSending = false;
            isAvoidAnalysisFinished = false;
            return 0;
        }
            break;
        case avoidmove:
        {
            int remainCount = RobotVisionWalk(robot, visionWalkParam);
            visionWalkParam.count++;

            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isFirstTime = true;
                isSending = false;
                isAvoidAnalysisFinished = false;
                return -1;
            }
        }
            break;

        case turn:
        {

            int remainCount = RobotVisionWalk(robot, visionWalkParam);
            visionWalkParam.count++;

            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isFirstTime = true;
                isSending = false;
                isAvoidAnalysisFinished = false;
                return -1;
            }
        }
            break;
        case flatmove:
        {
            int remainCount = RobotVisionWalk(robot, visionWalkParam);
            visionWalkParam.count++;

            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isFirstTime = true;
                isSending = false;
                isAvoidAnalysisFinished = false;
                return -1;
            }
        }
            break;
        }
    }
    else
    {
        if(isSending)
        {
            return -1;
        }
        else
        {
            visionPipe.sendToNrt(6);
            isSending = true;
            return -1;
        }
    }
}

auto VisionAvoidWrapper::stopVisionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    isStop = true;
}

}
