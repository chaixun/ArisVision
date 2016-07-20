#include "AvoidMove.h"
#include <fstream>

namespace VisionAvoid
{

aris::sensor::KINECT VisionAvoidWrapper::kinect1;
VISION_WALK_PARAM VisionAvoidWrapper::visionWalkParam;
aris::control::Pipe<int> VisionAvoidWrapper::visionPipe(true);
std::thread VisionAvoidWrapper::visionThread;

vector<ObsPose> VisionAvoidWrapper::obsPosesGCS;

TerrainAnalysis VisionAvoidWrapper::terrainAnalysisResult;
ObstacleDetection VisionAvoidWrapper::obstacleDetectionResult;
AvoidControl VisionAvoidWrapper::avoidControlResult;

atomic_bool VisionAvoidWrapper::isAvoidAnalysisFinished(false);
atomic_bool VisionAvoidWrapper::isSending(false);
atomic_bool VisionAvoidWrapper::isStop(false);

std::string fileName1 = "RobPose.txt";
std::ofstream robPoseFile(fileName1);
std::string fileName2 = "obsPose.txt";
std::ofstream obsPoseFile(fileName2);

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

            cout<<"Curr Robot Pos: x:"<<robPoses.back().x<<" y:"<<robPoses.back().y<<" gama:"<<robPoses.back().gama<<endl;

            if(robPoseFile.is_open())
            {
                robPoseFile << robPoses.back().x <<" "<< robPoses.back().y <<" "<< robPoses.back().gama <<std::endl;
            }

            obstacleDetectionResult.ObstacleDetecting(visiondata.get().obstacleMap, robPoses.back());

            if(obstacleDetectionResult.obsPoses.size() > 0)
            {
                if(obsPosesGCS.size() == 0)
                {
                    obsPosesGCS.push_back(obstacleDetectionResult.obsPoses[0]);

                    if(obsPoseFile.is_open())
                    {
                        obsPoseFile << obsPosesGCS.back().x <<" "<< obsPosesGCS.back().y <<" "<< obsPosesGCS.back().r <<std::endl;
                    }

                }
                else if(fabs(obsPosesGCS.back().x - obstacleDetectionResult.obsPoses[0].x) > obsPosesGCS.back().r
                        ||fabs(obsPosesGCS.back().y - obstacleDetectionResult.obsPoses[0].y) > obsPosesGCS.back().r)
                {
                    obsPosesGCS.push_back(obstacleDetectionResult.obsPoses[0]);

                    if(obsPoseFile.is_open())
                    {
                        obsPoseFile << obsPosesGCS.back().x <<" "<< obsPosesGCS.back().y <<" "<< obsPosesGCS.back().r <<std::endl;
                    }

                }
            }

            for(int i = 0; i < obsPosesGCS.size(); i++)
            {
                cout<<"Obs "<<i<<" Pos: x:"<<obsPosesGCS[i].x<<" y:"<<obsPosesGCS[i].y<<" radius:"<<obsPosesGCS[i].r<<endl;
            }

            avoidControlResult.AvoidWalkControl(robPoses.back(), obsPosesGCS);

            robPoses.push_back(avoidControlResult.nextRobotPos);

            cout<<"Walk Step Num: "<<avoidControlResult.avoidWalkParam.stepNum<<endl;
            cout<<"Walk Step Len: "<<avoidControlResult.avoidWalkParam.stepLength<<endl;
            cout<<"Walk Step Dir: "<<avoidControlResult.avoidWalkParam.walkDirection<<endl;
            cout<<"Rob Pose : "<<avoidControlResult.avoidWalkParam.robHead<<endl;
            cout<<"Next Robot Pos: x:"<<avoidControlResult.nextRobotPos.x<<" y:"<<avoidControlResult.nextRobotPos.y<<endl;

            visionWalkParam.movetype = avoidmove;
            visionWalkParam.walkLength = avoidControlResult.avoidWalkParam.stepLength;
            visionWalkParam.walkDirection = avoidControlResult.avoidWalkParam.walkDirection;
            visionWalkParam.walkNum = avoidControlResult.avoidWalkParam.stepNum;
            visionWalkParam.turndata = avoidControlResult.avoidWalkParam.turnAngel;

            if(visionWalkParam.walkNum == 1)
            {
                visionWalkParam.totalCount = 1500;
            }
            else
            {
                visionWalkParam.totalCount = 3000*(visionWalkParam.walkNum - 0.5);
            }

            if(visionWalkParam.turndata != 0)
            {
                if(visionWalkParam.walkNum == 1)
                {
                    visionWalkParam.totalCount = 800;
                }
                else
                {
                    visionWalkParam.totalCount = 1600*(visionWalkParam.walkNum - 0.5);
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
