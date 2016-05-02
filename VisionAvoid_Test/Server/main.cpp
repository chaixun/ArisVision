#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>
#include <vector>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>
#include <Aris_Vision.h>
#include "Vision_Gait0.h"
#include "Vision_ObstacleDetection.h"
#include "Vision_AvoidControl.h"
#include "Vision_RobotPos.h"
#include "Vision_Terrain0.h"

#include "rtdk.h"
#include "unistd.h"

using namespace aris::core;

aris::sensor::KINECT kinect1;

TerrainAnalysis terrainAnalysisResult;

atomic_bool isAvoidAnalysisFinished(false);
atomic_bool isSending(false);
atomic_bool isStop(false);

VISION_WALK_PARAM visionWalkParam;

aris::control::Pipe<int> visionPipe(true);

Pose targetPos{0, 8, 0, 0, 0, 0};
RobotPose robotPosResult;
ObstacleDetection obstacleDetectionResult;
vector<ObstaclePosition> obsPosesGCS;
AvoidControl avoidControlResult;

static auto visionThread = std::thread([]()
{   
    while(true)
    {
        int a;
        visionPipe.recvInNrt(a);

        auto visiondata = kinect1.getSensorData();

        terrainAnalysisResult.TerrainAnalyze(visiondata.get().gridMap, visiondata.get().pointCloud);

        cout<<"Curr Robot Pos: X:"<<robotPosResult.robotPoses.back().X<<" Y:"<<robotPosResult.robotPoses.back().Y<<endl;
        cout<<"Target Pos: X:"<<targetPos.X<<" Y:"<<targetPos.Y<<endl;
        cout<<"abs X: "<<fabs(robotPosResult.robotPoses.back().X - targetPos.X)<<endl;
        cout<<"abs Y: "<<fabs(robotPosResult.robotPoses.back().Y - targetPos.Y)<<endl;

        if(sqrt(pow((robotPosResult.robotPoses.back().Y - targetPos.Y),2)+pow((robotPosResult.robotPoses.back().X - targetPos.X),2)) <= 0.2)
        {
            visionWalkParam.movetype = nomove;
        }
        else
        {
            obstacleDetectionResult.ObstacleDetecting(visiondata.get().obstacleMap, robotPosResult.robotPoses.back());

            if(obstacleDetectionResult.obsPoses.size() > 0)
            {
                if(obsPosesGCS.size() == 0)
                {
                    obsPosesGCS.push_back(obstacleDetectionResult.obsPoses[0]);
                }
                else if(fabs(obsPosesGCS.back().X - obstacleDetectionResult.obsPoses[0].X) > obsPosesGCS.back().radius
                        ||fabs(obsPosesGCS.back().Y - obstacleDetectionResult.obsPoses[0].Y) > obsPosesGCS.back().radius)
                {
                    obsPosesGCS.push_back(obstacleDetectionResult.obsPoses[0]);
                }
            }

            for(int i = 0; i < obsPosesGCS.size(); i++)
            {
                cout<<"Obs "<<i<<" Pos: X:"<<obsPosesGCS[i].X<<" Y:"<<obsPosesGCS[i].Y<<" Radius:"<<obsPosesGCS[i].radius<<endl;
            }

            avoidControlResult.AvoidWalkControl(targetPos, robotPosResult.robotPoses.back(), obsPosesGCS);

            robotPosResult.robotPoses.push_back(avoidControlResult.nextRobotPos);

            cout<<"Walk Step Num: "<<avoidControlResult.avoidWalkParam.stepNum<<endl;
            cout<<"Walk Step Len: "<<avoidControlResult.avoidWalkParam.stepLength<<endl;
            cout<<"Walk Step Dir: "<<avoidControlResult.avoidWalkParam.walkDirection<<endl;
            cout<<"Next Robot Pos: X:"<<avoidControlResult.nextRobotPos.X<<" Y:"<<avoidControlResult.nextRobotPos.Y<<endl;

            visionWalkParam.movetype = avoidmove;
            visionWalkParam.walkLength = avoidControlResult.avoidWalkParam.stepLength;
            visionWalkParam.walkDirection = avoidControlResult.avoidWalkParam.walkDirection;
            visionWalkParam.walkNum = avoidControlResult.avoidWalkParam.stepNum;
            if(visionWalkParam.walkNum == 1)
            {
                visionWalkParam.totalCount = 1200;
            }
            else if(visionWalkParam.walkNum == 2)
            {
                visionWalkParam.totalCount = 2400;
            }
            else if(visionWalkParam.walkNum > 2)
            {
                visionWalkParam.totalCount = 2400*(visionWalkParam.walkNum - 1.5);
            }

        }

        isAvoidAnalysisFinished = true;

        cout<<"avoidAnalysisFinished"<<endl;
    }
});

auto visionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    aris::server::GaitParamBase param;
    msg_out.copyStruct(param);
}

auto visionWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int
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

auto stopVisionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    isStop = true;
}

int main(int argc, char *argv[])
{   
    kinect1.start();

    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/ArisVision/VisionAvoid/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/ArisVision/VisionAvoid/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
        xml_address = "/home/hex/ArisVision/VisionAvoid/Robot_VIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);
    rs.addCmd("vwk", visionWalkParse, visionWalk);
    rs.addCmd("swk", stopVisionWalkParse, visionWalk);

    rs.open();

    rs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)
            throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });

    aris::core::runMsgLoop();

    return 0;
}
