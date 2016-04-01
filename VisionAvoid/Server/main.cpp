#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>
#include <Aris_Vision.h>
#include "Vision_Terrain0.h"
#include "Vision_Gait0.h"

#include "rtdk.h"
#include "unistd.h"

using namespace aris::core;

aris::sensor::KINECT kinect1;

TerrainAnalysis terrainAnalysisResult;

atomic_bool isTerrainAnalysisFinished(false);
atomic_bool isSending(false);
atomic_bool isStop(false);

VISION_WALK_PARAM visionWalkParam;

aris::control::Pipe<int> visionPipe(true);

static auto visionThread = std::thread([]()
{
    while(true)
    {
        int a;
        visionPipe.recvInNrt(a);

        auto visiondata = kinect1.getSensorData();
        terrainAnalysisResult.TerrainAnalyze(visiondata.get().gridMap);

        if(terrainAnalysisResult.Terrain == FlatTerrain)
        {
            visionWalkParam.movetype = flatmove;
            double avoidMove[3] = {0, 0, 0.325};
            memcpy(visionWalkParam.movedata,avoidMove,3*sizeof(double));
            visionWalkParam.totalCount = 5000/2;
            cout<<"terrain move"<<endl;
        }
        else if(terrainAnalysisResult.Terrain == ObstacleTerrain)
        {
            if(terrainAnalysisResult.leftHeight > terrainAnalysisResult.rightHeight)
            {
                visionWalkParam.movetype = turn;
                visionWalkParam.turndata = -15;
                visionWalkParam.totalCount = 6000/2;
                cout<<"Turn Right"<<endl;
            }
            else
            {
                visionWalkParam.movetype = turn;
                visionWalkParam.turndata = 15;
                visionWalkParam.totalCount = 6000/2;
                cout<<"Turn Left"<<endl;
            }
        }
        isTerrainAnalysisFinished = true;
        cout<<"terrrainended"<<endl;
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

    if (isTerrainAnalysisFinished)
    {
        if(isFirstTime)
        {
            visionWalkParam.count = 0;
            isFirstTime = false;
        }

        auto &robot = static_cast<Robots::RobotBase &>(model);

        switch(visionWalkParam.movetype)
        {
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
                isTerrainAnalysisFinished = false;
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
                isTerrainAnalysisFinished = false;
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
        xml_address = "/home/hex/ArisVision/VisionStairs/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/ArisVision/VisionStairs/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
        xml_address = "/home/hex/ArisVision/VisionStairs/Robot_VIII.xml";
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
