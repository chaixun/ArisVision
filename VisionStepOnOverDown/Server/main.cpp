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
#include "Vision_Control0.h"
#include "Vision_Gait0.h"

#include "rtdk.h"
#include "unistd.h"

using namespace Aris::Core;

Aris::Sensor::KINECT kinect1;

TerrainAnalysis terrainAnalysisResult;

atomic_bool isTerrainAnalysisFinished(false);
atomic_bool isSending(false);
atomic_bool isStop(false);

enum TerrainType0
{
    terrainNotKnown = 0,
    terrainStepUp = 1,
    terrainStepDown = 2,
    terrainStepOver = 3,
};

TerrainType0 terrain0 = terrainNotKnown;

VISION_WALK_PARAM visionWalkParam;

Aris::Control::Pipe<int> visionPipe(true);

static auto visionThread = std::thread([]()
{
    while(true)
    {
        int a;
        visionPipe.recvInNrt(a);

        auto visiondata = kinect1.GetSensorData();
        terrainAnalysisResult.TerrainAnalyze(visiondata.Get().gridMap);

        if(terrain0 == terrainNotKnown)
        {
            if(terrainAnalysisResult.Terrain != FlatTerrain)
            {
                /*Adjust x y z theta*/
                double paramAdjust[4] = {0, 0, 0, 0};
                bool adjustFinished = false;
                visionAdjust(paramAdjust, &adjustFinished);

                if(adjustFinished == false)
                {
                    /*let robot move*/
                    if(paramAdjust[3] != 0)
                    {
                        visionWalkParam.movetype = turn;
                        visionWalkParam.turndata = paramAdjust[3];
                        visionWalkParam.totalCount = 6001;
                    }
                    else
                    {
                        visionWalkParam.movetype = flatmove;
                        memcpy(visionWalkParam.movedata,paramAdjust,3*sizeof(double));
                        visionWalkParam.totalCount = 5000;
                    }
                }
                else
                {
                    switch (terrainAnalysisResult.Terrain)
                    {
                    case StepUpTerrain:
                    {
                        /*the robot move body*/
                        double movebody[3] = {0, 0.2, 0};
                        visionWalkParam.movetype = bodymove;
                        memcpy(visionWalkParam.bodymovedata, movebody,3*sizeof(double));
                        visionWalkParam.totalCount = 2500;
                        terrain0 = terrainStepUp;
                    }
                        break;
                    case StepDownTerrain:
                    {
                        terrain0 = terrainStepDown;
                        double nextfootpos[6] = {0, 0, 0, 0, 0, 0};
                        visionStepDown(nextfootpos);
                        visionWalkParam.movetype = stepdown;
                        visionWalkParam.totalCount = 18000;
                        memcpy(visionWalkParam.stepdowndata,nextfootpos,sizeof(nextfootpos));
                    }
                        break;
                    case DitchTerrain:
                    {
                        terrain0 = terrainStepOver;
                        double stepoverdata[4] = {0, 0, 0, 0};
                        visionStepOver(stepoverdata);
                        visionWalkParam.movetype = flatmove;
                        visionWalkParam.totalCount = 5000;
                        memcpy(visionWalkParam.movedata,stepoverdata + 1, 3*sizeof(double));
                    }
                        break;
                    default:
                        break;
                    }
                }
            }
            else
            {
                cout<<"FLAT TERRAIN MOVE"<<endl;
                cout<<"MOVE FORWARD: "<<0.325<<endl;
                double move_data[3] = {0, 0, 0.325};

                visionWalkParam.movetype = flatmove;
                visionWalkParam.totalCount = 5000;
                memcpy(visionWalkParam.movedata,move_data,sizeof(move_data));
            }
        }
        else
        {
            switch (terrain0)
            {
            case terrainStepUp:
            {
                bool stepUpFinished = true;
                double nextfootpos[6] = {0, 0, 0, 0, 0, 0};
                visionStepUp(nextfootpos);
                visionWalkParam.movetype = stepup;
                visionWalkParam.totalCount = 18000;
                memcpy(visionWalkParam.stepupdata, nextfootpos,sizeof(nextfootpos));
                for(int i = 0; i < 6; i++)
                {
                    if(nextfootpos[i] != -0.85)
                    {
                        stepUpFinished = false;
                    }
                }
                if(stepUpFinished == true)
                {
                    terrain0 = terrainNotKnown;
                }
            }
                break;
            case terrainStepDown:
            {
                bool stepDownFinished = true;
                double nextfootpos[6] = {0, 0, 0, 0, 0, 0};
                static double lastfootpos[6] = {0, 0, 0, 0, 0, 0};

                visionStepDown(nextfootpos);

                for(int i = 0; i < 6; i++)
                {
                    if(nextfootpos[i] != -1.05||lastfootpos[i] != -1.05)
                    {
                        stepDownFinished = false;
                    }

                }
                if(stepDownFinished == true)
                {
                    double movebody[3] = {0, -0.2, 0};
                    visionWalkParam.movetype = bodymove;
                    visionWalkParam.totalCount = 2500;
                    memcpy(visionWalkParam.bodymovedata, movebody, sizeof(movebody));
                    terrain0 = terrainNotKnown;
                }
                else
                {
                    visionWalkParam.movetype = stepdown;
                    visionWalkParam.totalCount = 18000;
                    memcpy(visionWalkParam.stepdowndata,nextfootpos,sizeof(nextfootpos));
                }
                memcpy(lastfootpos, nextfootpos, 6*sizeof(double));
            }
                break;
            case terrainStepOver:
            {
                double stepoverdata[4] = {0, 0, 0, 0};
                visionStepOver(stepoverdata);
                visionWalkParam.movetype = flatmove;
                visionWalkParam.totalCount = 5000;
                memcpy(visionWalkParam.movedata,stepoverdata + 1, 3*sizeof(double));

                if(int(stepoverdata[0]) == 4)
                {
                    terrain0 = terrainNotKnown;
                }
            }
                break;
            default:
                break;
            }
        }

        isTerrainAnalysisFinished = true;
    }

});

auto visionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)->void
{

}

auto visionWalk(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase & plan_param)->int
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
            RobotVisionWalk(robot, visionWalkParam);
            int remainCount = visionWalkParam.totalCount - visionWalkParam.count - 1;
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
            RobotVisionWalk(robot, visionWalkParam);
            int remainCount = visionWalkParam.totalCount - visionWalkParam.count - 1;
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
        case bodymove:
        {
            RobotBody(robot, visionWalkParam);
            int remainCount = visionWalkParam.totalCount - visionWalkParam.count - 1;
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
        case stepup:
        {
            RobotStepUp(robot, visionWalkParam);
            int remainCount = visionWalkParam.totalCount - visionWalkParam.count - 1;
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
        case stepdown:
        {
            RobotStepDown(robot, visionWalkParam);
            int remainCount = visionWalkParam.totalCount - visionWalkParam.count - 1;
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

auto stopVisionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)->void
{
    isStop = true;
}

int main(int argc, char *argv[])
{
    kinect1.Start();
    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";

        xml_address = "/home/hex/ArisVision/VisionStepOnOverDown/Client/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/ArisVision/VisionStepOnOverDown/Client/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
        xml_address = "/home/hex/ArisVision/VisionStepOnOverDown/Client/Robot_VIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = Aris::Server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);
    rs.addCmd("vwk", visionWalkParse, visionWalk);
    rs.addCmd("svw", stopVisionWalkParse, visionWalk);

    rs.open();

    rs.setOnExit([&]()
    {
        Aris::Core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)
            throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        Aris::Core::stopMsgLoop();
    });

    Aris::Core::runMsgLoop();

    return 0;

}
