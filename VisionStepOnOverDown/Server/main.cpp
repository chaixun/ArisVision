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

using namespace aris::core;

aris::sensor::KINECT kinect1;

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

aris::control::Pipe<int> visionPipe(true);

static auto visionThread = std::thread([]()
{
    while(true)
    {
        int a;
        visionPipe.recvInNrt(a);

        auto visiondata = kinect1.getSensorData();
        terrainAnalysisResult.TerrainAnalyze(visiondata.get().gridMap);

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
                        visionWalkParam.totalCount = 6000/2;
                        cout<<"terrain turn"<<endl;
                    }
                    else
                    {
                        visionWalkParam.movetype = flatmove;
                        memcpy(visionWalkParam.movedata,paramAdjust,3*sizeof(double));
                        visionWalkParam.totalCount = 5000/2;
                        cout<<"terrain move"<<endl;
                    }
                }
                else
                {
                    switch (terrainAnalysisResult.Terrain)
                    {
                    case StepUpTerrain:
                    {
                        cout<<"Move Body Up "<<endl;
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
                        double nextfootpos[7] = {0, 0, 0, 0, 0, 0, 0};
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
                        visionWalkParam.totalCount = 5000/2;
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
                visionWalkParam.totalCount = 5000/2;
                memcpy(visionWalkParam.movedata,move_data,sizeof(move_data));
            }
        }
        else
        {
            switch (terrain0)
            {
            case terrainStepUp:
            {
                double nextfootpos[7] = {0, 0, 0, 0, 0, 0, 0};
                visionStepUp(nextfootpos);

                visionWalkParam.movetype = stepup;
                visionWalkParam.totalCount = 18000;
                memcpy(visionWalkParam.stepupdata, nextfootpos,sizeof(nextfootpos));

                if(int(nextfootpos[6]) == 4)
                {
                    terrain0 = terrainNotKnown;
                }
            }
                break;
            case terrainStepDown:
            {
                double nextfootpos[7] = {0, 0, 0, 0, 0, 0, 0};
                visionStepDown(nextfootpos);

                if(int(nextfootpos[6]) == 5)
                {
                    cout<<"Move Body Down "<<endl;
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
            }
                break;
            case terrainStepOver:
            {
                double stepoverdata[4] = {0, 0, 0, 0};
                visionStepOver(stepoverdata);
                visionWalkParam.movetype = flatmove;
                visionWalkParam.totalCount = 5000/2;
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
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
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
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
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
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
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
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
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
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;
        case stopmove:
        {
            isStop = false;
            isFirstTime = true;
            isSending = false;
            isTerrainAnalysisFinished = false;
            return 0;
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
        xml_address = "/home/hex/ArisVision/VisionStepOnOverDown/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/ArisVision/VisionStepOnOverDown/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
        xml_address = "/home/hex/ArisVision/VisionStepOnOverDown/Robot_VIII.xml";
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
