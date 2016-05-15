#include "GaitMove.h"

namespace Escaping
{

double EscapingGaitWrapper::bodyPose[6] = {0, 0, 0, 0, 0, 0};
double EscapingGaitWrapper::feetPosi[18] = {-0.3, -0.45, -0.35, 0.3, 0.45, 0.3,
                                            0, 0, 0, 0, 0, 0,
                                            -0.65, 0, 0.65, -0.65, 0, 0.65};

GAIT_CMD EscapingGaitWrapper::gaitCommand = NOCMD;

EscapingPlanner EscapingGaitWrapper::escapingPlanner;

void EscapingGaitWrapper::escapingParse(const string &cmd, const map<string, string> &params, aris::core::Msg &msg)
{
    EscapingGaitParam param;
    for(auto &i:params)
    {
        if(i.first == "g")
        {
            gaitCommand = GAIT_CMD::GENERPATHANDPOSE;
            break;
        }
        else if(i.first == "b")
        {
            gaitCommand = GAIT_CMD::RUNGAIT;
            break;
        }
        else if(i.first == "s")
        {
            gaitCommand = GAIT_CMD::STOP;
            break;
        }
        else
        {
            cout<<"Parse Failed! "<<endl;
            break;
        }
    }

    msg.copyStruct(param);
    cout<<"Finish Parse!!!"<<endl;
}

int EscapingGaitWrapper::escapingGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const EscapingGaitParam &>(param_in);

    int timeNow = param.count;

    switch (gaitCommand)
    {
    case GAIT_CMD::NOCMD:
        break;
    case GAIT_CMD::GENERPATHANDPOSE:
        escapingPlanner.GenEscapPath();
        break;
    case GAIT_CMD::RUNGAIT:
        escapingPlanner.PlannerStart(timeNow);
        break;
    default:
        break;
    }

    gaitCommand = GAIT_CMD::NOCMD;

    if(escapingPlanner.GetPlannerState() == EscapingPlanner::GENBODYANDFOOTFINISHED)
    {
        cout<<"Generate Finished"<<endl;
    }

    escapingPlanner.OutBodyandFeetTraj(bodyPose, feetPosi, timeNow);

    if(escapingPlanner.GetPlannerState() == EscapingPlanner::PATHFOLLOWINGFINISHED)

    {
        return 0;
    }

    robot.SetPeb(bodyPose);
    robot.SetPee(feetPosi);
}

}
