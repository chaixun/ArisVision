#include "Vision_Gait0.h"
#include <string.h>
#include <math.h>
#include <iostream>

#ifndef PI
#define PI 3.141592653589793
#endif

void RobotVisionWalk(Robots::RobotBase &robot, const VISION_WALK_PARAM &param)
{
    //初始化
    static Aris::Dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];

    if (param.count%param.totalCount == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double a;
    double b;
    double d;
    double h;

    switch(param.movetype)
    {
    case turn:
    {
        a = 0;
        b = param.turndata/180*PI*2;
        d = 0;
        h = 0.05;
    }
        break;
    case flatmove:
    {
        if(param.movedata[0] != 0)
        {
            if(param.movedata[0] > 0)
            {
                a = -PI/2;
                d = param.movedata[0] * 2;
            }
            else
            {
                a = PI/2;
                d = -param.movedata[0] * 2;
            }
            b = 0;
            h = 0.05;
        }
        else
        {
            a = PI;
            b = 0;
            d = param.movedata[2] * 2;
            h = 0.05;
        }
    }
        break;
    default:
        break;
    }

    const double front[3]{ -std::sin(a),0,-std::cos(a) };
    const double left[3]{ -std::cos(a),0,std::sin(a) };
    const double up[3]{ 0,1,0 };

    int period_count = param.count%param.totalCount;
    const double s = -(PI / 2)*cos(PI * (period_count + 1) / param.totalCount) + PI / 2;//s 从0到PI.

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);


    double pq_b[7]{ 0,0,0,std::sin(b / 2)*up[0],std::sin(b / 2)*up[1],std::sin(b / 2)*up[2],std::cos(b / 2) };
    double pq_b_half[7]{ 0,0,0,std::sin(b / 4)*up[0],std::sin(b / 4)*up[1],std::sin(b / 4)*up[2],std::cos(b / 4) };
    double pq_b_quad[7]{ 0,0,0,std::sin(b / 8)*up[0],std::sin(b / 8)*up[1],std::sin(b / 8)*up[2],std::cos(b / 8) };
    double pq_b_eighth[7]{ 0,0,0,std::sin(b / 16)*up[0],std::sin(b / 16)*up[1],std::sin(b / 16)*up[2],std::cos(b / 16) };
    double pm_b[16], pm_b_half[16], pm_b_quad[16], pm_b_eighth[16];

    s_pq2pm(pq_b, pm_b);
    s_pq2pm(pq_b_half, pm_b_half);
    s_pq2pm(pq_b_quad, pm_b_quad);
    s_pq2pm(pq_b_eighth, pm_b_eighth);

    const int leg_begin_id = (param.count / param.totalCount) % 2 == 1 ? 3 : 0;

    if ((param.count / param.totalCount) == 0)//加速段
    {
        //规划腿
        for (int i = leg_begin_id; i < 18; i += 6)
        {
            //单腿运动需要分解成延圆周的直线运动，还有延自身的转动
            double leg_forward_dir[3], forward_d[3];
            s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

            s_pm_dot_v3(pm_b_half, beginPee + i, forward_d);
            s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
            s_daxpy(3, d/2, leg_forward_dir, 1, forward_d, 1);

            for (int j = 0; j < 3; ++j)
            {
                Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
            }
        }

        //规划身体位置
        double body_forward_dir[3], body_left_dir[3];
        s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
        s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

        for (int i = 0; i < 3; ++i)
        {
            Peb[i] = left[i] * s_interp(param.totalCount, period_count+1, 0, d*std::tan(b / 8) / 4 / std::cos(b / 8), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::sin(b / 4))
                    + front[i] * s_interp(param.totalCount, period_count + 1, 0, d / 4 / std::cos(b / 4), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::cos(b / 4));
        }

        //规划身体姿态
        double s_acc = Aris::Dynamic::acc_even(param.totalCount, period_count + 1);
        double pq[7] = { 0,0,0,std::sin(s_acc*b / 8)*up[0],std::sin(s_acc*b / 8)*up[1] ,std::sin(s_acc*b / 8)*up[2],std::cos(s_acc*b / 8) };
        double pe[6];
        s_pq2pe(pq, pe);
        std::copy(pe + 3, pe + 6, Peb + 3);
    }
    else if ((param.count / param.totalCount) == (param.n * 2 - 1))//减速段
    {
        //规划腿
        for (int i = leg_begin_id; i < 18; i += 6)
        {
            //单腿运动需要分解成延圆周的直线运动，还有延自身的转动
            double leg_forward_dir[3], forward_d[3];
            s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

            s_pm_dot_v3(pm_b_half, beginPee + i, forward_d);
            s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
            s_daxpy(3, d / 2, leg_forward_dir, 1, forward_d, 1);

            for (int j = 0; j < 3; ++j)
            {
                Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
            }
        }

        //规划身体位置
        double body_forward_dir[3], body_left_dir[3];
        s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
        s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

        for (int i = 0; i < 3; ++i)
        {
            Peb[i] = left[i] * s_interp(param.totalCount, period_count+1, 0, d*std::tan(b / 8) / 4 / std::cos(b / 8), 0, 0)
                    + front[i] * s_interp(param.totalCount, period_count+1, 0, d / 4 / std::cos(b / 4), d/2 / param.totalCount / std::cos(b / 2),0);
        }

        //规划身体姿态
        double s_dec = Aris::Dynamic::dec_even(param.totalCount, period_count + 1);
        double pq[7] = { 0,0,0,std::sin(s_dec*b / 8)*up[0],std::sin(s_dec*b / 8)*up[1] ,std::sin(s_dec*b / 8)*up[2],std::cos(s_dec*b / 8) };
        double pe[6];
        s_pq2pe(pq, pe);
        std::copy(pe + 3, pe + 6, Peb + 3);
    }
    else//匀速段
    {
        //规划腿
        for (int i = leg_begin_id; i < 18; i += 6)
        {
            //单腿运动需要分解成延圆周的直线运动，还有延自身的转动
            double leg_forward_dir[3], forward_d[3];
            s_pm_dot_v3(pm_b_half, front, leg_forward_dir);

            s_pm_dot_v3(pm_b, beginPee + i, forward_d);
            s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
            s_daxpy(3, d, leg_forward_dir, 1, forward_d, 1);

            for (int j = 0; j < 3; ++j)
            {
                Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
            }
        }

        //规划身体位置
        double d2 = d / 2 / std::cos(b / 4);
        for (int i = 0; i < 3; ++i)
        {
            Peb[i] = left[i] * s_interp(param.totalCount, period_count + 1, 0, d2*std::sin(b / 4), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::sin(b / 2))
                    + front[i] * s_interp(param.totalCount, period_count + 1, 0, d/2, d / 2 / param.totalCount / std::cos(b / 2), d / 2 / param.totalCount / std::cos(b / 2)*std::cos(b/2));
        }

        //规划身体姿态
        double s_even = even(param.totalCount, period_count + 1);
        double pq[7] = { 0,0,0,std::sin(s_even*b / 4)*up[0],std::sin(s_even*b / 4)*up[1] ,std::sin(s_even*b / 4)*up[2],std::cos(s_even*b / 4) };
        double pe[6];
        s_pq2pe(pq, pe);
        std::copy(pe + 3, pe + 6, Peb + 3);
    }

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);
}

void RobotBody(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam)
{
    static Aris::Dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double beginPeb[6];

    if (pParam.count%pParam.totalCount == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
        robot.GetPeb(beginPeb, beginMak);
    }

    double Peb[6], Pee[18];
    std::copy(beginPeb, beginPeb + 6, Peb);
    std::copy(beginPee, beginPee + 18, Pee);

    double s = -(PI / 2)*cos(PI * (pParam.count + 1) / pParam.totalCount) + PI / 2;

    Peb[0] += pParam.bodymovedata[0] * (1 - cos(s))/2;
    Peb[1] += pParam.bodymovedata[1] * (1 - cos(s))/2;
    Peb[2] += pParam.bodymovedata[2] * (1 - cos(s))/2;

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);
}

void RobotStepUp(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam)
{
    static Aris::Dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];

    if (pParam.count%pParam.totalCount == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double pBodyPE[6] = {0, 0, 0, 0, 0, 0};

    double pEE[18] =
    { -0.3, -1.05, -0.65,
      -0.45, -1.05, 0,
      -0.3, -1.05, 0.65,
      0.3, -1.05, -0.65,
      0.45, -1.05, 0,
      0.3, -1.05, 0.65 };

    double stepUpH = 0.25;
    double stepUpD = 0.325;

    double StepUpNextPos[6] = {0, 0, 0, 0, 0, 0};

    memcpy(StepUpNextPos,pParam.stepupdata,6*sizeof(double));

    static double StepUpCurrentPos[6] = {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05};

    for(int i = 0; i < 6; i++)
    {
        pEE[i*3 + 1] = StepUpCurrentPos[i];
    }

    int periodcounter = pParam.totalCount / 6;

    if(pParam.count < periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1) / periodcounter) + PI / 2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05)) * (1 - cos(s))/2;
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05)) * (1 - cos(s))/2;
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05)) * (1 - cos(s))/2;
    }
    else if(pParam.count >= periodcounter && pParam.count < 2 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05));

        pEE[2] += stepUpD * (1 - cos(s))/2;
        pEE[8] += stepUpD * (1 - cos(s))/2;
        pEE[14] += stepUpD * (1 - cos(s))/2;

        pBodyPE[2] += stepUpD/2 * (1 - cos(s))/2;
    }
    else if(pParam.count >= 2 * periodcounter && pParam.count < 3 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 2*periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05)) * (1 - cos(s)) / 2;
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05)) * (1 - cos(s)) / 2;
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05)) * (1 - cos(s)) / 2;

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[2] += stepUpD/2;
    }
    else if(pParam.count >= 3 * periodcounter && pParam.count < 4 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 3*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepUpH - (StepUpCurrentPos[1] + 1.05)) * (1 - cos(s))/2;
        pEE[10] += (stepUpH - (StepUpCurrentPos[3] + 1.05)) * (1 - cos(s))/2;
        pEE[16] += (stepUpH - (StepUpCurrentPos[5] + 1.05)) * (1 - cos(s))/2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05));

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[2] += stepUpD/2;
    }
    else if(pParam.count >= 4 * periodcounter && pParam.count < 5 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 4*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepUpH - (StepUpCurrentPos[1] + 1.05));
        pEE[10] += (stepUpH - (StepUpCurrentPos[3] + 1.05));
        pEE[16] += (stepUpH - (StepUpCurrentPos[5] + 1.05));

        pEE[5] += stepUpD * (1 -cos(s))/2;
        pEE[11] += stepUpD * (1 -cos(s))/2;
        pEE[17] += stepUpD * (1 -cos(s))/2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05));

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[2] += stepUpD/2 + stepUpD/2 * (1 - cos(s))/2;
    }
    else
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 5*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepUpH - (StepUpCurrentPos[1] + 1.05))
                - (stepUpH - (StepUpNextPos[1] + 1.05)) * (1 - cos(s))/2;
        pEE[10] += (stepUpH - (StepUpCurrentPos[3] + 1.05))
                - (stepUpH - (StepUpNextPos[3] + 1.05)) * (1 - cos(s))/ 2;
        pEE[16] += (stepUpH - (StepUpCurrentPos[5] + 1.05))
                - (stepUpH - (StepUpNextPos[5] + 1.05)) * (1 - cos(s))/2;

        pEE[5] += stepUpD;
        pEE[11] += stepUpD;
        pEE[17] += stepUpD;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05));

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[2] += stepUpD/2 + stepUpD/2;
    }

    if (pParam.totalCount - pParam.count - 1 == 0)
    {
        memcpy(StepUpCurrentPos, StepUpNextPos, 6*sizeof(double));
    }

    //pRobot->SetPee(pEE, pBodyPE);
    robot.SetPeb(pBodyPE, beginMak);
    robot.SetPee(pEE, beginMak);
}

void RobotStepDown(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam)
{
    static Aris::Dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];

    if (pParam.count%pParam.totalCount == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double pBodyPE[6] = {0, 0, 0, 0, 0, 0};

    double pEE[18] =
    { -0.3, -0.85, -0.65,
      -0.45, -0.85, 0,
      -0.3, -0.85, 0.65,
      0.3, -0.85, -0.65,
      0.45, -0.85, 0,
      0.3, -0.85, 0.65 };

    static double StepDownCurrentPos[6] = {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85};

    for(int i = 0; i < 6; i++)
    {
        pEE[i*3 + 1] = StepDownCurrentPos[i];
    }

    double stepDownH = 0.05;
    double stepDownD = 0.325;

    double StepDownNextPos[6] = {0, 0, 0, 0, 0, 0};

    memcpy(StepDownNextPos,pParam.stepdowndata,6*sizeof(double));

    int periodcounter = pParam.totalCount / 6;

    if(pParam.count < periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1) / periodcounter) + PI / 2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85)) * (1 - cos(s))/2;
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85)) * (1 - cos(s))/2;
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85)) * (1 - cos(s))/2;
    }
    else if(pParam.count >= periodcounter && pParam.count < 2 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85));

        pEE[2] += stepDownD * (1 -cos(s))/2;
        pEE[8] += stepDownD * (1 -cos(s))/2;
        pEE[14] += stepDownD * (1 -cos(s))/2;

        pBodyPE[2] += stepDownD/2 * (1 - cos(s))/2;
    }
    else if(pParam.count >= 2 * periodcounter && pParam.count < 3 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 2*periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85)) * (1 - cos(s)) / 2;
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85)) * (1 - cos(s)) / 2;
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85)) * (1 - cos(s)) / 2;

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[2] += stepDownD/2;
    }
    else if(pParam.count >= 3 * periodcounter && pParam.count < 4 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 3*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85)) * (1 - cos(s))/2;
        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85)) * (1 - cos(s))/2;
        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85)) * (1 - cos(s))/2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85));

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[2] += stepDownD/2;
    }
    else if(pParam.count >= 4 * periodcounter && pParam.count < 5 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 4*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85));
        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85));
        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85));

        pEE[5] += stepDownD * (1 -cos(s))/2;
        pEE[11] += stepDownD * (1 -cos(s))/2;
        pEE[17] += stepDownD * (1 -cos(s))/2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85));

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[2] += stepDownD/2 + stepDownD/2 * (1 - cos(s))/2;
    }
    else
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 5*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85))
                - (stepDownH + (-StepDownNextPos[1] - 0.85)) * (1 - cos(s)) / 2;
        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85))
                - (stepDownH + (-StepDownNextPos[3] - 0.85)) * (1 - cos(s)) / 2;
        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85))
                - (stepDownH + (-StepDownNextPos[5] - 0.85)) * (1 - cos(s)) / 2;

        pEE[5] += stepDownD;
        pEE[11] += stepDownD;
        pEE[17] += stepDownD;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85));

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[2] += stepDownD/2 + stepDownD/2;
    }

    if (pParam.totalCount - pParam.count - 1 == 0)
    {
        memcpy(StepDownCurrentPos, StepDownNextPos, 6*sizeof(double));
    }

    //pRobot->SetPee(pEE, pBodyPE);
    robot.SetPeb(pBodyPE, beginMak);
    robot.SetPee(pEE, beginMak);
}

