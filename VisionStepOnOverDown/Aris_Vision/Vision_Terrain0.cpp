#include "Vision_Terrain0.h"
#include <math.h>
#include <string.h>

double TerrainAnalysis::CurrentHeight[4] = {0, 0, 0, 0};
int TerrainAnalysis::leftedge_z[6] = {0, 0, 0, 0, 0, 0};
int TerrainAnalysis::rightedge_z[6] = {0, 0, 0, 0, 0, 0};
int TerrainAnalysis::leftedge_x[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int TerrainAnalysis::rightedge_x[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int TerrainAnalysis::Terrain = FlatTerrain;

int frame_num = 0;

void TerrainAnalysis::TerrainAnalyze(const float oriGridMap[120][120])
{
    float GridMap[120][120];
    memcpy(GridMap, oriGridMap, 120*120*sizeof(float));
    bool positive[6] = {false, false, false, false, false, false};
    bool negative[6] = {false, false, false, false, false, false};

    for (int p = 0; p < 6; p++)
    {
        leftedge_z[p] = 0;
        rightedge_z[p] = 0;
    }

    for (int q = 0; q < 10; q++)
    {
        leftedge_x[q] = 0;
        rightedge_x[q] = 0;
    }

    Terrain = FlatTerrain;

    //Judge Terrain

    bool* positive_pointer = positive;
    bool* negative_pointer = negative;

    for(int k = 29; k <= 49; k++)
    {
        // fill in nan data along z middle 60
        int r = 2;
        while(GridMap[k+1][60] == 0 && r < 10)
        {
            GridMap[k+1][60] = GridMap[k+r][60];
            r++;
        }

        if(GridMap[k+1][60] - GridMap[k][60] > 0.05)
        {
            *positive_pointer = true;
            positive_pointer++;
        }

        if(GridMap[k+1][60] - GridMap[k][60] < -0.05)
        {
            *negative_pointer = true;
            negative_pointer++;
        }
    }

    if(positive[0] == true&& negative[0] == false)
    {
        Terrain = StepUpTerrain;
    }
    if(positive[0] == false&& negative[0] == true)
    {
        Terrain = StepDownTerrain;
    }
    if(positive[0] == true&& negative[0] == true)
    {
        Terrain = DitchTerrain;
    }
    if(positive[0] == false&& negative[0] == false)
    {
        Terrain = FlatTerrain;
    }

    if(Terrain != FlatTerrain)
    {
        //Find Edge

        int* rightz_pointer = rightedge_z;
        int* leftz_pointer = leftedge_z;
        int* rightx_pointer = rightedge_x;
        int* leftx_pointer = leftedge_x;

        /*Find Edge Along Z*/
        for(int m = 29; m <= 55; m++)
        {
            //fill in nan data along z right 49
            int p = 2;
            while(GridMap[m + 1][49] == 0 && p < 10)
            {
                GridMap[m + 1][49] = GridMap[m + p][49];
                p++;
            }

            if(fabs(GridMap[m+1][49]-GridMap[m][49]) > 0.05)
            {
                *rightz_pointer = m + 1;
                rightz_pointer++;
            }

            //fill in nan data along z left 49
            int q = 2;
            while(GridMap[m + 1][72] == 0 && q < 10)
            {
                GridMap[m + 1][72] = GridMap[m + q][72];
                q++;
            }

            if(fabs(GridMap[m+1][72]-GridMap[m][72]) > 0.05)
            {
                *leftz_pointer = m + 1;
                leftz_pointer++;
            }
        }

        /*Find Edge Along X*/
        for(int k = 0; k < 30; k++)
        {
            //fill in nan data along x right 50
            int s = 1;
            while(GridMap[60][60-k-1] == 0 && (60-k-1-s) > 23)
            {
                GridMap[60][60-k-1] = GridMap[60][60-k-1-s];
                s++;
            }

            if(fabs(GridMap[60][60-k-1] - GridMap[60][60-k]) > 0.05 )
            {
                *rightx_pointer = 60 - k;
                rightx_pointer++;
            }

            //fill in nan data along x left 50
            int t = 1;
            while(GridMap[60][60+k+1] == 0 && (60+k+1+t) < 97)
            {
                GridMap[60][60+k+1] = GridMap[60][60+k+1+t];
                t++;
            }

            if(fabs(GridMap[60][60+k+1] - GridMap[60][60+k]) > 0.05 )
            {
                *leftx_pointer = 60 + k;
                leftx_pointer++;
            }
        }
    }
    //Judge Command

    CurrentHeight[0] = (GridMap[39][43] + GridMap[39][44] + GridMap[40][43] + GridMap[40][44])/4;
    CurrentHeight[1] = (GridMap[39][48] + GridMap[39][49] + GridMap[40][48] + GridMap[40][49])/4;
    CurrentHeight[2] = (GridMap[39][78] + GridMap[39][79] + GridMap[40][78] + GridMap[40][79])/4;
    CurrentHeight[3] = (GridMap[39][72] + GridMap[39][73] + GridMap[40][72] + GridMap[40][73])/4;

    std::stringstream out;
    out<<frame_num;
    std::string filename = "GridMap" + out.str() + ".txt";
    std::ofstream Gridmapfile(filename);
    if (Gridmapfile.is_open())
    {
        for(int i = 0; i < 120; i++)
        {
            for(int j = 0; j < 120; j++)
            {
                Gridmapfile<<GridMap[i][j]<<" ";
            }
            Gridmapfile<<endl;
        }
    }
    Gridmapfile<<leftedge_z[0]<<" "<<rightedge_z[0];

    frame_num++;
}
