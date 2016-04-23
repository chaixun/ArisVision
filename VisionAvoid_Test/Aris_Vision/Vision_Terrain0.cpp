#include "Vision_Terrain0.h"

int TerrainAnalysis::Terrain = UnknownTerrain;
float TerrainAnalysis::leftHeight = 0;
float TerrainAnalysis::rightHeight = 0;

void TerrainAnalysis::TerrainAnalyze(const float oriGridMap[120][120])
{
    float GridMap[120][120];
    memcpy(GridMap, oriGridMap, 120*120*sizeof(float));

    bool isObstacle = false;
    Terrain = UnknownTerrain;
    leftHeight = 0;
    rightHeight = 0;

    for(int i=40; i<=54; i++)
    {
        for(int j = 41;j<=60;j++)
        {
            if(GridMap[i][j]>=0.225)
            {
                isObstacle = true;
                rightHeight = rightHeight + GridMap[i][j];
            }
        }
    }

    for(int i=40; i<=54; i++)
    {
        for(int j = 61;j<=80;j++)
        {
            if(GridMap[i][j]>=0.225)
            {
                isObstacle = true;
                leftHeight = leftHeight + GridMap[i][j];
            }
        }
    }

    if(isObstacle == false)
    {
        Terrain = FlatTerrain;
    }
    else
    {
        Terrain = ObstacleTerrain;
    }
}
