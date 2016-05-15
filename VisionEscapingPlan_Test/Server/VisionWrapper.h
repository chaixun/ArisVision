#ifndef VISIONWRAPPER_H
#define VISIONWRAPPER_H

#include <thread>
#include <chrono>
#include "aris.h"
#include "Aris_Vision.h"
#include "Vision_ObstacleDetection.h"
#include "Vision_Terrain0.h"

namespace Vision
{
   using namespace aris::core;

  aris::sensor::KINECT kinect1;

  TerrainAnalysis TerrainAnalysisResult;
  ObstacleDetection ObstacleDetectionResult;

  std::thread visionThread;

  void StartVision()
  {
      kinect1.start();
      cout<<"Begin Vision Thread! "<<endl;
  }
}



#endif // VISIONWRAPPER_H
