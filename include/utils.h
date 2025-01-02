#ifndef __UTILS__
#define __UTILS__

//declarations for utilities such as graphing classes and math functions.

#include <vector>
#include <string>
#include <iostream>
#include <climits>
#include <algorithm>

template <typename T>
extern int sign(T);

extern float logDrive(float);

extern enum autonPath {
  Right40 = 0,
  Right20 = 1,
  RightTallGoal = 2,
  Left40 = 3,
  Left20 = 4,
  LeftTallGoal = 5, 
  FullAWP = 6,
  Skills = 7
};

template <typename T>
extern T angleWrap(T);

extern float clip(float, float, float);

extern class Graph {
  public:
    float* dataVar;
    int least;
    int greatest;
    float goal;
    float prevX = 0;
    float prevY = 0;
    float px = 0;
    float py = 0;
    float graphBottomBorder =  225.0;
    float graphTopBorder = 25.0;
    float graphLeftBorder = 75.0;
    float graphRightBorder = 475.0;
    int maxSize;

    std::vector<float> graphData;

    Graph(float, int, float);

    void updateData(float);

    void drawGraph();
};

#endif
