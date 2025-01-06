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

extern double logDrive(double);

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
T angleWrap(T);

double clip(double, double, double);

double to_rad(double deg);

double to_deg(double rad);

double getRadius(double x, double y, double x1, double y1, double a);

class Graph {
  public:
    double* dataVar;
    int least;
    int greatest;
    double goal;
    double prevX = 0;
    double prevY = 0;
    double px = 0;
    double py = 0;
    double graphBottomBorder =  225.0;
    double graphTopBorder = 25.0;
    double graphLeftBorder = 75.0;
    double graphRightBorder = 475.0;
    int maxSize;

    std::vector<std::vector<double>> graphData {{},{}};

    Graph(double, int, double);

    void updateData(double dataPoint, int index);

    void drawGraph();
};

#endif
