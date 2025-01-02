#include "vex.h"
#include "math.h"
#include "utils.h"

//signum function, returns -1 if negative, 0 if 0, and 1 if positive
template <typename T> 
int sign(T val) {  
  //compares input and 0, true and false are interpreted as 1 and 0 in c++
  return (T(0) < val) - (val < T(0));
}

template <typename T>
T angleWrap(T _theta) {
  return fmod(_theta, 360);
}

float clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

Graph::Graph(float var, int ms, float g) {
  least = var;
  greatest = var;
  maxSize = ms;
  goal = g;
}

void Graph::updateData(float dataPoint) {
  graphData.push_back(dataPoint);

  if (graphData.size() > maxSize) {
    graphData.erase(graphData.begin());
  }
  
  least = *min_element(graphData.begin(), graphData.end());
  greatest = *max_element(graphData.begin(), graphData.end());
  
  if (greatest == least) {
    greatest += 0.1;
  }
}

void Graph::drawGraph() {
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.drawLine(graphLeftBorder, graphTopBorder, graphLeftBorder, graphBottomBorder);
  Brain.Screen.drawLine(graphLeftBorder, graphBottomBorder, graphRightBorder, graphBottomBorder);
  Brain.Screen.setPenColor(color::green);

  prevX = graphLeftBorder;
  prevY = graphTopBorder + ((graphData[0] - this->least) * ((graphBottomBorder-graphTopBorder)/(this->greatest-this->least)));

  for (int i = 0; i < graphData.size(); i++) {
    px = i * ((graphRightBorder - graphLeftBorder) / graphData.size()) + graphLeftBorder;
    py = 272 - (graphTopBorder*2 + ((graphData[i] - this->least) * ((graphBottomBorder - graphTopBorder)/(this->greatest - this->least))));
    Brain.Screen.drawLine(prevX, prevY, px, py);    
    prevX = px;
    prevY = py; 
  }
  
  Brain.Screen.printAt(graphLeftBorder-75, py, "%.2f", graphData[graphData.size()-1]);
  Brain.Screen.setPenColor(color::red);
  py = 272 - (graphTopBorder*2 + ((this->goal - this->least) * ((graphBottomBorder - graphTopBorder)/(this->greatest - this->least))));
  Brain.Screen.drawLine(graphLeftBorder, py, graphRightBorder, py);
  Brain.Screen.render();
}