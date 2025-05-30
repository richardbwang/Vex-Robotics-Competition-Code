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

double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

double to_rad(double deg) {
  return (deg * M_PI / 180);
}

double to_deg(double rad) {
  return (rad * 180 / M_PI);
}

double getRadius(double x, double y, double x1, double y1, double a) {
  double dx = x1 - x;
  double dy = y1 - y;
  if((2 * dy * sin(to_rad(90 - a))) == 0) {
    return 999;
  }
  return (dx * dx + dy * dy) / (2 * dy * sin(to_rad(90 - a)));
}

Graph::Graph(double var, int ms, double g) {
  least = var;
  greatest = var;
  maxSize = ms;
  goal = g;
}

void Graph::updateData(double dataPoint, int index) {
  graphData[index].push_back(dataPoint);

  if (graphData[index].size() > maxSize) {
    graphData[index].erase(graphData[index].begin());
  }
  
  double t_least = *min_element(graphData[index].begin(), graphData[index].end());
  double t_greatest = *max_element(graphData[index].begin(), graphData[index].end());
  least = least < t_least ? least : t_least;
  greatest = greatest > t_greatest ? greatest : t_greatest;
  
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
  prevY = graphTopBorder + ((graphData[0][0] - this->least) * ((graphBottomBorder-graphTopBorder)/(this->greatest-this->least)));
  color color_array[2] = {color::green, color::orange};
  for (int index = 0; index < graphData.size(); index++) {
    Brain.Screen.setPenColor(color_array[index]);
    for (int i = 0; i < graphData[index].size(); i++) {
      px = i * ((graphRightBorder - graphLeftBorder) / graphData[index].size()) + graphLeftBorder;
      py = 272 - (graphTopBorder*2 + ((graphData[index][i] - this->least) * ((graphBottomBorder - graphTopBorder)/(this->greatest - this->least))));
      Brain.Screen.drawLine(prevX, prevY, px, py);    
      prevX = px;
      prevY = py; 
    }
    Brain.Screen.printAt(graphLeftBorder - 75 * (index + 1), py, "%.2f", graphData[index][graphData.size()-1]);
  }
  Brain.Screen.setPenColor(color::red);
  py = 272 - (graphTopBorder*2 + ((this->goal - this->least) * ((graphBottomBorder - graphTopBorder) / (this->greatest - this->least))));
  Brain.Screen.drawLine(graphLeftBorder, py, graphRightBorder, py);
  Brain.Screen.render();
}