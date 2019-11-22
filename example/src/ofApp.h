#include "ofMain.h"
#include "ofxUrg.h"

class ofApp : public ofBaseApp
{
  ofxUrg::Processor urg;
  ofEasyCam cam;

public:
  void setup();
  void update();
  void draw();
};
