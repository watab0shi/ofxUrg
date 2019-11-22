#include "ofApp.h"

void ofApp::setup()
{
  ofLogToConsole();
  ofSetFrameRate(60);
  ofSetVerticalSync(true);
  ofBackground(0);

  urg.setMode(ofxUrg::DISTANCE_INTENSITY);
  urg.setupEthernet();

  ofLogNotice("Product", urg.productType());
  ofLogNotice("Serial", urg.serialId());
  ofLogNotice("Status", urg.status());
  ofLogNotice("State", urg.state());
  ofLogNotice("Firmware version", urg.firmwareVersion());

  urg.start();
}

void ofApp::update()
{
  urg.update();
}

void ofApp::draw()
{
  ofSetColor(128);
  urg.drawDebug();

  cam.begin();
  {
    ofEnableDepthTest();
    ofPushMatrix();
    {
      float s = 0.1;
      ofScale(s, s, s);
      ofRotateZ(-90);
      ofSetColor(255);
      urg.drawDebugPolar();
    }
    ofPopMatrix();

    ofFill();
    ofSetColor(255, 0, 0);
    for(auto c : urg.getClusters())
    {
      ofDrawSphere(c.centroid.getRotated(90, ofVec3f(1, 0, 0)) * ofVec3f(-1, -1, 1), 3);
    }

    ofDrawAxis(100);
    ofDisableDepthTest();
  }
  cam.end();

  ofSetColor(255);
  ofDrawBitmapString(ofToString(ofGetFrameRate(), 0), 20, 20);
  ofDrawBitmapString(ofToString(urg.getFps()), 20, 40);
}
