#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

    ofSetLogLevel(OF_LOG_VERBOSE);
    
    kinect.init();
    kinect.open();
    
    if (kinect.isConnected()){
        ofLogNotice() << "Sensor-emitter distance: " << kinect.getSensorEmitterDistance() << " cm";
        ofLogNotice() << "Sensor-camera distance : " << kinect.getSensorCameraDistance() << " cm";
        ofLogNotice() << "Zero plane distance    : " << kinect.getZeroPlaneDistance() << " mm";
        ofLogNotice() << "Zero plane pixel size  : " << kinect.getZeroPlanePixelSize() << " mm";
    }
    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    
    nearThreshold = 230;
    farThreshold = 70;

    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    
//    angle = 30;
//    kinect.setCameraTiltAngle(angle);
    
    bDrawPointCloud = false;
    
    
}

//--------------------------------------------------------------
void ofApp::update(){

    ofBackground(128, 128, 128);
    kinect.update();
    
    if (kinect.isFrameNew()){
        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
        if (bThreshWithOpenCV){
            grayThreshNear = grayImage;
            grayThreshFar  = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else{
            unsigned char * pix = grayImage.getPixels();
            
            int numPixels = grayImage.getWidth() * grayImage.getHeight();
            for (int i = 0; i < numPixels; i++) {
                if (pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        grayImage.flagImageChanged();
        contourFinder.findContours(grayImage,                       // ofxCvGrayScaleImage input
                                   10,                              // minimum area
                                   (kinect.width * kinect.height)/2,// maximum area
                                   20,                              // nConsidered
                                   false);                          // bFindHoles
    }
}

//--------------------------------------------------------------
void ofApp::draw(){

//    kinect.draw(640, 0, 640, 480);
//    kinect.drawDepth(0, 0, 640, 480);
    
    ofSetColor(255,255, 255);
    
    if (bDrawPointCloud) {
        easyCam.begin();
        drawPointCloud();
        easyCam.end();
    } else {
        kinect.drawDepth(10, 10, 400, 300);
        kinect.draw(420, 10, 400, 300);
        
        grayImage.draw(10,320,400,300);
        contourFinder.draw(10, 320, 400, 300);
    }
    
    ofSetColor(255, 255, 255);
    stringstream reportStream;
    
    if (kinect.hasAccelControl()) {
        reportStream << "accel is : " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
        
    } else {
        reportStream << "newer kinect device " << "motor/led/accel are not supported" << endl << endl;
    }
    
}

void ofApp::drawPointCloud(){
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step =2 ;
    
    for (int y = 0; y < h; y += step) {
        for (int x = 0; x < w; x += step) {
            if (kinect.getDistanceAt(x, y)){
                mesh.addColor(kinect.getColorAt(x, y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    
    glPointSize(3);
    ofPushMatrix();
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000);
    glEnable(GL_DEPTH_TEST);
    mesh.drawVertices();
    glDisable(GL_DEPTH_TEST);
    ofPopMatrix();
    
}

void ofApp::exit(){
    kinect.setCameraTiltAngle(0);
    kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

    switch (key) {
        case ' ':
            bThreshWithOpenCV = !bThreshWithOpenCV;
            break;
            
        case 'p':
            bDrawPointCloud = !bDrawPointCloud;
            break;
            
        case '>':
        case '.':
            farThreshold++;
            if(farThreshold >255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold--;
            if(farThreshold < 0) farThreshold = 0;
            break;
        
        case '+':
        case '=':
            nearThreshold++;
            if(nearThreshold >255) nearThreshold = 255;
            break;
            
        case '-':
            nearThreshold--;
            if(nearThreshold < 0) nearThreshold = 0;
            break;
            
        case '1':
            kinect.setLed(ofxKinect::LED_GREEN);
            break;
            
        case '2':
            kinect.setLed(ofxKinect::LED_RED);
            break;

        case '3':
            kinect.setLed(ofxKinect::LED_YELLOW);
            break;
            
        case '4':
            kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            break;
        
        case '5':
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;
            
        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
            break;
            
        case OF_KEY_UP:
            angle--;
            if (angle > 30) {
                angle = 30;
            }
            kinect.setCameraTiltAngle(angle);
            break;
        
        case OF_KEY_DOWN:
            angle--;
            if (angle < -30) {
                angle = -30;
            }
            kinect.setCameraTiltAngle(angle);
            break;
            
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
