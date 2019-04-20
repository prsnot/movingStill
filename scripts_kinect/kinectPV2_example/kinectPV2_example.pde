// KinectPV2 Library example
// by Tassos Kanellos and Anna Laskari
// 07.10.2018
// Transmedia Labworks 2018 (Visual Unit)
// Greek National Opera

// Adapted from the examples of KinectPV2, Kinect for Windows v2 library for processing
// by Thomas Sanchez Lengeling. http://codigogenerativo.com/
// Added point colouring based on depth
// Added PeasyCam library navigation functionality 
// Added OSC communication feature

import java.nio.*; // KinectPV2 Library
import KinectPV2.KJoint; // KinectPV2 Library
import KinectPV2.*; // KinectPV2 Library
import peasy.*; // PeasyCam Library (Orbit Camera)
import oscP5.*; // OSC Library
import netP5.*; // OSC Library

OscP5 oscP5; 
NetAddress myRemoteLocation; 

PeasyCam cam;

KinectPV2 kinect;

int maxD = 4000; // 4m
int minD = 0;  //  0m
float scaleVal = 100.0;
int pointCloudSampleVal = 6;
PImage bodyImg;

PFont font;

void setup() 
{
  size(1920, 1080, P3D);
  font = createFont("Impact", 128);

  oscP5 = new OscP5(this, 999);
  myRemoteLocation = new NetAddress("127.0.0.1", 5);

  cam  =new PeasyCam(this, 150);


  kinect = new KinectPV2(this);
  //kinect.enableColorImg(true);
  kinect.enablePointCloud(true);
  //kinect.setLowThresholdPC(minD);
  //kinect.setHighThresholdPC(maxD);
  kinect.enableDepthImg(true);
  kinect.enableSkeleton3DMap(true);
  kinect.enableBodyTrackImg(true);
  //kinect.enableDepthMaskImg(true);
  kinect.init();

  strokeWeight(0.1);
  frameRate(120);
  strokeCap(SQUARE);
  perspective(PI/2f, float(width)/float(height), ((float(height)/2.0) / tan(PI/6f)) / 10000f, ((float(height)/2.0) / tan(PI/6f)) * 100f);
}

void draw() 
{
  bodyImg = kinect.getBodyTrackImage();
  bodyImg.loadPixels();

  background(0); 
  drawCS();
  pushMatrix();
  rotateZ(PI);
  rotateY(PI);

  translate(0, 0, -200);
  
  //kinect.setLowThresholdPC(minD);
  //kinect.setHighThresholdPC(maxD);
  FloatBuffer pointCloudBuffer = kinect.getPointCloudDepthPos();
  strokeWeight(3);
  for (int i = 0; i < kinect.WIDTHDepth * kinect.HEIGHTDepth; i+=pointCloudSampleVal) 
  {
    float x = pointCloudBuffer.get(i*3 + 0) * scaleVal;
    float y = pointCloudBuffer.get(i*3 + 1) * scaleVal;
    float z = pointCloudBuffer.get(i*3 + 2) * scaleVal;
    if (bodyImg.pixels[i] != color(255)) stroke(0, 0, 255);
    else stroke(map(z, 0, 400, 255, 0), map(z, 0, 400, 0, 255), 0);
    point(x, y, z);
  }

  //image(kinect.getColorImage(), 0, 0, 320, 240);
  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeleton3d();
  float x=0;
  float y=3;
  
  //individual JOINTS
  for (int i = 0; i < skeletonArray.size(); i++) 
  {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) 
    {
      KJoint[] joints = skeleton.getJoints();
      //draw different color for each hand state
      //drawHandState(joints[KinectPV2.JointType_HandRight]);
      //drawHandState(joints[KinectPV2.JointType_HandLeft]);
      
      //Draw body
      color col  = skeleton.getIndexColor();
      stroke(col);
      drawBody(joints);
      x = joints[KinectPV2.JointType_SpineBase].getX();
      y = joints[KinectPV2.JointType_SpineBase].getZ();
    }
  }
  popMatrix();


  //OscMessage msg = new OscMessage("/kinect"); 
  //msg.add(x);
  //msg.add(y);
  //oscP5.send(msg, myRemoteLocation);

  ortho();
  stroke(255, 0, 0);
  cam.beginHUD();
  textFont(font);
  textSize(100);
  text("fps:"+frameRate, 50, 100);
  text("pts:"+int(kinect.WIDTHDepth * kinect.HEIGHTDepth / float(pointCloudSampleVal)), 50, 200);
  text("res:"+kinect.WIDTHDepth + " x " +  kinect.HEIGHTDepth, 50, 300);
  // text("X: " + x + " - Y: " + y, 50, 400);
  cam.endHUD();
  perspective(PI/2f, float(width)/float(height), ((float(height)/2.0) / tan(PI/6f)) / 10000f, ((float(height)/2.0) / tan(PI/6f)) * 100f);
}


void drawBody(KJoint[] joints) {
  drawBone(joints, KinectPV2.JointType_Head, KinectPV2.JointType_Neck);
  drawBone(joints, KinectPV2.JointType_Neck, KinectPV2.JointType_SpineShoulder);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_SpineMid);

  drawBone(joints, KinectPV2.JointType_SpineMid, KinectPV2.JointType_SpineBase);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderRight);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderLeft);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipRight);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipLeft);

  // Right Arm    
  drawBone(joints, KinectPV2.JointType_ShoulderRight, KinectPV2.JointType_ElbowRight);
  drawBone(joints, KinectPV2.JointType_ElbowRight, KinectPV2.JointType_WristRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_HandRight);
  drawBone(joints, KinectPV2.JointType_HandRight, KinectPV2.JointType_HandTipRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_ThumbRight);

  // Left Arm
  drawBone(joints, KinectPV2.JointType_ShoulderLeft, KinectPV2.JointType_ElbowLeft);
  drawBone(joints, KinectPV2.JointType_ElbowLeft, KinectPV2.JointType_WristLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_HandLeft);
  drawBone(joints, KinectPV2.JointType_HandLeft, KinectPV2.JointType_HandTipLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_ThumbLeft);

  // Right Leg
  drawBone(joints, KinectPV2.JointType_HipRight, KinectPV2.JointType_KneeRight);
  drawBone(joints, KinectPV2.JointType_KneeRight, KinectPV2.JointType_AnkleRight);
  drawBone(joints, KinectPV2.JointType_AnkleRight, KinectPV2.JointType_FootRight);

  // Left Leg
  drawBone(joints, KinectPV2.JointType_HipLeft, KinectPV2.JointType_KneeLeft);
  drawBone(joints, KinectPV2.JointType_KneeLeft, KinectPV2.JointType_AnkleLeft);
  drawBone(joints, KinectPV2.JointType_AnkleLeft, KinectPV2.JointType_FootLeft);

  drawJoint(joints, KinectPV2.JointType_HandTipLeft);
  drawJoint(joints, KinectPV2.JointType_HandTipRight);
  drawJoint(joints, KinectPV2.JointType_FootLeft);
  drawJoint(joints, KinectPV2.JointType_FootRight);

  drawJoint(joints, KinectPV2.JointType_ThumbLeft);
  drawJoint(joints, KinectPV2.JointType_ThumbRight);

  drawJoint(joints, KinectPV2.JointType_Head);
}

void drawJoint(KJoint[] joints, int jointType) 
{
  //strokeWeight(2.0f + joints[jointType].getZ()*8);
  strokeWeight(10);
  point(joints[jointType].getX()*scaleVal, joints[jointType].getY()*scaleVal, joints[jointType].getZ()*scaleVal);
}

void drawBone(KJoint[] joints, int jointType1, int jointType2) 
{
  //strokeWeight(2.0f + joints[jointType1].getZ()*8);
  strokeWeight(4);
  //point(joints[jointType2].getX()*scaleVal, joints[jointType2].getY()*scaleVal, joints[jointType2].getZ()*scaleVal);
  line(joints[jointType1].getX()*scaleVal, joints[jointType1].getY()*scaleVal, joints[jointType1].getZ()*scaleVal, joints[jointType2].getX()*scaleVal, joints[jointType2].getY()*scaleVal, joints[jointType2].getZ()*scaleVal);
}

void drawHandState(KJoint joint) 
{
  handState(joint.getState());
  // strokeWeight(5.0f + joint.getZ()*8);
  strokeWeight(10);
  point(joint.getX()*scaleVal, joint.getY()*scaleVal, joint.getZ()*scaleVal);
}

void handState(int handState) 
{
  switch(handState) 
  {
  case KinectPV2.HandState_Open:
    stroke(0, 255, 0);
    break;
  case KinectPV2.HandState_Closed:
    stroke(255, 0, 0);
    break;
  case KinectPV2.HandState_Lasso:
    stroke(0, 0, 255);
    break;
  case KinectPV2.HandState_NotTracked:
    stroke(100, 100, 100);
    break;
  }
}

void drawCS()
{
  strokeWeight(2);
  stroke(255, 0, 0);
  line(0, 0, 0, 50, 0, 0);
  stroke(0, 255, 0);
  line(0, 0, 0, 0, 50, 0);
  stroke(0, 0, 255);
  line(0, 0, 0, 0, 0, 50);
}
