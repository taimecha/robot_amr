#pragma once
#include <algorithm>
#include <cmath>
class AccelStepper {
 long pos=0,target=0;float velocity=0;
 public:
 static constexpr int DRIVER=1;
 long physical=0; bool stalled=false;
 AccelStepper(int,int,int){}
 void setCurrentPosition(long p){pos=target=p;velocity=0;}
 long currentPosition()const{return pos;}
 long targetPosition()const{return target;}
 void moveTo(long p){target=p;}
 long distanceToGo()const{return target-pos;}
 void setSpeed(float s){velocity=s;}
 float speed()const{return velocity;}
 bool runSpeed(){if(stalled||velocity==0)return false;int s=velocity>0?1:-1;pos+=s;physical+=s;return true;}
 bool run(){if(stalled||target==pos)return false;int s=target>pos?1:-1;pos+=s;physical+=s;return pos!=target;}
 void setMaxSpeed(float){} void setAcceleration(float){}
 void setMinPulseWidth(int){} void setPinsInverted(bool,bool=false,bool=false){}
};
