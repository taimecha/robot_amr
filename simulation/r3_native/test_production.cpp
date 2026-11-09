#include <cassert>
#include <iostream>
#include "production_lift_under_test.hpp"
void requestImuZeroFromSerial(){}
void reset(bool lower=false) {
 liftMode=LiftMode::UNHOMED;liftHomed=false;liftHomeStage=LiftHomeStage::NONE;liftHomeLeavingUpper=false;
 liftStepper.setCurrentPosition(1000);liftStepper.physical=0;liftStepper.stalled=false;
 lowerLimit.active=lowerLimit.rawActive=lower;upperLimit.active=upperLimit.rawActive=false;
 pins[LOWER_LIMIT_PIN]=lower?LOW:HIGH;pins[UPPER_LIMIT_PIN]=HIGH;
 liftLastCommandMs=0;
}
int main(){
 reset();startLiftHome(0);lowerLimit.active=lowerLimit.rawActive=true;pins[LOWER_LIMIT_PIN]=LOW;updateLift(1);
 assert(liftHomeStage==LiftHomeStage::HOME_SETTLE && !liftHomed);
 for(int t=2;t<104;t++){liftLastCommandMs=t;updateLift(t);}
 assert(liftHomed && liftMode==LiftMode::IDLE && liftStepper.currentPosition()==0 && liftStepper.physical==-100);
 setLiftTarget(.08f,200);pins[LOWER_LIMIT_PIN]=HIGH;lowerLimit.active=lowerLimit.rawActive=false;
 for(int t=201;t<8202;t++){liftLastCommandMs=t;updateLift(t);}
 assert(liftStepper.currentPosition()==8000 && liftStepper.physical==7900);
 reset(true);startLiftHome(0);assert(liftHomeStage==LiftHomeStage::RELEASE_LOWER && !liftHomed);liftLastCommandMs=5001;updateLift(5001);assert(liftMode==LiftMode::FAULT);
 reset();startLiftHome(0);lowerLimit.active=lowerLimit.rawActive=true;pins[LOWER_LIMIT_PIN]=LOW;updateLift(1);updateLift(2);updateLift(351);assert(liftMode==LiftMode::UNHOMED && !liftHomed && liftStepper.physical==-1);
 reset();startLiftHome(0);lowerLimit.active=lowerLimit.rawActive=true;pins[LOWER_LIMIT_PIN]=LOW;updateLift(1);liftLastCommandMs=1002;updateLift(1002);assert(liftMode==LiftMode::FAULT && !liftHomed);
 reset();startLiftHome(0);lowerLimit.active=lowerLimit.rawActive=true;pins[LOWER_LIMIT_PIN]=LOW;updateLift(1);upperLimit.active=upperLimit.rawActive=true;pins[UPPER_LIMIT_PIN]=LOW;updateLift(2);assert(liftMode==LiftMode::FAULT && liftStepper.physical==0);
 reset();liftHomed=true;liftMode=LiftMode::MOVING;liftStepper.setCurrentPosition(100);liftTargetSteps=0;liftStepper.moveTo(0);lowerLimit.active=lowerLimit.rawActive=true;pins[LOWER_LIMIT_PIN]=LOW;updateLift(1);assert(liftStepper.currentPosition()==100 && liftStepper.distanceToGo()==0);
 reset();startLiftHome(0);lowerLimit.active=lowerLimit.rawActive=true;pins[LOWER_LIMIT_PIN]=LOW;updateLift(1);stopLift(LiftMode::UNHOMED);updateLift(2);assert(!liftHomed && liftStepper.physical==0);
 std::cout<<"production: exact100 downsteps, final zero, lift8000, lower release/stuck, watchdog, settle timeout, upper, normal lower, STOP PASS\n";
}
