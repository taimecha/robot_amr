#include <cassert>
#include <iostream>
#include "../../esp32_firmware/Projects/cang_nang_ha/src/main.cpp"
void reset(bool active=false) {
 fault=FaultCode::NONE;homed=false;armed=false;homing=false;homingStage=HomingStage::NONE;
 sessionActive=true;lastCommandMs=0;stepper.setCurrentPosition(1000);stepper.physical=0;stepper.stalled=false;
 lowerLimit.stableLevel=active?HIGH:LOW;upperLimit.stableLevel=LOW;
}
int main(){
 reset();startHoming(0);lowerLimit.stableLevel=HIGH;handleLimitSwitches(1);
 assert(homingStage==HomingStage::HOME_SETTLE && !homed);
 for(int t=2;t<104;t++){lastCommandMs=t;handleLimitSwitches(t);checkWatchdog(t);runMotion(t);}
 assert(homed && !homing && !armed && stepper.currentPosition()==0 && stepper.physical==-100);
 assert(metresToSteps(.080f)==8000);stepper.moveTo(8000);armed=true;lowerLimit.stableLevel=LOW;
 for(int t=0;t<8000;t++)runMotion(200+t);
 assert(stepper.currentPosition()==8000 && stepper.physical==7900);
 reset(true);startHoming(0);assert(homingStage==HomingStage::RELEASE_LOWER && !homed);
 runMotion(5001);assert(fault==FaultCode::HOMING_RELEASE_FAILED && !homed);
 reset();startHoming(0);lowerLimit.stableLevel=HIGH;handleLimitSwitches(1);runMotion(2);
 lastCommandMs=0;checkWatchdog(301);runMotion(301);assert(fault==FaultCode::COMM_TIMEOUT && !homed && stepper.physical==-1);
 reset();startHoming(0);lowerLimit.stableLevel=HIGH;handleLimitSwitches(1);stepper.stalled=true;runMotion(1002);assert(fault==FaultCode::HOME_SETTLE_FAILED);
 reset();startHoming(0);lowerLimit.stableLevel=HIGH;handleLimitSwitches(1);lowerLimit.stableLevel=LOW;runMotion(2);assert(fault==FaultCode::HOME_SETTLE_FAILED);
 reset();startHoming(0);lowerLimit.stableLevel=HIGH;handleLimitSwitches(1);upperLimit.stableLevel=HIGH;handleLimitSwitches(2);runMotion(2);assert(fault==FaultCode::BOTH_LIMITS_ACTIVE && stepper.physical==0);
 reset();homed=true;armed=true;stepper.setCurrentPosition(100);stepper.moveTo(0);lowerLimit.stableLevel=HIGH;handleLimitSwitches(1);assert(stepper.currentPosition()==100 && stepper.distanceToGo()==0);
 reset();startHoming(0);lowerLimit.stableLevel=HIGH;handleLimitSwitches(1);latchFault(FaultCode::COMM_TIMEOUT);runMotion(2);assert(!homed && stepper.physical==0);
 DebouncedInput d{1};pins[1]=LOW;d.begin(0);pins[1]=HIGH;d.update(1);d.update(15);assert(!d.active());d.update(16);assert(d.active());
 std::cout<<"standalone: exact100 downsteps, final zero, lift8000, release/stuck, watchdog, timeout, lost switch, upper, normal lower, debounce PASS\n";
}
