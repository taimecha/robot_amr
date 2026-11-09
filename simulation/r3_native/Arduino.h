#pragma once
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define OUTPUT 3
#define pdTRUE 1
#define portMUX_INITIALIZER_UNLOCKED 0
using QueueHandle_t = void*;
using portMUX_TYPE = int;
inline int pins[128]{};
inline uint32_t fakeMillis=0;
inline uint32_t millis(){return fakeMillis;}
inline int digitalRead(int p){return pins[p];}
inline void digitalWrite(int p,int v){pins[p]=v;}
inline void pinMode(int,int){}
inline void delay(int){}
inline int xQueueSend(QueueHandle_t,const void*,int){return 1;}
inline int xQueueReceive(QueueHandle_t,void*,int){return 0;}
struct SerialStub {
 template<class... T> void print(T...){}
 template<class... T> void println(T...){}
 template<class... T> void printf(T...){}
 void begin(int){}
 int available(){return 0;}
 char read(){return 0;}
};
inline SerialStub Serial;
