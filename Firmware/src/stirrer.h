#ifndef STIRRER_H
#define STIRRER_H

void initStirrer();    // PWM setup + startup kick (call once from setup())
void startStirrer();
void stopStirrer();
bool isStirrerRunning();

#endif // STIRRER_H
