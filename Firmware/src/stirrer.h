#ifndef STIRRER_H
#define STIRRER_H

void initStirrer();    // LEDC setup + startup kick (call once from setup())
void startStirrer();
void stopStirrer();
void pauseStirrer();   // Set duty to 0 (for clean ADC reads)
void resumeStirrer();  // Restore previous duty (no kick)

#endif // STIRRER_H
