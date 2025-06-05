#ifndef TIMER_SYNC_H
#define TIMER_SYNC_H

#define _TIMERINTERRUPT_LOGLEVEL_     4
#include "ESP32TimerInterrupt.h"

// Base timer interval (1ms)
#define HW_TIMER_INTERVAL_MS      1

// Init ESP32 timer 0
ESP32Timer ITimer0(0);

// Shared variables
volatile bool shouldSample = false;
volatile uint32_t timerCount = 0;
volatile uint32_t targetCount = 0;

bool IRAM_ATTR TimerHandler0(void* timerNo)
{ 
    if (++timerCount == targetCount)
    {
        shouldSample = true;
        timerCount = 0;
        ITimer0.stopTimer();
    }
    return true;
}

class TimerSync {
private:
    bool timerInitialised;

public:
    TimerSync() : timerInitialised(false) {
    }

    void init() {
        if (!timerInitialised) {
            if (ITimer0.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler0)) {
                Serial.print(F("Timer initialised with interval of "));
                Serial.print(HW_TIMER_INTERVAL_MS);
                Serial.println(F(" milliseconds"));
                timerInitialised = true;
            } else {
                Serial.println(F("Timer initialisation failed"));
            }
            ITimer0.stopTimer();
        }
        shouldSample = false;
        timerCount = 0;
    }

    bool scheduleSampling(unsigned long delayMicros) {
        if (!timerInitialised) {
            Serial.println("Timer not initialised");
            return false;
        }

        // Convert microseconds to milliseconds (rounding up)
        uint32_t delayMs = (delayMicros + 999) / 1000;
        
        // Calculate number of timer intervals
        targetCount = delayMs / HW_TIMER_INTERVAL_MS;
        
        Serial.print("Scheduling sampling with delay of ");
        Serial.print(delayMs);
        Serial.print("ms (");
        Serial.print(targetCount);
        Serial.println(" intervals)");

        timerCount = 0;
        shouldSample = false;
        
        ITimer0.restartTimer();
        return true;
    }

    bool shouldTakeSample() {
        return shouldSample;
    }

    void reset() {
        ITimer0.stopTimer();
        shouldSample = false;
        timerCount = 0;
    }
};

#endif // TIMER_SYNC_H