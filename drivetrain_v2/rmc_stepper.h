#ifndef RMC_STEPPER_H
#define RMC_STEPPER_H

#include "Arduino.h"

class RmcStepper {
    public:
        RmcStepper(unsigned char pulse_pin, unsigned char direction_pin,
                   unsigned int pulses_per_second);

        void setCurrentPos(long pos);
        void setGoalPos(long pos);
        void setSpeed(unsigned int pulses_per_second);
        
        long getCurrentPos();
        float getStepTime();

        void stop();
        void start();
        
        static void setActiveStepper(RmcStepper& stepper);
        static void handleActiveInterrupt();

    private:
        unsigned char pulse_pin_, direction_pin_;
        bool pulse_state_ = false;
        bool running_ = false;
        volatile long goal_pos_ = 0;
        volatile long current_pos_ = 0;
        float pulse_time_;
        
        static RmcStepper* active_stepper_;
    
        void startTimer_();
        void updateDirection_();
        void onTimerInterrupt_();
};

#endif

