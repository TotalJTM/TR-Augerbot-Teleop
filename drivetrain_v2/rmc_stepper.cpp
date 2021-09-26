#include "rmc_stepper.h"

const float TIMER_INCREMENT_PER_SECOND = 6000000.0;
const float MAX_RATE = 25000;

const unsigned short MAX_WAIT = 65535;

SIGNAL(TIMER1_COMPA_vect) {
    RmcStepper::handleActiveInterrupt();
}

RmcStepper::RmcStepper(unsigned char pulse_pin, unsigned char direction_pin,
                       unsigned int pulses_per_second) :
    pulse_pin_(pulse_pin), direction_pin_(direction_pin)
{
    pinMode(pulse_pin_, OUTPUT);
    pinMode(direction_pin_, OUTPUT);

    setSpeed(pulses_per_second);
    
    setActiveStepper(*this);
};

void RmcStepper::setCurrentPos(long pos) {
    current_pos_ = pos;
}

void RmcStepper::setGoalPos(long pos) {
    if (running_ && pos != current_pos_) {
        goal_pos_ = pos;
        startTimer_();
    } else {
        goal_pos_ = pos;
    }
}

void RmcStepper::setSpeed(unsigned int pulses_per_second) {
    pulse_time_ = 1.0f / (static_cast<float>(pulses_per_second) * 2.0f);
}

long RmcStepper::getCurrentPos() {
    return current_pos_;
}

float RmcStepper::getStepTime() {
    return pulse_time_;
}

void RmcStepper::start() {
    if (!running_) {
        running_ = true;

        if (current_pos_ != goal_pos_) {
            startTimer_();
        }
    }
}

void RmcStepper::stop() {
    running_ = false;
}

void RmcStepper::setActiveStepper(RmcStepper& stepper) {
    active_stepper_ = &stepper;
}

void RmcStepper::handleActiveInterrupt() {
    active_stepper_->onTimerInterrupt_();
}

void RmcStepper::startTimer_() {
    unsigned int num_clocks = static_cast<unsigned int>(pulse_time_ * TIMER_INCREMENT_PER_SECOND);
    
    interrupts();
    sei();

    if (num_clocks > MAX_WAIT) {
        num_clocks = MAX_WAIT;
    }
    
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 5000;
    
    TCCR1B = (1 << WGM12) | (1 << CS10);
    TCNT1 = 0;
    TIMSK1 = (1 << OCIE1A);
}

void RmcStepper::onTimerInterrupt_() {
    noInterrupts();
    cli();
  
    TCCR1B = 0;
    TIMSK1 = 0;
  
    pulse_state_ = !pulse_state_;
    digitalWrite(pulse_pin_, pulse_state_);

    if (pulse_state_ == false) {
        if (current_pos_ < goal_pos_) {
            current_pos_++;
            digitalWrite(direction_pin_, LOW);
        } else if (current_pos_ > goal_pos_) {
            current_pos_--;
            digitalWrite(direction_pin_, HIGH);
        }
        
        if (running_ && current_pos_ != goal_pos_) {
            startTimer_();
        }
    } else {
        startTimer_();
    }
}

RmcStepper* RmcStepper::active_stepper_ = NULL;

