// ---------------------------------------------------------------------------
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2012 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
//
// See "SonarSensorAsync.h" for purpose, syntax, version history, links, and more.
// ---------------------------------------------------------------------------
#define DEBUG 0

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <RTL_Stdlib.h>
#include <RTL_Math.h>
#include <EventQueue.h>
#include "SonarSensorAsync.h"


//******************************************************************************
// Useful macros
//******************************************************************************

#define ECHO_TIMER_FREQ 24      // Frequency to check for a ping echo in async mode (every 24uS is about 0.4cm accuracy).

#define OUTPUT_HI(reg, pinMask) *reg |=  pinMask; 
#define OUTPUT_LO(reg, pinMask) *reg &= ~pinMask; 
#define OUTPUT_HI_PULSE(reg, pinMask, duration) *reg |=  pinMask; delayMicroseconds(duration); 
#define OUTPUT_LO_PULSE(reg, pinMask, duration) *reg &= ~pinMask; delayMicroseconds(duration); 
#define WAITFOR_LO(reg, pinMask, timeoutTime, timeoutFlag) while ( (*reg & pinMask) && !timeoutFlag) { timeoutFlag = (micros() > timeoutTime); }
#define WAITFOR_HI(reg, pinMask, timeoutTime, timeoutFlag) while (!(*reg & pinMask) && !timeoutFlag) { timeoutFlag = (micros() > timeoutTime); }


//******************************************************************************
// Class variables
//******************************************************************************
DEFINE_CLASSNAME(SonarSensorAsync);


// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
static inline void timer_start(uint16_t frequency, SonarSensorAsync& sensor);
static inline void timer_stop();


// -----------------------------------------------------------------------------
// SonarSensorAsync constructor
// -----------------------------------------------------------------------------
SonarSensorAsync::SonarSensorAsync(uint8_t triggerPin, uint8_t echoPin) : SonarSensor(triggerPin, echoPin)
{
    _asyncPing = 0;
    _echoBit = digitalPinToBitMask(echoPin);                               // Get the port register bit-mask for the echo pin.
    _echoInput = (uint8_t*)portInputRegister(digitalPinToPort(echoPin));   // Get the input port register for the echo pin.
}


// ---------------------------------------------------------------------------
// EventSource Implementation
// ---------------------------------------------------------------------------

void SonarSensorAsync::Poll()
{
    TRACE(Logger(_classname_, this) << F("Poll") << endl);

    if (_asyncPing > 0)
    {
        Logger(_classname_, this) << F("Poll: _asyncPing distance=") << PingTimeToCentimeters(_asyncPing) << F(" cm") << endl;
        QueueEvent(SONAR_EVENT, _asyncPing);
        _asyncPing = 0;
    }
 }


// ---------------------------------------------------------------------------
// Internal Implementation
//
// NOTE: Only one sensor can be doing an async ping at any given time since there
//       is only one timer. Because of this, it is OK to use static variables 
//       to point to the currently active sensor and hold the ping timeout time.
//       These static variables must be volatile since they are referenced from 
//       an ISR
// ---------------------------------------------------------------------------

// Points to the sonar sensor currently being monitored by the timer interrupt
static volatile SonarSensorAsync* pActiveSensor = NULL;

// Timeout end time for current ping
static volatile uint32_t timeoutTime;

// ---------------------------------------------------------------------------
// Async ping methods (uses timer interrupt - won't work with ATmega8 and ATmega128)
// NOTE: The time functions are marked 'inline' to tell the compiler that the 
//       function code can be merged into the calling code rather than generating 
//       a function call. This can save code memory and improve speed. Because 
//       of this, the SonarSensorAsync methods must appear AFTER the timer 
//       functions so the compiler has all the information it needs to do the 
//       inlining.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Timer2/Timer4 interrupt method support functions.
// ---------------------------------------------------------------------------
static inline void timer_start(uint16_t frequency, SonarSensorAsync& sensor)
{
    timer_stop();                // Disable timer interrupt

#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
    TCCR4A = TCCR4C = TCCR4D = TCCR4E = 0;
    TCCR4B = (1<<CS42) | (1<<CS41) | (1<<CS40) | (1<<PSR4); // Set Timer4 pre-scaler to 64 (4uS/count, 4uS-1020uS range).
    TIFR4 = (1<<TOV4);
    TCNT4 = 0;                   // Reset Timer4 counter.
    OCR4C = min((frequency>>2) - 1, 255); // Every count is 4uS, so divide by 4 (bitwise shift right 2) subtract one, then make sure we don't go over 255 limit.
    TIMSK4 = (1<<TOIE4);                  // Enable Timer4 interrupt.
#else
    ASSR &= ~(1<<AS2);           // Set clock, not pin.
    TCCR2A = (1<<WGM21);         // Set Timer2 to CTC mode.
    TCCR2B = (1<<CS22);          // Set Timer2 pre-scaler to 64 (4uS/count, 4uS-1020uS range).
    TCNT2 = 0;                   // Reset Timer2 counter.
    OCR2A = min((frequency>>2) - 1, 255); // Every count is 4uS, so divide by 4 (bitwise shift right 2) subtract one, then make sure we don't go over 255 limit.
    TIMSK2 |= (1<<OCIE2A);       // Enable Timer2 interrupt.
#endif
}


// Disables timer interrupt
static inline void timer_stop()
{
#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
    TIMSK4 = 0;
#else
    TIMSK2 &= ~(1<<OCIE2A);
#endif
}


// ---------------------------------------------------------------------------
// Timer interrupt service routine (ISR) - called every 24 microseconds
//
// As an ISR, there are restrictions on what you can do in this function, and 
// what functions you can call form this method. Basically, keep it simple and 
// fast - only do the minimal necessary!!
//
// Among other things, the delay functions won't work - calling delay() or 
// delayMicroseconds() will hang the machine!!
//
// You may call the time functions (i.e., mills() and micros()) but their counts 
// will not increment for the duration of the ISR. This means the time values 
// returned by these functions will always be the last known time before the 
// ISR was called. This also implies that attempting a delay loop using these 
// functions will hang the machine since the timer will never increment! 
// ---------------------------------------------------------------------------
#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
ISR(TIMER4_OVF_vect)
#else
ISR(TIMER2_COMPA_vect)
#endif
{
    if (pActiveSensor != NULL) pActiveSensor->CheckEchoAsync();
}


void SonarSensorAsync::CheckEchoAsync()
{
    auto now = micros();
    auto echoReceived = !(*_echoInput & _echoBit);
    auto isTimeout = now > timeoutTime;

    if (echoReceived || isTimeout)              // If ping echo received or timeout occurred
    {
        timer_stop();                           // Disable timer interrupt
        _asyncPing = isTimeout ? MAX_PING : ((now - _pingStartTime) - 13); // Calculate ping time, 13uS of overhead.
        pActiveSensor = NULL;                   // Indicate we are done so that another async ping can start
    }
}

bool SonarSensorAsync::PingAsync()
{
    // Make sure an async ping is not already in progress
    // (only one async ping can be active at a time since there is only one timer)
    if (pActiveSensor == NULL)
    {
        // Trigger a ping, only start the timer if we got a good trigger
        if (TriggerPing())
        {
            _asyncPing = 0;
            pActiveSensor = this;
            timeoutTime = _pingStartTime + MAX_PING;
            timer_start(ECHO_TIMER_FREQ, *this);

            return true;
        }
    }

    return false;
}


// // Timer2 interrupt calls this function every 24 microseconds to check the ping status.
// // Basically, keep it simple and fast - only do the minimal necessary!!
// void SonarSensorAsync::CheckEchoAsync()
// {
    // volatile auto now = micros();
    // volatile auto timeout = now > _maxTime;
    
    // if (!(*_echoInput & _echoBit) || timeout)  // If ping echo received or timeout occurred
    // {
        // timer_stop();                          // Disable timer interrupt
        // _asyncPing = timeout ? MAX_PING : ((now - _lastPingStart) - 13); // Calculate ping time, 13uS of overhead.
        // pActiveSensor = NULL;                  // Indicate we are done so that another async ping can start
    // }
// }
