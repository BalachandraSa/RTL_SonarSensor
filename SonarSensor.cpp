// ---------------------------------------------------------------------------
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2012 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
//
// See "SonarSensor.h" for purpose, syntax, version history, links, and more.
// ---------------------------------------------------------------------------

#define DEBUG 0

#if defined(ARDUINO) && ARDUINO >= 100
    #include <Arduino.h>
#else
    #include <WProgram.h>
    #include <pins_arduino.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Debug.h>
#include "SonarSensor.h"


//******************************************************************************
// Useful macros
//******************************************************************************

#define ONE_PIN_MODE (TWO_PIN_MODE == false)
#define ECHO_TIMEOUT (MAX_SENSOR_DISTANCE*US_ROUNDTRIP_CM) // Maximum time to wait for an echo

#define OUTPUT_HI(reg, pinMask) *reg |=  pinMask; 
#define OUTPUT_LO(reg, pinMask) *reg &= ~pinMask; 
#define OUTPUT_HI_PULSE(reg, pinMask, duration) *reg |=  pinMask; delayMicroseconds(duration); 
#define OUTPUT_LO_PULSE(reg, pinMask, duration) *reg &= ~pinMask; delayMicroseconds(duration); 
#define WAITFOR_LO(reg, pinMask, timeoutTime, timeoutFlag) while ( (*reg & pinMask) && !timeoutFlag) { timeoutFlag = (micros() > timeoutTime); }
#define WAITFOR_HI(reg, pinMask, timeoutTime, timeoutFlag) while (!(*reg & pinMask) && !timeoutFlag) { timeoutFlag = (micros() > timeoutTime); }


//******************************************************************************
// Class variables
//******************************************************************************
// The sonar event ID.
EVENT_ID SonarSensor::SONAR_EVENT = EventSource::GenerateEventID();


// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
static inline void timer_start(uint16_t frequency, SonarSensor& sensor);
static inline void timer_stop();

static DebugHelper Debug("SonarSensor");


// -----------------------------------------------------------------------------
// SonarSensor constructor
// -----------------------------------------------------------------------------

SonarSensor::SonarSensor(uint8_t trigger_pin, uint8_t echo_pin) //, int max_cm_distance)
{
    _state.TriggerPin = trigger_pin;
    _state.AsyncValid = false;
    _echoInput = portInputRegister(digitalPinToPort(echo_pin));  // Get the input port register for the echo pin.
    _echoBit = digitalPinToBitMask(echo_pin);                    // Get the port register bitmask for the echo pin.

#if TWO_PIN_MODE
    pinMode(trigger_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
#endif
}


// ---------------------------------------------------------------------------
// Synchronous ping methods
// ---------------------------------------------------------------------------

uint16_t SonarSensor::Ping()
{
    Debug.Log(__func__);

    if (!TriggerPing()) return NO_ECHO;  // Trigger a ping, if it returns false return NO_ECHO

    while (*_echoInput & _echoBit)       // Wait for the ping echo.
    {
       // Stop the loop and return NO_ECHO (false) if we time out.
       if (micros() > _maxTime) return NO_ECHO;
    }

    return (micros() - _lastPingStart - 5); // Calculate ping time, 5uS of overhead.
}


uint16_t SonarSensor::PingInches()
{
    uint16_t echoTime = Ping();

    return PingTimeToInches(echoTime);
}


uint16_t SonarSensor::PingCentimeters()
{
    uint16_t echoTime = Ping();

    return PingTimeToCentimeters(echoTime);
}


uint16_t SonarSensor::PingMedian(uint8_t maxSamples)
{
    Debug.Log(__func__);

    uint16_t samples[maxSamples];

    samples[0] = NO_ECHO;

    for (int i = 0; i < maxSamples;)
    {
        uint16_t ping = Ping();

        if (ping == NO_ECHO)      // Ping out of range
        {
            maxSamples--;         // Reduce sample count since we are not including this ping
            ping = ECHO_TIMEOUT;  // Adjust "ping" variable so delay caclulation at end of loop is correct length
        }
        else                      // Ping in range, include as part of median
        {
            int j = 0;

            // Shift sample array to correct position for sort insertion.
            for (j = i; j > 0 && samples[j - 1] < ping; j--)
                samples[j] = samples[j - 1];

            // Add ping to array in sorted position.
            samples[j] = ping;
            i++;
        }

        // Delay between pings, in milliseconds
        // NOTE: "ping >> 10" divides ping by 1024, which roughly converts microseconds to milliseconds (close enough).
        if (i < maxSamples) delay(PING_INTERVAL - (ping >> 10));
    }

    uint16_t result = samples[maxSamples >> 1]; // Return the median ping distance.
    
    Debug.Log("%s=%i cm", __func__, PingTimeToCentimeters(result));
    
    return result;
}


// ---------------------------------------------------------------------------
// EventSource Implementation
// ---------------------------------------------------------------------------

void SonarSensor::Poll()
{
    if (_state.AsyncValid)
    {
        Debug.Log("%s(%i cm)", __func__, PingTimeToCentimeters(_asyncPing));
        DispatchEvent(SONAR_EVENT, _asyncPing);
        _state.AsyncValid = false;
    }
 }


// ---------------------------------------------------------------------------
// Internal Implementation
// ---------------------------------------------------------------------------

bool SonarSensor::TriggerPing()
{
    // Only trigger if ready for another ping
    if (!Ready()) return false;

    bool timeoutFlag = false;
    volatile uint8_t triggerBit = digitalPinToBitMask(_state.TriggerPin);                            // Get the port register bitmask for the trigger pin.
    volatile uint8_t* triggerOutput = portOutputRegister(digitalPinToPort(_state.TriggerPin));       // Get the output port register for the trigger pin.
    volatile uint8_t* triggerMode = (uint8_t*)portModeRegister(digitalPinToPort(_state.TriggerPin)); // Get the port mode register for the trigger pin.
    
#if ONE_PIN_MODE
    *triggerMode |= triggerBit;    // Set trigger pin to output.
#endif

    OUTPUT_LO_PULSE(triggerOutput, triggerBit, 4);  // Make sure trigger pin is low for at least 4 microseconds
    OUTPUT_HI_PULSE(triggerOutput, triggerBit, 10); // Send 10 microsecond trigger pulse per sensor spec
    OUTPUT_LO(triggerOutput, triggerBit);           // End pulse

#if ONE_PIN_MODE
    *triggerMode &= ~triggerBit;   // Set trigger pin to input (when in one-pin mode this is setting the echo pin to input as both trigger & echo are tied together).
#endif

    _maxTime =  micros() + MAX_SENSOR_DELAY;                 // Set a timeout while waiting for ping to start

    WAITFOR_LO(_echoInput, _echoBit, _maxTime, timeoutFlag); // Make sure echo pin is low
    WAITFOR_HI(_echoInput, _echoBit, _maxTime, timeoutFlag); // ... And then wait for it to go high to signal start of ping

    _lastPingStart = micros();                               // Note start of ping
    _maxTime = _lastPingStart + ECHO_TIMEOUT;                // Set ping timeout
    
    return !timeoutFlag;                                     // returns true if no timeouts occurred, otherwise false
}


// ---------------------------------------------------------------------------
// Async ping methods (uses timer interrupt - won't work with ATmega8 and ATmega128)
// NOTE: The time functions are marked 'inline' to tell the compiler that the 
//       function code can be merged into the calling code rather than generating 
//       a function call. This can save code memory and improve speed. Because 
//       of this, the SonarSensor Aysnc methods, which use the timer functions, 
//       appear AFTER the timer functions so the compiler has all the information 
//       it needs to do the inlining.
// ---------------------------------------------------------------------------

static volatile SonarSensor* _activeSensor = NULL;   // Must be volatile since referenced from an ISR


// ---------------------------------------------------------------------------
// Timer2/Timer4 interrupt method support functions (not called directly)
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
// You may call the time functions (i.e., mills() and micros() functions) 
// but their counts will not increment for the duration of the ISR. This means
// the time value returned these functions will always be the last known time 
// before the ISR was called. This also implies that attempting a delay loop using 
// these functions will hang the machine (since the time never increments)! 

#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
ISR(TIMER4_OVF_vect)
#else
ISR(TIMER2_COMPA_vect)
#endif
{
    if(_activeSensor != NULL) _activeSensor->CheckEchoAsync();
}


static inline void timer_start(uint16_t frequency, SonarSensor& sensor)
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


bool SonarSensor::PingAsync()
{
    // Make sure an async ping is not already in progress
    // (only one async ping can be active at a time since there is only one timer)
    if (_activeSensor == NULL)
    {
        // Trigger a ping, only start the timer if we got a good trigger
        if (TriggerPing())
        {
            _state.AsyncValid = false;
            _activeSensor = this;
            timer_start(ECHO_TIMER_FREQ, *this);

            return true;
        }
    }

    return false;
}


// Timer2 interrupt calls this function every 24 microseconds to check the ping status.
// Basically, keep it simple and fast - only do the minimal necessary!!
void SonarSensor::CheckEchoAsync()
{
    uint32_t now = micros();
    bool timeout = now > _maxTime;
    
    if (!(*_echoInput & _echoBit) || timeout)  // If ping echo received or timeout occurred
    {
        timer_stop();                          // Disable timer interrupt
        _asyncPing = timeout ? NO_ECHO : (now - _lastPingStart - 13); // Calculate ping time, 13uS of overhead.
        _activeSensor = NULL;                  // Indicate we are done so that another aysnc pinc can start
        _state.AsyncValid = true;
    }
}
