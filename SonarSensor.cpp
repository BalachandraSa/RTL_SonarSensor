// ---------------------------------------------------------------------------
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2012 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
//
// See "SonarSensor.h" for purpose, syntax, version history, links, and more.
// ---------------------------------------------------------------------------
#define DEBUG 0

#include <Arduino.h>
#include <avr/io.h>
#include <RTL_StdLib.h>
#include "SonarSensor.h"


//******************************************************************************
// Useful macros
//******************************************************************************
#define OUTPUT_HI(reg, pinMask) *reg |=  pinMask;
#define OUTPUT_LO(reg, pinMask) *reg &= ~pinMask;
#define OUTPUT_HI_PULSE(reg, pinMask, duration) *reg |=  pinMask; delayMicroseconds(duration);
#define OUTPUT_LO_PULSE(reg, pinMask, duration) *reg &= ~pinMask; delayMicroseconds(duration);
#define WAITFOR_LO(reg, pinMask, timeoutTime, timeoutFlag) while ( (*reg & pinMask) && !timeoutFlag) { timeoutFlag = (micros() > timeoutTime); }
#define WAITFOR_HI(reg, pinMask, timeoutTime, timeoutFlag) while (!(*reg & pinMask) && !timeoutFlag) { timeoutFlag = (micros() > timeoutTime); }
#define IS_HI(reg, pinMask) (*reg & pinMask)
#define IS_LO(reg, pinMask) (!(*reg & pinMask))

// Probably shouldn't change these values unless you really know what you're doing.
#define MAX_SENSOR_DELAY 18000  // Max microseconds it takes for sensor to start the ping (SRF06 is the highest measured, just under 18ms).
#define PING_INTERVAL 29        // Min delay between successive pings in milliseconds.


//******************************************************************************
// Class variables
//******************************************************************************
DEFINE_CLASSNAME(SonarSensor);


// -----------------------------------------------------------------------------
// SonarSensor constructor
// -----------------------------------------------------------------------------
SonarSensor::SonarSensor(uint8_t triggerPin, uint8_t echoPin)
{
    _triggerPin = triggerPin;
    _echoPin = echoPin;

#if TWO_PIN_MODE
    pinMode(_triggerPin, OUTPUT);
    pinMode(_echoPin, INPUT);
#endif
}


//******************************************************************************
// Triggers a ping. 
//
// Returns: True if successful, otherwise false if unable to trigger ping.
//******************************************************************************
bool SonarSensor::TriggerPing()
{
    volatile uint8_t  triggerBit = digitalPinToBitMask(_triggerPin);                               // Get the port register bitmask for the trigger pin.
    volatile uint8_t  echoBit = digitalPinToBitMask(_echoPin);                                     // Get the port register bit-mask for the echo pin.
    volatile uint8_t* triggerOutput = (uint8_t*)portOutputRegister(digitalPinToPort(_triggerPin)); // Get the output port register for the trigger pin.
    volatile uint8_t* echoInput = (uint8_t*)portInputRegister(digitalPinToPort(_echoPin));         // Get the input port register for the echo pin.
    volatile uint8_t* triggerMode = (uint8_t*)portModeRegister(digitalPinToPort(_triggerPin));     // Get the port mode register for the trigger pin.

#if ONE_PIN_MODE
    *triggerMode |= triggerBit;    // Set trigger pin to output.
#endif

    OUTPUT_LO_PULSE(triggerOutput, triggerBit, 4);  // Make sure trigger pin is low for at least 4 microseconds
    OUTPUT_HI_PULSE(triggerOutput, triggerBit, 10); // Send 10 microsecond trigger pulse per sensor spec
    OUTPUT_LO(triggerOutput, triggerBit);           // End pulse

#if ONE_PIN_MODE
    *triggerMode &= ~triggerBit;   // Set trigger pin to input (when in one-pin mode this is setting the echo pin to input as both trigger & echo are tied together).
#endif

    auto timeout = micros() + MAX_SENSOR_DELAY;     // Set a timeout while waiting for ping to start
    auto timeoutFlag = false;

    // If echo pin is low, then wait for it to go high
    if (IS_LO(echoInput, echoBit))
    {
        // Wait for echo pin to go high to signal start of ping
        WAITFOR_HI(echoInput, echoBit, timeout, timeoutFlag);
    }

    _pingStartTime = micros() - 4;  // subtract 4us to account for micros() overhead

    return !timeoutFlag;
}


//******************************************************************************
// Performs a single ultrasonic sensor measurement, returning ping time in 
// microseconds. 
//
// Returns: If successful, returns the microseconds taken for the ping, which 
//          can be converted to centimeters with PingTimeToCentimeters. If 
//          the ping failed then PING_FAILED is returned (0xFFFF).
//******************************************************************************
uint16_t SonarSensor::Ping()
{
    TRACE(Logger(_classname_, this) << F("Ping") << endl);

    volatile auto echoBit = digitalPinToBitMask(_echoPin);                              // Get the port register bit-mask for the echo pin.
    volatile auto echoInput = (uint8_t*)portInputRegister(digitalPinToPort(_echoPin));  // Get the input port register for the echo pin.

    if (!TriggerPing())
    {
        TRACE(Logger(_classname_, this) << F("Ping: !!!!!!!!!!!! failed to trigger !!!!!!!!!!!!") << endl);
        return PING_FAILED;
    }

    auto timeout = _pingStartTime + MAX_PING;        // Compute timeout time

    // Wait for the ping echo.
    while (*echoInput & echoBit)
    {
       if (micros() > timeout) 
       {
           TRACE(Logger(_classname_, this) << F("Ping: ------------ timed out ------------") << endl);
           break;
       }
    }

    // Calculate ping time, 5uS of overhead.
    auto ping = UDIFF(micros(), _pingStartTime) - 5;

    TRACE(Logger(_classname_, this) << F("Ping=") << ping << F("us") << endl);

    return ping;
}


//******************************************************************************
// Take an ultrasonic sensor sample over multiple pings. This method takes 
// the specified number of samples (default is 5) and averages them, which 
// is one way to reduce the effect of noisy data.
//
// Returns: Is successful, the average ping distance in centimeters. Otherwise,
//          returns PING_FAILED (0xFFFF).
//******************************************************************************
uint16_t SonarSensor::MultiPing(const uint8_t samples)
{
    TRACE(Logger(_classname_, this) << F("MultiPing, samples=") << samples << endl);

    uint32_t sum = 0;
    uint8_t  count = 0;

    for (auto i = 0; i < samples; i++)
    {
        auto ping = Ping();

        // throw out bad samples
        if (ping == PING_FAILED) continue;
      
        sum += ping;
        count++;
    }
    
    uint16_t ping = (count > 0) ? PingTimeToCentimeters(sum / count) : PING_FAILED;
    
    TRACE(Logger(_classname_, this) << F("MultiPing, ping=") << ping << endl); 
    return ping;
}


uint16_t SonarSensor::PingMedian(const uint8_t maxSamples)
{
    TRACE(Logger(_classname_, this) << F("PingMedian") << endl);

    uint16_t samples[10];
    uint8_t sampleCount = maxSamples;

    samples[0] = 0;

    for (auto i = 0; i < sampleCount;)
    {
        auto ping = Ping();

        if (ping == PING_FAILED)  // Failed Ping
        {
            sampleCount--;        // Reduce sample count since we are not including this ping
            ping = 0;             // Adjust "ping" variable so delay calculation at end of loop is correct length
        }
        else                      // Ping in range, include as part of median
        {
            auto j = 0;

            // Shift sample array to correct position for sort insertion.
            for (j = i; j > 0 && samples[j - 1] < ping; j--)
                samples[j] = samples[j - 1];

            // Add ping to array in sorted position.
            samples[j] = ping;
            i++;
        }

        // Delay between pings, in milliseconds
        // NOTE: "ping >> 10" divides ping by 1024, which roughly converts microseconds to milliseconds (close enough).
        //if (i < sampleCount) delay(PING_INTERVAL - (ping >> 10));
    }

    uint16_t result = samples[sampleCount >> 1]; // Return the median ping distance.

    TRACE(Logger(_classname_, this) << F("PingMedian: median ping distance=") << PingTimeToCentimeters(result) << F("cm") << endl);

    return result;
}

