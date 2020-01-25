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
#define MAX_RESET_TIME 140000   // Max microseconds required for sensor to reset after timeout
#define PING_INTERVAL 29000     // Min microseconds between successive pings.

// Minimum time between successive pings in microseconds (approx 50ms) 
static const uint16_t MIN_CYCLE_TIME = MAX_PING + PING_INTERVAL;


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
    _lastPing = 0;
    _pingStartTime = micros();

#if TWO_PIN_MODE
    pinMode(_triggerPin, OUTPUT);
    pinMode(_echoPin, INPUT);
#endif
}


bool SonarSensor::Ready() 
{
    volatile uint8_t  echoBit = digitalPinToBitMask(_echoPin);                              // Get the port register bit-mask for the echo pin.
    volatile uint8_t* echoInput = (uint8_t*)portInputRegister(digitalPinToPort(_echoPin));  // Get the input port register for the echo pin.

    auto nextStartTime = _pingStartTime + MIN_CYCLE_TIME;

    return IS_LO(echoInput, echoBit) && (micros() > nextStartTime);
//    return (micros() > nextStartTime);
};


//******************************************************************************
// Triggers a ping. 
//
// Returns: True if successful, otherwise false if unable to trigger ping.
//******************************************************************************
bool SonarSensor::TriggerPing()
{
    //TRACE(Logger(_classname_, this) << F("triggerPin=") << _triggerPin << F(", echoPin=") << _echoPin << endl);
    //TRACE(Logger(_classname_, this) << F("TWO_PIN_MODE=") << TWO_PIN_MODE << endl);

    volatile uint8_t  triggerBit = digitalPinToBitMask(_triggerPin);                               // Get the port register bitmask for the trigger pin.
    volatile uint8_t  echoBit = digitalPinToBitMask(_echoPin);                                     // Get the port register bit-mask for the echo pin.
    volatile uint8_t* triggerOutput = (uint8_t*)portOutputRegister(digitalPinToPort(_triggerPin)); // Get the output port register for the trigger pin.
    volatile uint8_t* echoInput = (uint8_t*)portInputRegister(digitalPinToPort(_echoPin));         // Get the input port register for the echo pin.

#if ONE_PIN_MODE
    volatile uint8_t* triggerMode = (uint8_t*)portModeRegister(digitalPinToPort(_triggerPin));     // Get the port mode register for the trigger pin.
    *triggerMode |= triggerBit;    // Set trigger pin to output.
#endif
    
    // Pulse the trigger pin to start the ping
    OUTPUT_LO_PULSE(triggerOutput, triggerBit, 4);  // Make sure trigger pin is low for at least 4 microseconds
    OUTPUT_HI_PULSE(triggerOutput, triggerBit, 10); // Send minimum 10 microsecond trigger pulse per sensor spec
    OUTPUT_LO(triggerOutput, triggerBit);           // End pulse

#if ONE_PIN_MODE
    *triggerMode &= ~triggerBit;   // Set trigger pin to input (when in one-pin mode this is setting the echo pin to input as both trigger & echo are tied together).
#endif

    // Wait for the echo pin to go high, which indicates the ping has started
    auto now = micros();

    for (auto timeout = now + MAX_SENSOR_DELAY; now <= timeout; now = micros())
    {
        if (IS_HI(echoInput, echoBit))  // Detected ping start
        {
            _pingStartTime = now - 4;   // subtract 4us to account for micros() overhead
            return true;
        }
    }

    TRACE(Logger(_classname_, this) << F("TriggerPing: timeout waiting for echo pin to go high") << endl);
    return false;
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
        TRACE(Logger(_classname_, this) << F("Ping failed to trigger") << endl);
        return PING_FAILED;
    }

    uint32_t now = micros();

    // Wait for echo pin to go low (or a timeout occurs)
    for (auto timeout = now + MAX_PING; IS_HI(echoInput, echoBit); now = micros())
    {
        if (now > timeout)
        {
            TRACE(Logger(_classname_, this) << F("------------ timed out ------------") << endl);
            break;
        }
    }

    // Calculate ping time, minus 5uS of overhead.
    _lastPing = udiff(now, _pingStartTime) - 5;
    TRACE(Logger(_classname_, this) << F("Ping=") << _lastPing << F("us") << endl);

    return (_lastPing > MIN_PING) ? _lastPing : PING_FAILED;
}


//******************************************************************************
// Take an ultrasonic sensor sample over 3 pings. This method takes 3 pings and
// averages the two that are closest together (or, equivalently, throws out the 
// most outlying ping).
//
// Returns: The average ping distance in centimeters, if successful. Otherwise,
//          returns PING_FAILED (0xFFFF).
//******************************************************************************
uint16_t SonarSensor::MultiPing()
{
    TRACE(Logger(_classname_, F("MultiPing")) << endl);

    int32_t pings[3];
    auto    timeout = millis() + 150;

    for (auto count = 0, failCount = 0; count < 3;)
    {
        uint16_t ping = PING_FAILED;

        if (Ready())
            ping = Ping();
        else if (millis() < timeout)
            continue;

        // if we got a good ping
        if (ping != PING_FAILED)
        {
            pings[count++] = ping;
            failCount = 0;
        }
        else
        {
            TRACE(Logger(_classname_, F("MultiPing")) << F("Ping Failed ") << ping << endl);
            if (++failCount >= 5) return PING_FAILED;
        }

        timeout = millis() + 150;
    }
    
    // Have the pings "vote" - the two pings that are closest to the average win
    auto avg_ping = (pings[0] + pings[1] + pings[2]) / 3;

    int32_t ds[3];
    
    ds[0] = abs(pings[0] - avg_ping);
    ds[1] = abs(pings[1] - avg_ping);
    ds[2] = abs(pings[2] - avg_ping);

    auto idx_max = 0;

    if (ds[1] > ds[idx_max]) idx_max = 1;
    if (ds[2] > ds[idx_max]) idx_max = 2;

    int32_t sum = 0;

    if (idx_max != 0) sum += pings[0];
    if (idx_max != 1) sum += pings[1];
    if (idx_max != 2) sum += pings[2];

    uint16_t ping = PingTimeToCentimeters(sum / 2);
    
    TRACE(Logger(_classname_, F("MultiPing")) << F("ping[0]=")   << pings[0] << F(" ds[0]=") << ds[0]
                                              << F(", ping[1]=") << pings[1] << F(" ds[1]=") << ds[1]
                                              << F(", ping[2]=") << pings[2] << F(" ds[2]=") << ds[2]
                                              << endl);
    TRACE(Logger(_classname_, F("MultiPing")) << F("average ping=") << avg_ping << endl);
    TRACE(Logger(_classname_, F("MultiPing")) << F("Discarding ping ") << idx_max << endl);
    TRACE(Logger(_classname_, F("MultiPing")) << F("MultiPing, ping=") << ping << endl);

    return ping;
}


uint16_t SonarSensor::PingMedian(const uint8_t maxSamples)
{
    TRACE(Logger(_classname_, F("MultiPing")) << F("PingMedian") << endl);

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

