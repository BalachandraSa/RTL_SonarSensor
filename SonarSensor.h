#ifndef _SonarSensor_h_
#define _SonarSensor_h_

// ---------------------------------------------------------------------------
// SonarSensor Library - v1.5 - 08/15/2012
//
// AUTHOR/LICENSE:
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2012 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
//
// LINKS:
// Project home: http://code.google.com/p/arduino-new-ping/
// Blog: http://arduino.cc/forum/index.php/topic,106043.0.html
//
// DISCLAIMER:
// This software is furnished "as is", without technical support, and with no 
// warranty, express or implied, as to its usefulness for any purpose.
//
// BACKGROUND:
// When I first received an ultrasonic sensor I was not happy with how poorly
// it worked. Quickly I realized the problem wasn't the sensor, it was the
// available ping and ultrasonic libraries causing the problem. The SonarSensor
// library totally fixes these problems, adds many new features, and breaths
// new life into these very affordable distance sensors. 
//
// FEATURES:
// * Works with many different ultrasonic sensor models: SR04, SRF05, SRF06, DYP-ME007 & Parallax PING)))™.
// * Interface with all but the SRF06 sensor using only one Arduino pin.
// * Doesn't lag for a full second if no ping/echo is received.
// * Ping sensors consistently and reliably at up to 30 times per second.
// * Timer interrupt method for event-driven sketches.
// * Built-in digital filter method ping_median() for easy error correction.
// * Uses port registers for a faster pin interface and smaller code size.
// * Allows you to set a maximum distance where pings beyond that distance are read as no ping "clear".
// * Ease of using multiple sensors (example sketch with 15 sensors).
// * More accurate distance calculation (cm, inches & uS).
// * Doesn't use pulseIn, which is slow and gives incorrect results with some ultrasonic sensor models.
// * Actively developed with features being added and bugs/issues addressed.
//
// CONSTRUCTOR:
//   SonarSensor sonar(triggerPin, echoPin [, max_cm_distance])
//     triggerPin & echoPin - Arduino pins connected to sensor trigger and echo.
//       NOTE: To use the same Arduino pin for trigger and echo, specify the same pin for both values.
//     max_cm_distance - [Optional] Maximum distance you wish to sense. Default=500cm.
//
// SYNTAX:
//   sonar.ping() - Send a ping and get the echo time (in microseconds) as a result. 
//   sonar.ping_in() - Send a ping and get the distance in whole inches.
//   sonar.ping_cm() - Send a ping and get the distance in whole centimeters.
//   sonar.ping_median(iterations) - Do multiple pings (default=5), discard out of range pings and return median in microseconds. 
//   sonar.convert_in(echoTime) - Convert echoTime from microseconds to inches (rounds to nearest inch).
//   sonar.convert_cm(echoTime) - Convert echoTime from microseconds to centimeters (rounds to nearest cm).
//   sonar.ping_timer(function) - Send a ping and call function to test if ping is complete.
//   sonar.check_timer() - Check if ping has returned within the set distance limit.
//   NewPing::timer_us(frequency, function) - Call function every frequency microseconds.
//   NewPing::timer_ms(frequency, function) - Call function every frequency milliseconds.
//   NewPing::timer_stop() - Stop the timer.
//
// HISTORY:
// 08/15/2012 v1.5 - Added ping_median() method which does a user specified
//   number of pings (default=5) and returns the median ping in microseconds
//   (out of range pings ignored). This is a very effective digital filter.
//   Optimized for smaller compiled size (even smaller than sketches that
//   don't use a library).
//
// 07/14/2012 v1.4 - Added support for the Parallax PING)))™ sensor. Interface
//   with all but the SRF06 sensor using only one Arduino pin. You can also
//   interface with the SRF06 using one pin if you install a 0.1uf capacitor
//   on the trigger and echo pins of the sensor then tie the trigger pin to
//   the Arduino pin (doesn't work with Teensy). To use the same Arduino pin
//   for trigger and echo, specify the same pin for both values. Various bug
//   fixes.
//
// 06/08/2012 v1.3 - Big feature addition, event-driven ping! Uses Timer2
//   interrupt, so be mindful of PWM or timing conflicts messing with Timer2
//   may cause (namely PWM on pins 3 & 11 on Arduino, PWM on pins 9 and 10 on
//   Mega, and Tone library). Simple to use timer interrupt functions you can
//   use in your sketches totaly unrelated to ultrasonic sensors (don't use if
//   you're also using NewPing's ping_timer because both use Timer2 interrupts).
//   Loop counting ping method deleted in favor of timing ping method after
//   inconsistant results kept surfacing with the loop timing ping method.
//   Conversion to cm and inches now rounds to the nearest cm or inch. Code
//   optimized to save program space and fixed a couple minor bugs here and
//   there. Many new comments added as well as line spacing to group code
//   sections for better source readability.
//
// 05/25/2012 v1.2 - Lots of code clean-up thanks to Arduino Forum members.
//   Rebuilt the ping timing code from scratch, ditched the pulseIn code as it
//   doesn't give correct results (at least with ping sensors). The NewPing
//   library is now VERY accurate and the code was simplified as a bonus.
//   Smaller and faster code as well. Fixed some issues with very close ping
//   results when converting to inches. All functions now return 0 only when
//   there's no ping echo (out of range) and a positive value for a successful
//   ping. This can effectively be used to detect if something is out of range
//   or in-range and at what distance. Now compatible with Arduino 0023.
//
// 05/16/2012 v1.1 - Changed all I/O functions to use low-level port registers
//   for ultra-fast and lean code (saves from 174 to 394 bytes). Tested on both
//   the Arduino Uno and Teensy 2.0 but should work on all Arduino-based
//   platforms because it calls standard functions to retrieve port registers
//   and bit masks. Also made a couple minor fixes to defines.
//
// 05/15/2012 v1.0 - Initial release.
// ---------------------------------------------------------------------------

#include <inttypes.h>
#include <RTL_Stdlib.h>
#include <RTL_Math.h>

// Probably shouldn't change these values unless you really know what you're doing.
#define MIN_DISTANCE 2          // Minimum measureable distance in centimeters
#define MAX_DISTANCE 400        // Maximum measureable distance in centimeters, no reason to wait for ping longer than sound takes to travel this distance and back.
#define US_ROUNDTRIP_IN 146     // Microseconds it takes sound to travel round-trip 1 inch (2 inches total), uses integer to save compiled code space.
#define US_ROUNDTRIP_CM 57      // Microseconds it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space.
#define PING_FAILED 0xFFFF      // Value returned if ping failed to trigger.
#define MIN_PING (MIN_DISTANCE * US_ROUNDTRIP_CM) // Minimum ping duration in microseconds
#define MAX_PING (MAX_DISTANCE * US_ROUNDTRIP_CM) // Maximum ping duration in microseconds


// Conversion from microseconds to distance (round result to nearest cm or inch).
#define PingTimeToDistance(echoTime, conversionFactor) (max((echoTime + conversionFactor / 2) / conversionFactor, (echoTime ? 1 : 0)))
#define PingTimeToCentimeters(echoTime) (((uint16_t)(echoTime) + US_ROUNDTRIP_CM / 2) / US_ROUNDTRIP_CM)
#define PingTimeToInches(echoTime)      (((uint16_t)(echoTime) + US_ROUNDTRIP_IN / 2) / US_ROUNDTRIP_IN)

// Sensor can be connected to Arduino in either one-pin or two-pin mode (default).
// Two-pin mode: The sensor trigger and echo pins are connected to seperate pins on the Arduino.
//               The Arduino trigger pin is always in output mode and the Arduino echo pin is always in input mode.   
// One-pin mode: The sensor trigger and echo pins are physically tied together and connected to one pin on the Arduino.
//               The Arduino pin needs to be switched to output mode when triggering and to input mode when checking for echo 
#define TWO_PIN_MODE true  // Set to "true" for two-pin mode, false for one-pin mode
#define ONE_PIN_MODE (!TWO_PIN_MODE)


class SonarSensor
{
    DECLARE_CLASSNAME;

    /*--------------------------------------------------------------------------
    Constants
    --------------------------------------------------------------------------*/

    
    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: SonarSensor(uint8_t triggerPin, uint8_t echoPin);

    
    /*--------------------------------------------------------------------------
    Public methods
    --------------------------------------------------------------------------*/
    
    //**************************************************************************
    /// Performs a single ultrasonic sensor measurement, returning ping time in 
    /// microseconds. 
    ///
    /// Returns: If successful, returns the microseconds taken for the ping, which 
    ///          can be converted to centimeters with PingTimeToCentimeters. If 
    ///          the ping failed then PING_FAILED is returned (0xFFFF).
    //**************************************************************************
    public: uint16_t Ping();

    //**************************************************************************
    /// Performs a single ultrasonic sensor measurement, returning ping distance 
    /// in centimeters. 
    ///
    /// Returns: If successful, returns the ping distance in centimeters. If the 
    ///          ping failed then PING_FAILED is returned (0xFFFF).
    //**************************************************************************
    public: uint16_t PingCentimeters() { auto ping = Ping(); return ping == PING_FAILED ? PING_FAILED : PingTimeToCentimeters(ping); };

    //**************************************************************************
    // Take an ultrasonic sensor sample over 3 pings. This method takes 3 pings and
    // averages the two that are closest together (or, equivalently, throws out the 
    // most outlying ping).
    //
    // Returns: The average ping distance in centimeters, if successful. Otherwise,
    //          returns PING_FAILED (0xFFFF).
    //**************************************************************************
    public: uint16_t MultiPing();

    public: uint16_t PingMedian(const uint8_t maxSamples = 5);

    public: bool Ready();

    
    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    protected: bool TriggerPing();
    
    
    protected: uint8_t  _triggerPin;
    protected: uint8_t  _echoPin;
    protected: uint16_t _lastPing;
    protected: uint32_t _pingStartTime;
};

#endif
