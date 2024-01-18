#pragma once

#include <VL53L1X.h>
#include <Wire.h>
#include "SR04.hpp"
#include "constant.hpp"

bool flag_for_VL53L1X_setup_ = false;

class DIST_SENSOR_;
void VL53L1X_setup(DIST_SENSOR_ *, const int);

class DIST_SENSOR_
{
    friend void VL53L1X_setup(DIST_SENSOR_ *, const int);

public:
    DIST_SENSOR_(){};
    DIST_SENSOR_(int PIN_XSHUT)
    {
        if (DISTANCE_SENSOR_MODE != 1)
            return;
    }
    DIST_SENSOR_(int PIN_TRIG, int PIN_ECHO)
    {
        if (DISTANCE_SENSOR_MODE != 0)
            return;
        this->SR04_attach_(PIN_TRIG, PIN_ECHO);
    }
    ~DIST_SENSOR_(){};

    void attach(int PIN_XSHUT)
    {
        if (DISTANCE_SENSOR_MODE != 1)
            return;
        this->VL53L1X_attach_(PIN_XSHUT);
    }

    void attach(int PIN_TRIG, int PIN_ECHO)
    {
        if (DISTANCE_SENSOR_MODE != 0)
            return;
        this->SR04_attach_(PIN_TRIG, PIN_ECHO);
    }

    double get_distance()
    {
        if (DISTANCE_SENSOR_MODE == 0)
            return this->SR04_get_distance_();
        else if (DISTANCE_SENSOR_MODE == 1)
            return this->VL53L1X_.read();
    }

private:
    int TRIG_, ECHO_;

    void SR04_(int TRIG, int ECHO)
    {
        this->SR04_attach_(TRIG, ECHO);
    }

    void SR04_attach_(int TRIG, int ECHO)
    {
        TRIG_ = TRIG;
        ECHO_ = ECHO;
        this->SR04_setup_();
    }

    double SR04_get_distance_()
    {
        // return distance in [mm]
        // if failed return -1
        double duration = 0;
        double distance = 0;

        digitalWrite(TRIG_, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_, LOW);
        duration = pulseIn(ECHO_, HIGH, TIMEOUT);

        if (duration > 0)
        {
            duration /= 2;
            distance = duration * speed_of_sound * 1000 / 1000000;
        }
        else
        {
            distance = -1;
        }

        return distance;
    }

    void SR04_setup_()
    {
        pinMode(TRIG_, OUTPUT);
        pinMode(ECHO_, INPUT);
    }

    int XSHUT_;
    VL53L1X VL53L1X_;
    void VL53L1X_attach_(int XSHUT)
    {
        XSHUT_ = XSHUT;
    }
};

void VL53L1X_setup(DIST_SENSOR_ *DIST_SENSOR, const int VL53L1X_count)
{
    if (DISTANCE_SENSOR_MODE != 1)
    {
        return;
    }

    Wire.begin();

    if (DEBUG)
        Serial.println("I2C started.");

    for (size_t i = 0; i < VL53L1X_count; ++i)
    {
        pinMode(DIST_SENSOR[i].XSHUT_, OUTPUT);
        digitalWrite(DIST_SENSOR[i].XSHUT_, LOW);
    }

    if (DEBUG)
        Serial.println("All sensors shutted down.");

    for (size_t i = 0; i < VL53L1X_count; ++i)
    {
        digitalWrite(DIST_SENSOR[i].XSHUT_, HIGH);
        // pinMode(DIST_SENSOR[i].XSHUT_, INPUT);
        delay(10);
        DIST_SENSOR[i].VL53L1X_.setTimeout(500);
        if (DIST_SENSOR[i].VL53L1X_.init() == false)
        {
            if (DEBUG)
            {
                Serial.print("Failed to detect and initialize sensor ");
                Serial.println(i);
                Serial.println("Retrying...");
            }
            delay(100);
            asm volatile("  jmp 0");
        }
        else
        {
            if (DEBUG)
            {
                Serial.print("Initialized VL53L1X_");
                Serial.println(i);
            }
        }
        DIST_SENSOR[i].VL53L1X_.setAddress(0x2A + i * 2);
        // VL53L1X_.setDistanceMode(VL53L1X::Long);
        // VL53L1X_.setMeasurementTimingBudget(5000);
        DIST_SENSOR[i].VL53L1X_.startContinuous(50);
    }
}