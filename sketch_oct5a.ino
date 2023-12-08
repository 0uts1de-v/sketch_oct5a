#include "constant.hpp"

#include <Wire.h>
#include "dist_sensor.hpp"
#include "L298N.hpp"
#include "CdS.hpp"

//----------START pin numbers----------
const int dist_sensor_count = 3;
const int SR04_PIN[dist_sensor_count * 2] = {2, 3, 4, 5, 6, 7};
const int VL53L1X_XSHUT[dist_sensor_count] = {2, 3, 4};
// digital

const int L298N_IN1 = 8;
const int L298N_IN2 = 9;
const int L298N_ENA = 10;
const int L298N_IN3 = 12;
const int L298N_IN4 = 13;
const int L298N_ENB = 11;
// digital

const int CdS_count = 2;
const int CdS_PIN[CdS_count] = {0, 1};
// analog
//----------END pin numbers----------

//----------START prototype----------
void linetrace();
void noline();
void obstacle();
void testrun();
void print_debug();
//----------END prototype----------

//----------START global var----------
const auto boot_time = millis();
L298N_ L298N;
DIST_SENSOR_ DIST_SENSOR[dist_sensor_count];
// VL53L1X VL53L1X_[VL53L1X_count];
CdS_ CdS[CdS_count];
uint16_t dist[dist_sensor_count] = {0, 0, 0};
bool line[CdS_count] = {false, false};
//----------END global var----------

void setup()
{
    Serial.begin(115200);
    if (DEBUG)
        Serial.println("Setup begin.");

    for (size_t i = 0; i < dist_sensor_count; ++i)
    {
        if (DISTANCE_SENSOR_MODE == 0)
            DIST_SENSOR[i].attach(SR04_PIN[i * 2], SR04_PIN[i * 2 + 1]);
        else if (DISTANCE_SENSOR_MODE == 1)
            DIST_SENSOR[i].attach(VL53L1X_XSHUT[i]);
    }
    if (DISTANCE_SENSOR_MODE == 1)
        VL53L1X_setup(DIST_SENSOR, dist_sensor_count);

    L298N.attach(L298N_IN1, L298N_IN2, L298N_ENA, L298N_IN3, L298N_IN4, L298N_ENB);

    for (size_t i = 0; i < CdS_count; ++i)
    {
        CdS[i].attach(CdS_PIN[i]);
    }

    if (DEBUG)
        Serial.println("Setup done.");
}

void loop()
{
    if (TESTMODE)
        testrun();

    for (size_t i = 0; i < CdS_count; ++i)
    {
        line[i] = CdS[i].get_onblackline(); // l, r
    }

    if (line[0] == true || line[1] == true)
    {
        linetrace();
    }
    else
    {
        noline();
    }

    print_debug();
}

void linetrace()
{
    unsigned long lost_time = 0, d = 0, last_d = 0;
    bool lr = 0; // 0: left  1: right
    bool lost_flag = false;
    while (true)
    {
        d = (millis() - boot_time) / 250;
        if (last_d == 0)
            last_d = d;
        if (d != last_d)
        {
            // dist[2] = VL53L1X_[2].read(); // front
            if (dist[2] > 0 && dist[2] < 150)
            {
                obstacle();
                break;
            }
        }

        for (size_t i = 0; i < CdS_count; ++i)
        {
            line[i] = CdS[i].get_onblackline(); // l, r
        }
        if (line[0] == true && line[1] == true)
        {
            L298N.move_front(SPEED);
            lost_flag = false;
        }
        else if (line[0] == true)
        {
            L298N.right_wheel(SPEED);
            L298N.left_wheel(SPEED * 0.5);
            lr = 0;
            lost_flag = false;
        }
        else if (line[1] == true)
        {
            L298N.left_wheel(SPEED);
            L298N.right_wheel(SPEED * 0.5);
            lr = 1;
            lost_flag = false;
        }
        else
        {
            if (lost_flag == false)
                lost_time = millis();
            lost_flag = true;
            if (lost_time - millis() > 2000)
                break;
            if (lr == 0)
            {
                L298N.turn_left(SPEED);
            }
            else
            {
                L298N.turn_right(SPEED);
            }
        }

        print_debug();
    }
}

void noline()
{
    bool first = true;
    const double Kp = 20, Ki = 0.5, Kd = 0.5;
    unsigned long t[2] = {0, 0};
    long d[2] = {0, 0};
    double integral = 0, control = 0;

    while (!CdS[0].get_onblackline() && !CdS[1].get_onblackline())
    {
        for (size_t i = 0; i < dist_sensor_count; ++i)
        {
            dist[i] = DIST_SENSOR[i].get_distance(); // l, r, f
            if (dist[i] < 0)
                dist[i] = 999;
        }

        if (dist[2] < 50)
        { // 前方障害物で後退
            L298N.move_back(SPEED);
        }
        else
        {
            L298N.move_front(SPEED);
        }

        d[1] = d[0];
        t[1] = t[0];
        t[0] = millis();
        d[0] = dist[0] - dist[1];
        control = 0;
        control += Kp * d[0];
        if (!first)
        {
            integral += (d[0] + d[1]) * (t[0] - t[1]) / 2;
            control += Ki * (d[0] + d[1]) * (t[0] - t[1]) / 2;
            control += Kd * (d[0] - d[1]) / (t[0] - t[1]);
        }

        if (control > 1)
            control = 1;

        if (dist[0] < dist[1])
        {
            L298N.left_wheel(SPEED * control);
            L298N.right_wheel(0);
        }
        else
        {
            L298N.right_wheel(SPEED * control);
            L298N.left_wheel(0);
        }
        first = false;

        print_debug();
    }
}

void obstacle()
{
    while (DIST_SENSOR[0].get_distance() > 150)
    {
        L298N.turn_right(SPEED);
        print_debug();
    }
    while (!CdS[0].get_onblackline() && !CdS[1].get_onblackline())
    {
        while (DIST_SENSOR[0].get_distance() < 150)
        {
            L298N.move_front(SPEED);
            print_debug();
        }
        while (DIST_SENSOR[0].get_distance() > 150)
        {
            L298N.turn_left(SPEED);
            print_debug();
        }
    }
}

void testrun()
{
    while (true)
    {
        auto d = millis() - boot_time;
        if ((d / 2000) % 2)
            L298N.turn_left(SPEED);
        else
            L298N.turn_right(SPEED);
        print_debug();
    }
}

void print_debug()
{
    if (DEBUG)
    {
        String s;
        for (size_t i = 0; i < CdS_count; ++i)
        {
            s += "CdS_" + String(i) + line[i] ? "true  " : "false ";
        }
        for (size_t i = 0; i < dist_sensor_count; ++i)
        {
            s += "\tdist_" + String(i) + ": " + String(dist[i]);
        }
        Serial.println(s);
    }
}