#include "constant.hpp"

#include <Wire.h>
#include <VL53L1X.h>
#include "L298N.hpp"
#include "CdS.hpp"

//----------START pin numbers----------
const int VL53L1X_count = 3;
const int VL53L1X_XSHUT[VL53L1X_count] = {2, 3, 4};
// digital

const int L298N_IN1=8;
const int L298N_IN2=9;
const int L298N_ENA=10;
const int L298N_IN3=12;
const int L298N_IN4=13;
const int L298N_ENB=11;
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
//----------END prototype----------

//----------START global var----------
const auto boot_time = millis();
L298N_ L298N;
VL53L1X VL53L1X_[VL53L1X_count];
CdS_ CdS[CdS_count];
uint16_t dist[VL53L1X_count] = {0, 0, 0};
bool line[2] = {false, false};
//----------END global var----------

void setup() {
  if (DEBUG) {
    Serial.begin(115200);
    Serial.println("Setup begin.");
  }
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  for (size_t i = 0; i < VL53L1X_count; ++i) {
    pinMode(VL53L1X_XSHUT[i], OUTPUT);
    digitalWrite(VL53L1X_XSHUT[i], LOW);
  }
  for (size_t i = 0; i < VL53L1X_count; ++i) {
    delay(50);
    // pinMode(VL53L1X_XSHUT[i], INPUT);
    digitalWrite(VL53L1X_XSHUT[i], HIGH);
    delay(50);
    VL53L1X_[i].setTimeout(50);
    VL53L1X_[i].init();
    if (!VL53L1X_[i].init()) {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      Serial.print("Retrying...");
      asm volatile ("  jmp 0");
    }
    VL53L1X_[i].setAddress(0x2A + i);
    VL53L1X_[i].setDistanceMode(VL53L1X::Medium);
    VL53L1X_[i].setMeasurementTimingBudget(20000);
    VL53L1X_[i].startContinuous(20);
  }

  L298N.attach(L298N_IN1, L298N_IN2, L298N_ENA, L298N_IN3, L298N_IN4, L298N_ENB);
  
  for (size_t i = 0; i < CdS_count; ++i) {
    CdS[i].attach(CdS_PIN[i]);
  }

  Serial.print("Setup done.");  
}

void loop() {  
  if (TESTMODE) testrun();

  for (size_t i = 0; i < CdS_count; ++i) {
    line[i] = CdS[i].get_onblackline(); // l, r
  }
  
  if (line[0] == true || line[1] == true) {
    linetrace();
  }
  else {
    noline();
  }

  if (DEBUG) {
    String s;
    for (size_t i = 0; i < CdS_count; ++i) {
      s += "CdS_" + String(i) + line[0] ? "true" : "false";
    }
    for (size_t i = 0; i < VL53L1X_count; ++i) {
      s += "VL53L1X_" + String(i) + String(dist[i]);
    }
    Serial.println(s);
  }

  delay(DELAY);
}

void linetrace() {
  unsigned long lost_time = 0, d = 0, last_d = 0;
  bool lr = 0; // 0: left  1: right
  bool lost_flag = false;
  while (true) {
    d = (millis() - boot_time) / 250;
    if (last_d == 0) last_d = d;
    if (d != last_d) {
      dist[2] = VL53L1X_[2].read(); // front
      if (dist[2] > 0 && dist[2] < 150) {
        obstacle();
        break;
      }
    }

    for (size_t i = 0; i < CdS_count; ++i) {
      line[i] = CdS[i].get_onblackline(); // l, r
    }
    if (line[0] == true && line[1] == true) {
      L298N.move_front(SPEED);
      lost_flag = false;
    }
    else if (line[0] == true) {
      L298N.right_wheel(SPEED);
      L298N.left_wheel(SPEED * 0.5);
      lr = 0;
      lost_flag = false;
    }
    else if (line[1] == true) {
      L298N.left_wheel(SPEED);
      L298N.right_wheel(SPEED * 0.5);
      lr = 1;
      lost_flag = false;
    }
    else {
      if (lost_flag == false) lost_time = millis();
      lost_flag = true;
      if (lost_time - millis() > 2000) break;
      if (lr == 0) {
        L298N.turn_left(SPEED);
      }
      else {
        L298N.turn_right(SPEED);
      }
    }
    if (DEBUG) {
      String s;
      for (size_t i = 0; i < CdS_count; ++i) {
        s += "CdS_" + String(i) + line[0] ? "true" : "false";
      }
      for (size_t i = 0; i < VL53L1X_count; ++i) {
        s += "VL53L1X_" + String(i) + String(dist[i]);
      }
      Serial.println(s);
    }
  }
}

void noline() {
  bool first = true;
  const double Kp = 20, Ki = 0.5, Kd = 0.5;
  unsigned long t[2] = {0, 0};
  long d[2] = {0, 0};
  double integral = 0, control = 0;
  
  while (!CdS[0].get_onblackline() && !CdS[1].get_onblackline()) {
    d[1] = d[0];
    t[1] = t[0];
    t[0] = millis();

    for (size_t i = 0; i < VL53L1X_count; ++i) {
      dist[i] = VL53L1X_[i].read(); // l, r, f
      if (dist[i] < 0) dist[i] = 999;
    }
  
    if (dist[2] < 50) { // 前方障害物で後退
      L298N.move_back(SPEED);
    }
    else {
      L298N.move_front(SPEED);
    }

    d[0] = dist[0] - dist [1];
    control = 0;
    control += Kp * d[0];
    if (!first) {
      integral += (d[0] + d[1]) * (t[0] - t[1]) / 2;
      control += Ki * (d[0] + d[1]) * (t[0] - t[1]) / 2;
      control += Kd * (d[0] - d[1]) / (t[0] - t[1]);
    }

    if (dist[0] < dist[1]) {
      L298N.left_wheel(SPEED * control);
      L298N.right_wheel(0);
    }
    else {
      L298N.right_wheel(SPEED * control);
      L298N.left_wheel(0);
    }
    first = false;
  }
}

void obstacle() {
  while (VL53L1X_[0].read() > 150) {
    L298N.turn_right(SPEED);
  }
  while (!CdS[0].get_onblackline() && !CdS[1].get_onblackline()) {
    while (VL53L1X_[0].read() < 150) {
      L298N.move_front(SPEED);
    }
    while (VL53L1X_[0].read() > 150) {
      L298N.turn_left(SPEED);
    }
  }
}

void testrun() {
  while (true)
  {
    auto d = millis() - boot_time;
    if ((d / 2000)%2) L298N.turn_left(SPEED);
    else L298N.turn_right(SPEED);
  }  
}
