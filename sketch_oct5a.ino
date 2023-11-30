#include "constant.hpp"

#include <Wire.h>
#include <VL53L1X.h>
#include "L298N.hpp"
#include "CdS.hpp"

//----------START pin numbers----------
const int VL53L1X_count=3;
const int VL53L1X_XSHUT[VL53L1X_count] = {2, 3, 4};
// digital

const int L298N_IN1=8;
const int L298N_IN2=9;
const int L298N_ENA=10;
const int L298N_IN3=12;
const int L298N_IN4=13;
const int L298N_ENB=11;
// digital

const int CdS_PIN[2] = {0, 1};
// analog
//----------END pin numbers----------

//----------START prototype----------
void linetrace();
void ultrasonic();
void obstacle();
void testrun();
//----------END prototype----------

//----------START global var----------
L298N_ L298N;
VL53L1X VL53L1X_[VL53L1X_count];  // 命名規則が...  おのれ VL53L1X.h
CdS_ CdS[2];
const auto boot_time = millis();
uint16_t dist[VL53L1X_count] = {0, 0, 0};
bool line[2] = {false, false};
//----------END global var----------

void setup() {
  if (DEBUG) Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  for (size_t i = 0; i < VL53L1X_count; ++i) {
    pinMode(VL53L1X_XSHUT[i], OUTPUT);
    digitalWrite(VL53L1X_XSHUT[i], LOW);
  }
  for (size_t i = 0; i < VL53L1X_count; ++i) {
    delay(50);
    Serial.print("a");
    // pinMode(VL53L1X_XSHUT[i], INPUT);
    digitalWrite(VL53L1X_XSHUT[i], HIGH);
    delay(50);
    Serial.print("b");
    VL53L1X_[i].setTimeout(50);
    VL53L1X_[i].init();
    if (!VL53L1X_[i].init()) {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (true){};
    }
    VL53L1X_[i].setAddress(0x2A + i);
    VL53L1X_[i].setDistanceMode(VL53L1X::Medium);
    VL53L1X_[i].setMeasurementTimingBudget(20000);
    VL53L1X_[i].startContinuous(20);
  }

  L298N.attach(L298N_IN1, L298N_IN2, L298N_ENA, L298N_IN3, L298N_IN4, L298N_ENB);
  
  for (size_t i = 0; i < 2; ++i) {
    CdS[i].attach(CdS_PIN[i]);
  }

  Serial.print("setup done.");  
}

void loop() {

  if (DEBUG) {
    // char s[128];
    // sprintf(s, "CdS_left: %s  CdS_right: %s  ", line[0] ? "true" : "false", line[1] ? "true" : "false");
    // Serial.print(s);
    // Serial.print("SR04_left: ");
    // Serial.print(dist[0]);
    // Serial.print("  SR04_right: ");
    // Serial.print(dist[1]);
    // Serial.print("  SR04_front: ");
    // Serial.print(dist[2]);
    // Serial.print("\n");
    for (size_t i = 0; i < VL53L1X_count; ++i) {
      Serial.println(VL53L1X_[i].read());
    }
    
  }
  
  if (TESTMODE) testrun();

  line[0] = CdS[0].get_onblackline(); // left
  line[1] = CdS[1].get_onblackline(); // right
  if (line[0] == true || line[1] == true) {
    linetrace();
  }
  else {
    ultrasonic();
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

    line[0] = CdS[0].get_onblackline(); // left
    line[1] = CdS[1].get_onblackline(); // right
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
      char s[128];
      sprintf(s, "CdS_left: %s  CdS_right: %s  ", line[0] ? "true" : "false", line[1] ? "true" : "false");
      Serial.print(s);
      for (size_t i = 0; i < VL53L1X_count; ++i) {
        Serial.println(VL53L1X_[i].read());
      }
    }
  }
}

void ultrasonic() {
  dist[0] = VL53L1X_[0].read(); // left
  dist[1] = VL53L1X_[1].read(); // right
  dist[2] = VL53L1X_[2].read(); // front

  

  if (dist[0] < 0) dist[0] = 999;
  if (dist[1] < 0) dist[1] = 999;
  if (dist[2] < 0) dist[2] = 999;


  if (dist[2] < 100) {
    L298N.turn_right(SPEED);
  }


  if (dist[0] < dist[1]) {
    L298N.left_wheel(SPEED * 0.8);
    L298N.right_wheel(SPEED * 0.4);
  }
  else {
    L298N.left_wheel(SPEED * 0.4);
    L298N.right_wheel(SPEED * 0.8);
  }

  // if (dist[0] < dist[1]) {
  //   if (dist[0] < 100) {
  //     L298N.left_wheel(SPEED * 0.8);
  //     L298N.right_wheel(0);
  //   }
  //   else {
  //     L298N.left_wheel(0);
  //     L298N.right_wheel(SPEED * 0.8);
  //   }
  // }
  // else {
  //   if (dist[1] < 100) {
  //     L298N.left_wheel(0);
  //     L298N.right_wheel(SPEED * 0.8);
  //   }
  //   else {
  //     L298N.left_wheel(SPEED * 0.8);
  //     L298N.right_wheel(0);
  //   }
  // }
  
}

void obstacle() {
  while (true)
  {
    L298N.stop();
    if (VL53L1X_[2].read() > 150) break;
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
