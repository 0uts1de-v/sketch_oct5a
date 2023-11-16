#include "constant.hpp"

#include "L298N.hpp"
#include "SR04.hpp"
#include "CdS.hpp"

//----------START pin numbers----------
const int SR04_01_TRIG = 2;
const int SR04_01_ECHO = 3;
const int SR04_02_TRIG = 4;
const int SR04_02_ECHO = 5;
const int SR04_03_TRIG = 6;
const int SR04_03_ECHO = 7;
// digital

const int L298N_IN1=8;
const int L298N_IN2=9;
const int L298N_ENA=10;
const int L298N_IN3=12;
const int L298N_IN4=13;
const int L298N_ENB=11;
// digital

const int CdS_01_PIN = 0;
const int CdS_02_PIN = 1;
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
SR04_ SR04_01, SR04_02, SR04_03;
CdS_ CdS_01, CdS_02;
const auto boot_time = millis();
double dist_01 = 0, dist_02 = 0, dist_03 = 0;
bool line_01 = false, line_02 = false;
//----------END global var----------

void setup() {
  if (DEBUG) Serial.begin(115200);

  L298N.attach(L298N_IN1, L298N_IN2, L298N_ENA, L298N_IN3, L298N_IN4, L298N_ENB);
  SR04_01.attach(SR04_01_TRIG, SR04_01_ECHO); // left
  SR04_02.attach(SR04_02_TRIG, SR04_02_ECHO); // right
  SR04_03.attach(SR04_03_TRIG, SR04_03_ECHO); // front
  CdS_01.attach(CdS_01_PIN); // left
  CdS_02.attach(CdS_02_PIN); // right
}

void loop() {

  if (DEBUG) {
    char s[128];
    sprintf(s, "CdS_left: %s  CdS_right: %s  ", line_01 ? "true" : "false", line_02 ? "true" : "false");
    Serial.print(s);
    Serial.print("SR04_left: ");
    Serial.print(dist_01);
    Serial.print("  SR04_right: ");
    Serial.print(dist_02);
    Serial.print("  SR04_front: ");
    Serial.print(dist_03);
    Serial.print("\n");
  }
  
  if (TESTMODE) testrun();

  line_01 = CdS_01.get_onblackline(); // left
  line_02 = CdS_02.get_onblackline(); // right
  if (line_01 == true || line_02 == true) {
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
      dist_03 = SR04_03.get_distance(); // front
      if (dist_03 > 0 && dist_03 < 150) {
        obstacle();
        break;
      }
    }

    line_01 = CdS_01.get_onblackline(); // left
    line_02 = CdS_02.get_onblackline(); // right
    if (line_01 == true && line_02 == true) {
      L298N.move_front(SPEED);
      lost_flag = false;
    }
    else if (line_01 == true) {
      L298N.right_wheel(SPEED);
      L298N.left_wheel(SPEED * 0.5);
      lr = 0;
      lost_flag = false;
    }
    else if (line_02 == true) {
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
      sprintf(s, "CdS_left: %s  CdS_right: %s  ", line_01 ? "true" : "false", line_02 ? "true" : "false");
      Serial.print(s);
      Serial.print("SR04_left: ");
      Serial.print(dist_01);
      Serial.print("  SR04_right: ");
      Serial.print(dist_02);
      Serial.print("  SR04_front: ");
      Serial.print(dist_03);
      Serial.print("\n");
    }
  }
}

void ultrasonic() {
  dist_01 = SR04_01.get_distance(); // left
  dist_02 = SR04_02.get_distance(); // right
  dist_03 = SR04_03.get_distance(); // front

  

  if (dist_01 < 0) dist_01 = 999;
  if (dist_02 < 0) dist_02 = 999;
  if (dist_03 < 0) dist_03 = 999;


  if (dist_03 < 100) {
    L298N.turn_right(SPEED);
  }


  if (dist_01 < dist_02) {
    L298N.left_wheel(SPEED * 0.8);
    L298N.right_wheel(SPEED * 0.4);
  }
  else {
    L298N.left_wheel(SPEED * 0.4);
    L298N.right_wheel(SPEED * 0.8);
  }

  // if (dist_01 < dist_02) {
  //   if (dist_01 < 100) {
  //     L298N.left_wheel(SPEED * 0.8);
  //     L298N.right_wheel(0);
  //   }
  //   else {
  //     L298N.left_wheel(0);
  //     L298N.right_wheel(SPEED * 0.8);
  //   }
  // }
  // else {
  //   if (dist_02 < 100) {
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
    if (SR04_03.get_distance() > 150) break;
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
