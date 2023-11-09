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
void linetrace(bool, bool);
void ultrasonic(double, double, double);
void obstacle(bool, bool, double, double, double);
void testrun();
//----------END prototype----------

//----------START global var----------
L298N_ L298N;
SR04_ SR04_01, SR04_02, SR04_03;
CdS_ CdS_01, CdS_02;
const unsigned long boot_time = millis();
//----------END global var----------

void setup() {
  if (DEBUG) Serial.begin(115200);

  L298N.attach(L298N_IN1, L298N_IN2, L298N_ENA, L298N_IN3, L298N_IN4, L298N_ENB);
  SR04_01.attach(SR04_01_TRIG, SR04_01_ECHO);
  SR04_02.attach(SR04_02_TRIG, SR04_02_ECHO);
  SR04_03.attach(SR04_03_TRIG, SR04_03_ECHO);
  CdS_01.attach(CdS_01_PIN);
  CdS_02.attach(CdS_02_PIN);
}

void loop() {
  int mode = 0; // 0: linetrace, 1: ultrasonic, 2: obstacle, 3: test
  float dist_01 = 0, dist_02 = 0, dist_03 = 0;
  bool line_01 = false, line_02 = false; 
  dist_01 = SR04_01.get_distance(); // left
  dist_02 = SR04_02.get_distance(); // right
  dist_03 = SR04_03.get_distance(); // front
  line_01 = CdS_01.get_onblackline(); // left
  line_02 = CdS_02.get_onblackline(); // right
  
  if (line_01 == false && line_02 == false) {
    mode = 1; // ultrasonic
  }
  else if (dist_01 <= 50) { // とりあえず 50 mm
    mode = 2; // obstacle
  }
  else {
    mode = 0; // linetrace
  }

  if (TESTMODE) mode = 3;
  
  if (DEBUG) {
    char s[128];
    sprintf(s, "mode: %d  CdS_left: %s  CdS_right: %s  ", mode, line_01 ? "true" : "false", line_02 ? "true" : "false");
    Serial.print(s);
    Serial.print("SR04_left: ");
    Serial.print(dist_01);
    Serial.print("  SR04_right: ");
    Serial.print(dist_02);
    Serial.print("  SR04_front: ");
    Serial.print(dist_03);
    Serial.print("\n");
  }

  

  switch (mode) {
  case 0:
    linetrace(line_01, line_02);
    break;

  case 1:
    ultrasonic(dist_01, dist_02, dist_03);
    break;

  case 2:
    obstacle(line_01, line_02, dist_01, dist_02, dist_03);
    break;
  
  case 3:
    testrun();
    break;

  default:
    L298N.stop();
    break;
  }
  
  
  delay(DELAY);
}

void linetrace(bool left, bool right) {
  if (left == true && right == true) {
    L298N.move_front(SPEED);
  }
  else if (left == true) {
    L298N.left_wheel(SPEED);
    L298N.right_wheel(0);
  }
  else {
    L298N.right_wheel(SPEED);
    L298N.left_wheel(0);
  }
}

void ultrasonic(double left, double right, double front) {
  if (front > 50 && abs(left - right) < 50) {
    L298N.move_front(SPEED);
  }
  else if (front < 50 && left < right) {
    L298N.turn_right(SPEED);
  }
  else if (front < 50 && left > right) {
    L298N.turn_left(SPEED);
  }
  else if (front > 50 && left < right) {
    L298N.left_wheel(SPEED);
    L298N.right_wheel(SPEED * 0.75);
  }
  else if (front > 50 && left > right) {
    L298N.left_wheel(SPEED * 0.75);
    L298N.right_wheel(SPEED);
  }
  else {
    L298N.move_back(SPEED * 0.5);
  }
}

void obstacle(bool cds_left, bool cds_right, double us_front, double us_left, double us_right) {
  L298N.turn_right(SPEED);
  delay(500);
  L298N.move_front(SPEED);
  delay(1000);
  L298N.turn_left(SPEED);
  delay(500);
  L298N.move_front(SPEED);
  delay(1000);
  L298N.turn_left(SPEED);
  delay(500);
  L298N.move_front(SPEED);
  delay(1000);
  // 🍊成
}

void testrun() {
  auto d = millis() - boot_time;
  if ((d / 1000)%2) L298N.move_front(100);
  else L298N.move_back(100);
}
