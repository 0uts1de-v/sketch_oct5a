#ifndef _SR04_HPP
#define _SR04_HPP

#include "constant.hpp"

const double speed_of_sound = 331.5 + 0.6 * TEMPERTURE; // 25℃

class SR04_;

class SR04_ {
  public:
    SR04_() {};

    SR04_(int TRIG, int ECHO) {
        this -> attach(TRIG, ECHO);
    }

    ~SR04_() {};

    void attach(int TRIG, int ECHO) {
      TRIG_ = TRIG;
      ECHO_ = ECHO;
      this -> setup();
    }

    double get_distance() {
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

      if (duration > 0) {
        duration /= 2;
        distance = duration * speed_of_sound * 1000 / 1000000;
      }
      else {
        distance = -1;
      }
      
      return distance;
    }
  private:
    int TRIG_, ECHO_;

    void setup() {
      pinMode(TRIG_, OUTPUT);
      pinMode(ECHO_, INPUT);
    }
};

#endif