#ifndef _CDS_HPP
#define _CDS_HPP

#include "constant.hpp"

class CdS_;

class CdS_ {
  public:
    CdS_() {};

    CdS_(int PIN) {
      this -> attach(PIN);
    }

    ~CdS_() {};

    void attach(int PIN) {
      PIN_ = PIN;
    }

    bool get_onblackline() {
      // on the blackline -> true
      double voltage = analogRead(PIN_) * 5 / 1023.0;
      // if (DEBUG) Serial.print(voltage);
      if (DEBUG) Serial.println(voltage);
      if (voltage <= CDS_TRESHOLD) {
        return true;
      }
      else {
        return false;
      }
    }
  private:
    int PIN_;
};

#endif