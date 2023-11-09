#ifndef _L298N_HPP
#define _L298N_HPP

class L298N_;

class L298N_ {
  // speed: -100 ~ 100 [%]
  public:
      L298N_() {};

      L298N_(int IN1, int IN2, int ENA, int IN3, int IN4, int ENB) {
        this -> attach(IN1, IN2, ENA, IN3, IN4, ENB);
      };

      ~L298N_() {};

      void attach(int IN1, int IN2, int ENA, int IN3, int IN4, int ENB) {
        IN1_ = IN1;
        IN2_ = IN2;
        ENA_ = ENA;
        IN3_ = IN3;
        IN4_ = IN4;
        ENB_ = ENB;
        this -> setup();
      }

      void move_front(double speed) {
        left_wheel(speed);
        right_wheel(speed);
      }

      void move_back(double speed) {
        move_front(-1 * speed);
      }

      void turn_left(double speed) {
        left_wheel(-1 * speed);
        right_wheel(speed);
      }

      void turn_right(double speed) {
        turn_left(-1 * speed);
      }

      void stop() {
        move_front(0);
      }

      void left_wheel(double speed) {
        if (speed >= 0) {
            digitalWrite(IN1_, HIGH);
            digitalWrite(IN2_, LOW);
        }
        else {
            digitalWrite(IN1_, LOW);
            digitalWrite(IN2_, HIGH);
        }
        analogWrite(ENA_, abs(speed) * 255 / 100);
      }

      void right_wheel(double speed) {
        if (speed >= 0) {
            digitalWrite(IN3_, HIGH);
            digitalWrite(IN4_, LOW);
        }
        else {
            digitalWrite(IN3_, LOW);
            digitalWrite(IN4_, HIGH);
        }
        analogWrite(ENB_, abs(speed) * 255 / 100);
      }
      
      
  private:
    int IN1_, IN2_, ENA_, IN3_, IN4_, ENB_;

    void setup() {
      pinMode(IN1_, OUTPUT);
      pinMode(IN2_, OUTPUT);
      pinMode(ENA_, OUTPUT);
      pinMode(IN3_, OUTPUT);
      pinMode(IN4_, OUTPUT);
      pinMode(ENB_, OUTPUT);
    }
};

#endif