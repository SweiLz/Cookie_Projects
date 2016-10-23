#include "define.h"
#include <24FJ48GA002.h>
#include <stdio.h>
#include <stdlib.h>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#fuses FRC_PLL
#use delay(clock = 32MHZ)
#use rs232(baud = 57600, xmit = PIN_B14, rcv = PIN_B15)
#use fixed_io(b_outputs = Motor_X_Dir, Motor_Y_Dir, Motor_Z_Dir, Motor_X_EN, Motor_Y_EN, Motor_Z_EN, Servo_G, Servo_R)

#PIN_SELECT OC1 = Servo_G
#PIN_SELECT OC2 = Servo_R
#PIN_SELECT OC3 = Motor_X_EN
#PIN_SELECT OC4 = Motor_Y_EN
#PIN_SELECT OC5 = Motor_Z_EN
// INT 0 Encode_Y_A
#PIN_SELECT INT1 = Encode_X_A
#PIN_SELECT INT2 = Encode_Z_A

#define MOTOR_HZ 5
#define SERVO_HZ 500
#define PID_HZ 10

unsigned int calHz(int Hz) { return (unsigned int)(250000 / Hz); }
unsigned int motorHz = calHz(MOTOR_HZ);
unsigned int servoHz = calHz(SERVO_HZ);
unsigned int pidHz = calHz(PID_HZ);

float Kp = 25.0;
float Ki = 0.0;
float Kd = 0.0;
int Current[3] = {0, 0, 0};
int Target[3] = {0, 0, 0};
float LastError[3] = {0, 0, 0};
float Integral[3] = {0.0, 0.0, 0.0};
float Derivative[3] = {0.0, 0.0, 0.0};
float Output[3] = {0.0, 0.0, 0.0};

void Init_Encoder() {
  enable_interrupts(INT_EXT0); // Encode_Y
  ext_int_edge(0, L_TO_H);     // Rising Edge
  enable_interrupts(INT_EXT1); // Encode_X
  ext_int_edge(1, L_TO_H);     // Rising Edge
  enable_interrupts(INT_EXT2); // Encode_Z
  ext_int_edge(2, L_TO_H);     // Rising Edge
}

void Init_PID() {
  setup_timer1(TMR_INTERNAL | TMR_DIV_BY_64, pidHz);
  enable_interrupts(INT_TIMER1);
}

void Init_Servo() {
  setup_timer2(TMR_INTERNAL | TMR_DIV_BY_64, servoHz);
  setup_compare(1, COMPARE_PWM | COMPARE_TIMER2);
  set_pwm_duty(1, 1);
  setup_compare(2, COMPARE_PWM | COMPARE_TIMER2);
  set_pwm_duty(2, 1);
}

void Init_MotorPWM() {
  setup_timer3(TMR_INTERNAL | TMR_DIV_BY_64, motorHz);
  setup_compare(3, COMPARE_PWM | COMPARE_TIMER3);
  set_pwm_duty(3, 1);
  setup_compare(4, COMPARE_PWM | COMPARE_TIMER3);
  set_pwm_duty(4, 1);
  setup_compare(5, COMPARE_PWM | COMPARE_TIMER3);
  set_pwm_duty(5, 1);
}

#INT_EXT1
void INT_EN_X() { input(Encode_X_B) ? Current[0]-- : Current[0]++; }
#INT_EXT0
void INT_EN_Y() { input(Encode_Y_B) ? Current[1]-- : Current[1]++; }
#INT_EXT2
void INT_EN_Z() { input(Encode_Z_B) ? Current[2]-- : Current[2]++; }

#INT_TIMER1
void PID_ISR() {
  for (int i = 0; i < 3; i++) {
    float Error = (float)(Target[i] - Current[i]);
    Integral[i] = Integral[i] + (Error/pidHz);
    Derivative[i] = (Error - LastError[i])*pidHz;
    Output[i] = Kp * Error + Ki * Integral[i] + Kd * Derivative[i];
    LastError[i] = Error;
    output_bit(5712 + i, Output[i] > 0);
  }
  set_pwm_duty(3, (int)MIN(MAX(abs(Output[0]), 1), motorHz - 1));
  set_pwm_duty(4, (int)MIN(MAX(abs(Output[1]), 1), motorHz - 1));
  set_pwm_duty(5, (int)MIN(MAX(abs(Output[2]), 1), motorHz - 1));
}

void setTarget(unsigned int Value[5]) {
  Target[0] = Value[0];
  Target[1] = Value[1];
  Target[2] = Value[2];
  set_pwm_duty(1, 250 + Value[3]);
  set_pwm_duty(2, 250 + Value[4]);
}

void Initial_Position() {
  while (1) {
    output_bit(Motor_X_Dir, !input(Limit_X));
    output_bit(Motor_Y_Dir, !input(Limit_Y));
    output_bit(Motor_Z_Dir, !input(Limit_Z));
    set_pwm_duty(3, input(Limit_X) ? (int)(motorHz / 10) : 10);
    set_pwm_duty(4, input(Limit_Y) ? (int)(motorHz / 10) : 10);
    set_pwm_duty(5, input(Limit_Z) ? (int)(motorHz / 10) : 10);
    if (!input(Limit_X) && !input(Limit_Y) && !input(Limit_Z))
      break;
  }
}

int main() {
  disable_interrupts(GLOBAL);
  set_tris_b(0x1F8);
  Init_MotorPWM();
  Initial_Position();
  Init_Servo();
  Init_Encoder();
  Init_PID();
  enable_interrupts(GLOBAL);

  char buffer[5][256];
  unsigned int value[5];
  char count[5];
  char index = 0;

  while (TRUE) {
    // @123,456,789,22,33,#
    // @12,34,56,100,200,#
    if (kbhit()) {
      char ch = getc();
      if (ch == '@') {
        index = 0;
        count[index] = 0;
      } else if (ch == ',') {
        buffer[index][count[index]++] = '\0';
        index++;
        count[index] = 0;
      } else if (ch == '#') {
        for (int i = 0; i < 5; i++) {
          value[i] = atoi(buffer[i]);
          printf("%d, ", value[i]);
        }
        printf("\r\n");
        setTarget(value);
      } else {
        buffer[index][count[index]++] = ch;
      }
    }
  }
  return 0;
}
