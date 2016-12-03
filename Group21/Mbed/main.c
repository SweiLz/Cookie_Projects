/*#######################################*/
#include <24FJ48GA002.h>
#include <stdio.h>
#include <stdlib.h>
#include "BL_Support.h"
/*#######################################*/
#define MOTOR_HZ 5000 //200Hz
#define SERVO_HZ 20000 //50Hz
#define CONTROL_HZ 20000 //50Hz
#define LIMIT_RANGE_X_LOW 50
#define LIMIT_RANGE_X_HIGH 4000
#define LIMIT_RANGE_Y_LOW 50
#define LIMIT_RANGE_Y_HIGH 3500
#define LIMIT_RANGE_Z_LOW 300
#define LIMIT_RANGE_Z_HIGH 1300
#define MARGIN 8
#define PULLEY_RADIUS 12.65
#define MAP_X 492
#define MAP_Y 5266
#define HOME_FEED 2050
#define PID_MIN_FEED 150
#define Kp 4
#define GRIPPER_OFFSET_X -370
#define GRIPPER_OFFSET_Y -143
//Cur.X = X.Cur;
//Cur.Y = (A.Cur + B.Cur)/2;
//Cur.Z = (A.Cur - B.Cur)/2;
/*#######################################*/
inline int16 MAX(int16 x, int16 y) {return (x > y) ? x : y;}
inline int16 MIN(int16 x, int16 y) {return (x < y) ? x : y;}
inline int16 mm2pulse(int16 mm) {return (int16) ((24.45 * mm) / PULLEY_RADIUS);}
typedef struct {BOOLEAN Dir : 1, State : 1;unsigned int16 Pwm, Cur, Tar, Count;int Turn;} Motor;
typedef struct {unsigned int16 Pwm;} Servo;
typedef struct {int16 X, Y, Z;} Point;
void init_Motor(Motor &M) {M.State = M.Dir = M.Pwm = M.Count = M.Cur = M.Tar = M.Turn = 0;}
void init_Servo(Servo &S) {S.Pwm = 500;}
void init_Point(Point &P) {P.X = P.Y = P.Z = 0;}
Motor X, A, B;
Servo G, R;
Point Target, Command;

BOOLEAN RUNNING = 0,GO_STATE = 0;

char SM_Buf[50];
unsigned int SM_Val[5];
char SM_BufIndex = 0;
char SM_ValIndex = 0;
char SM_ID = 0;

void PID(Motor &M) {
    M.Cur = M.Count;
    M.Turn = (M.Tar - M.Cur) * Kp;
    M.State = 0;
    if (M.Turn > MARGIN) {
        M.Dir = 0;
        M.Pwm = MIN(MAX(M.Turn, PID_MIN_FEED), MOTOR_HZ);
    }
    else if (M.Turn < -MARGIN) {
        M.Dir = 1;
        M.Pwm = MIN(MAX(-M.Turn, PID_MIN_FEED), MOTOR_HZ);
    }
    else {
        M.Pwm = 0;
        M.State = 1;
    }
}

#INT_EXT0
void INT_EN_A(){A.Count += input(Encode_A_B)*2 - 1;}
#INT_EXT1
void INT_EN_X(){X.Count += input(Encode_X_B)*2 - 1;}
#INT_EXT2
void INT_EN_B(){B.Count += input(Encode_B_B)*2 - 1;}


#INT_TIMER1

void Main_Control() {
    if (GO_STATE) {
        
        PID(X);
        PID(A);
        PID(B);
      
        if (X.State) {
            set_pwm_duty(MX, 0);
        } else {
            output_bit(Motor_X_Dir, X.Dir);
            set_pwm_duty(MX, X.Pwm);
        }
        if (A.State) {
            set_pwm_duty(MA, 0);
        } else {
            output_bit(Motor_A_Dir, A.Dir);
            set_pwm_duty(MA, A.Pwm);
        }
        if (B.State) {
            set_pwm_duty(MB, 0);
        } else {
            output_bit(Motor_B_Dir, B.Dir);
            set_pwm_duty(MB, B.Pwm);
        }
        if (X.State && A.State && B.State) {
            GO_STATE = 0;
        }
                set_pwm_duty(SG,G.Pwm);
                set_pwm_duty(SR,R.Pwm);
    }
}

void Homing() {
    set_pwm_duty(MX, 0);
    set_pwm_duty(MA, 0);
    set_pwm_duty(MB, 0);
    set_pwm_duty(SG, 1220);
    set_pwm_duty(SR, 550);
    
    output_bit(Motor_X_Dir, 1);
    set_pwm_duty(MX, HOME_FEED);
    while (input(Limit_X)) {delay_ms(1);}
    output_bit(Motor_X_Dir, 0);
    set_pwm_duty(MX, 160);
    while (!input(Limit_X)) {delay_ms(1);}
    set_pwm_duty(MX, 0);

    char LY = 1, LZ = 1;
    set_pwm_duty(MA, HOME_FEED);
    set_pwm_duty(MB, HOME_FEED);
    while (LY || LZ) {
        LY = input(Limit_Y);
        LZ = input(Limit_Z);
        output_bit(Motor_B_Dir, !LZ);
        output_bit(Motor_A_Dir, LY);
        delay_ms(1);
    }
    set_pwm_duty(MA, 0);
    set_pwm_duty(MB, 0);
    output_bit(Motor_A_Dir, 0);
    set_pwm_duty(MA, 160);
    while (!input(Limit_Y) || !input(Limit_Z)) {delay_ms(1);
    }

    set_pwm_duty(MA, 0);
    set_pwm_duty(MB, 0);
    set_pwm_duty(MX, 0);
    
    
    init_Point(Target);
    init_Point(Command);

    X.Count = 0;
    A.Count = 0;
    B.Count = 0;
    Target.X = mm2pulse(MIN(MAX((0), LIMIT_RANGE_X_LOW), LIMIT_RANGE_X_HIGH));
    Target.Y = mm2pulse(MIN(MAX((0), LIMIT_RANGE_Y_LOW), LIMIT_RANGE_Y_HIGH));
    Target.Z = mm2pulse(LIMIT_RANGE_Z_LOW);
    G.Pwm = 1220;
    R.Pwm = 550;
    RUNNING = 1;
}

#INT_RDA       

void UART1_Isr() {
    char ch = getc();
    switch (SM_ID) {
        case 0:
            if (ch == '<') {
                SM_BufIndex = 0;
                SM_ValIndex = 0;
                SM_ID = 1;
            }
            if (ch == '@') {
                Homing();
            }
            break;
        case 1:
            if (ch == ',') {
                SM_Buf[SM_BufIndex] = '\0';
                SM_BufIndex = 0;
                SM_ID = 2;
                SM_Val[SM_ValIndex++] = atoi(SM_Buf);
            } else {
                SM_Buf[SM_BufIndex++] = ch;
            }
            break;
        case 2:
            if (ch == ',') {
                SM_Buf[SM_BufIndex] = '\0';
                SM_BufIndex = 0;
                SM_ID = 3;
                SM_Val[SM_ValIndex++] = atoi(SM_Buf);
            } else {
                SM_Buf[SM_BufIndex++] = ch;
            }
            break;
        case 3:
            if (ch == ',') {
                SM_Buf[SM_BufIndex] = '\0';
                SM_BufIndex = 0;
                SM_ID = 4;
                SM_Val[SM_ValIndex++] = atoi(SM_Buf);
            } else {
                SM_Buf[SM_BufIndex++] = ch;
            }
            break;
        case 4:
            if (ch == ',') {
                SM_Buf[SM_BufIndex] = '\0';
                SM_BufIndex = 0;
                SM_ID = 5;
                SM_Val[SM_ValIndex++] = atoi(SM_Buf);
            } else {
                SM_Buf[SM_BufIndex++] = ch;
            }
            break;
        case 5:
            if (ch == '>') {
                SM_Buf[SM_BufIndex] = '\0';
                SM_BufIndex = 0;
                SM_ID = 0;
                SM_Val[SM_ValIndex] = atoi(SM_Buf);
                Target.X = mm2pulse(MIN(MAX((MAP_X + SM_Val[0] + GRIPPER_OFFSET_X), LIMIT_RANGE_X_LOW), LIMIT_RANGE_X_HIGH));
                Target.Y = mm2pulse(MIN(MAX((MAP_Y - SM_Val[1] + GRIPPER_OFFSET_Y), LIMIT_RANGE_Y_LOW), LIMIT_RANGE_Y_HIGH));
                Target.Z = mm2pulse(MIN(MAX((LIMIT_RANGE_Z_LOW + SM_Val[2]), LIMIT_RANGE_Z_LOW), LIMIT_RANGE_Z_HIGH));
                G.Pwm = MIN(MAX(500 + (int) (0.72 * SM_Val[3]), 500), 1220);
                R.Pwm = MIN(MAX(550 + (int) (0.95 * SM_Val[4]), 550), 2270);

                RUNNING = 1;

            } else {
                SM_Buf[SM_BufIndex++] = ch;
            }
            break;
    }
}

void main(void) {
    delay_ms(1000);
    init_Motor(X);
    init_Motor(A);
    init_Motor(B);
    init_Servo(G);
    init_Servo(R);

    disable_interrupts(GLOBAL);
    //Initial Motor and Servo
    setup_timer2(TMR_INTERNAL | TMR_DIV_BY_8, MOTOR_HZ);
    setup_compare(MX, COMPARE_PWM | COMPARE_TIMER2);
    setup_compare(MA, COMPARE_PWM | COMPARE_TIMER2);
    setup_compare(MB, COMPARE_PWM | COMPARE_TIMER2);
    setup_timer3(TMR_INTERNAL | TMR_DIV_BY_8, SERVO_HZ);
    setup_compare(SG, COMPARE_PWM | COMPARE_TIMER3);
    setup_compare(SR, COMPARE_PWM | COMPARE_TIMER3);
    //Initial Encoder
    enable_interrupts(INT_EXT0); // Encode_A
    ext_int_edge(0, L_TO_H);
    enable_interrupts(INT_EXT1); // Encode_X
    ext_int_edge(1, L_TO_H);
    enable_interrupts(INT_EXT2); // Encode_B
    ext_int_edge(2, L_TO_H);
    //Initial Control Loop
    setup_timer1(TMR_INTERNAL | TMR_DIV_BY_8, CONTROL_HZ);
    enable_interrupts(INT_TIMER1);
    //Initial Serial Interrupt
    clear_interrupt(INT_RDA);
    enable_interrupts(INT_RDA);
    enable_interrupts(GLOBAL);

    Homing();
    
    while (TRUE) {
        if (RUNNING == 1) {
            X.Tar = Target.X;
            A.Tar = (Target.Z) + (Target.Y);
            B.Tar = (Target.Y) - (Target.Z);
            GO_STATE = 1;
            while (GO_STATE);
            putc('f');
            RUNNING = 0;
        }
    }
}

