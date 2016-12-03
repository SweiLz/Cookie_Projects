/*****************************************/
#define    TIMER2_DIVIDE       10000
#define    TIMER2_multi        100

#define    TIMER3_DIVIDE       20000
#define    TIMER3_multi        25

#define    BACKWARD            0
#define    FORWARD             1

#define    PHASE_X             PIN_B4 //          DXI0
#define    PHASE_Y             PIN_A2 //          DXO0
#define    PHASE_Z             PIN_A4 //          DXO1
#define    MOTOR_X_DIRECTION_  PIN_B3 //          DXO3
#define    MOTOR_Y_DIRECTION_  PIN_A0 //          AXI0
#define    MOTOR_Z_DIRECTION_  PIN_A1 //          AXI1
#define    ENCODER_X           PIN_B5 //          DXI1
#define    ENCODER_Y           PIN_B6 //          DXI2
#define    ENCODER_Z           PIN_B7 //    INT0  DXI3
#define    MOTOR_X_PWM         PIN_B0 //          AXI2
#define    MOTOR_Y_PWM         PIN_B1 //          AXI3
#define    MOTOR_Z_PWM         PIN_B2 //          DXO2

#define    SERVO_ROTATE        PIN_B8
#define    SERVO_CRANK         PIN_B9
#define   LIMIT_SWITCH_X       PIN_B10
#define   LIMIT_SWITCH_Y       PIN_B11
#define   LIMIT_SWITCH_Z       PIN_B14
/*****************************************

/*****************************************/
#include "System.h"
/*#include <24FJ48GA002.H>
#fuses FRC_PLL 
#use delay(clock= 32000000)
#PIN_SELECT U1RX = PIN_B12
#PIN_SELECT U1TX = PIN_B13
#use rs232(baud= 9600, UART1)*/

#include "BL_Support.h"

#INCLUDE <stdlib.h>
#use fixed_io(b_outputs= MOTOR_X_DIRECTION_,MOTOR_X_PWM,MOTOR_Y_PWM,MOTOR_Y_PWM,SERVO_ROTATE,SERVO_CRANK )
#use fixed_io(a_outputs= MOTOR_Y_DIRECTION_,MOTOR_Z_DIRECTION_)
//#use fast_io(A)
//#use fast_io(B)

/**MOTOR**/
#PIN_SELECT OC1 = MOTOR_X_PWM 
#PIN_SELECT OC2 = MOTOR_Y_PWM 
#PIN_SELECT OC3 = MOTOR_Z_PWM 
#PIN_SELECT OC4 = SERVO_ROTATE 
#PIN_SELECT OC5 = SERVO_CRANK 

/**INTERRUPT ENCODER**/
// INT0                     // Encoder Z
#PIN_SELECT INT1 =  ENCODER_X  
#PIN_SELECT INT2 =  ENCODER_Y  

typedef struct {
    int pwm;
    long target_position;
    int swing_limit;
    int swing_count;
    BOOLEAN limit_switch;
    BOOLEAN limit_switch_old;
    BOOLEAN direction;
    BOOLEAN direction_old;
    BOOLEAN en_direction;
    BOOLEAN en_direction_old;
    long encoder_count;
    long dState; // Last position input
    long iState; // Integrator state
    int iMax, iMin;
    // Maximum and minimum allowable integrator state
    double iGain, // integral gain
    pGain, // proportional gain
    dGain; // derivative gain
} SPid;
// PID
double KP = 4;
double KI = 0;
double KD = 0;
int KPZ = 2;
int INTEGRAL_MAX = 3;
int INTEGRAL_MIN = -3;
SPid motor_x, motor_y, motor_z;
int swing_limit_max = 2;
unsigned long timer1 = 0;
int servo_rotate_pwm = 0;
int servo_crank_pwm = 0;
int servo_rotate_pwm_old = 0;
int servo_crank_pwm_old = 0;
BOOLEAN confirmation_state = FALSE;
BOOLEAN gohome_state = FALSE;
BOOLEAN home_state_x = FALSE;
BOOLEAN home_state_y = FALSE;
BOOLEAN home_state_z = FALSE;
BOOLEAN z_home_state = FALSE;
BOOLEAN log_state = 0;
char SM_Buf[50];
char SM_BufIndex = 0;
char SM_ID = 0;
int Display_state;
int debug_mode = 1;
char servo_state;
#int_TIMER2

void TIMER2_isr() {
    timer1++;

    if (motor_x.direction == FORWARD)output_high(MOTOR_X_DIRECTION_);
    else output_low(MOTOR_X_DIRECTION_);
    if (motor_y.direction == FORWARD)output_high(MOTOR_Y_DIRECTION_);
    else output_low(MOTOR_Y_DIRECTION_);
    if (motor_z.direction == FORWARD)output_high(MOTOR_Z_DIRECTION_);
    else output_low(MOTOR_Z_DIRECTION_);

    set_pwm_duty(1, motor_x.pwm * TIMER2_multi);
    set_pwm_duty(2, motor_y.pwm * TIMER2_multi);
    set_pwm_duty(3, motor_z.pwm * TIMER2_multi);


    motor_x.limit_switch = input(LIMIT_SWITCH_X);
    motor_y.limit_switch = input(LIMIT_SWITCH_Y);
    motor_z.limit_switch = input(LIMIT_SWITCH_Z);

}

#int_TIMER3

void TIMER3_isr() {
    /* set_pwm_duty(4, (servo_rotate_pwm));
     set_pwm_duty(5, (servo_crank_pwm));*/

}

void SM_RxD(char c) {
    if (SM_ID == 0) {
        SM_Buf[SM_BufIndex] = c;
        if (SM_Buf[SM_BufIndex] == '#' || SM_BufIndex >= 49) {
            SM_BufIndex = 0;
            SM_ID = 1;
            // printf("change");
        } else SM_BufIndex++;
    }
    if (SM_ID == 1) {
        // printf("SM_Buf[0] = %s ",SM_Buf);
        switch (SM_Buf[0]) {
            case '@': //Go to
                confirmation_state = TRUE;
                log_state = TRUE;
                int commacount[2] = {49, 49};
                int commaplus = 0;
                for (int j = 0; j <= strlen(SM_Buf); j++) {
                    commacount[commaplus] = j;
                    if (SM_Buf[commacount[0]] == ',')commaplus = 1;
                    if (SM_Buf[commacount[1]] == ',')break;
                    ;
                }

                char bufferX[16], bufferY[16], bufferZ[16];
                strcpy(bufferX, &SM_Buf[1]);
                long A_core = atoi32(&bufferX);


                strcpy(bufferY, &SM_Buf[commacount[0] + 1]);
                long Y_core = atoi32(&bufferY);


                motor_x.target_position = A_core - Y_core; // core XY Equation
                motor_y.target_position = A_core + Y_core;

                motor_x.swing_count = 0;
                motor_x.swing_limit = (motor_x.target_position - motor_x.encoder_count) / 10;
                if (motor_x.swing_limit < 0)motor_x.swing_limit = -motor_x.swing_limit;
                if (motor_x.swing_limit > swing_limit_max)motor_x.swing_limit = swing_limit_max;
                motor_y.swing_count = 0;
                motor_y.swing_limit = (motor_y.target_position - motor_y.encoder_count) / 10;
                if (motor_y.swing_limit < 0)motor_y.swing_limit = -motor_y.swing_limit;
                if (motor_y.swing_limit > swing_limit_max)motor_y.swing_limit = swing_limit_max;

                strcpy(bufferZ, &SM_Buf[commacount[1] + 1]);
                motor_z.target_position = atoi32(&bufferZ);
                motor_z.swing_count = 0;
                motor_z.swing_limit = (motor_z.target_position - motor_z.encoder_count) / 10;
                if (motor_z.swing_limit < 0)motor_z.swing_limit = -motor_z.swing_limit;
                if (motor_z.swing_limit > swing_limit_max)motor_z.swing_limit = swing_limit_max;
                Display_state = 1;
                break;



            case 'C':
                char bufferC[3];
                strcpy(bufferC, &SM_Buf[1]);
                int numS = atoi(&bufferC);
                if (numS == 0) {
                    Display_state = 2;
                } else {
                    swing_limit_max = numS;
                    Display_state = 3;
                }
                break;

            case 'P':
                Display_state = 4;
                break;
            case 'D':
                switch (SM_Buf[1]) {
                    case '@': //Go to
                        confirmation_state = TRUE;
                        int commacount2[5] = {49, 49, 49, 49, 49};
                        int commaplus2 = 0;
                        for (int j2 = 0; j2 <= strlen(SM_Buf); j2++) {
                            commacount2[commaplus2] = j2;
                            if (SM_Buf[commacount2[commaplus2]] == ',')commaplus2++;
                            if (SM_Buf[commacount2[4]] == ',')break;

                        }
                        char bufferX2[8], bufferY2[8], bufferZ2[8], bufferCX[2], bufferCY[2], bufferCZ[2];
                        strcpy(bufferX2, &SM_Buf[2]);
                        motor_x.pwm = atoi(&bufferX2);
                        if (motor_x.pwm < 0)motor_x.pwm = 0;
                        if (motor_x.pwm > 100)motor_x.pwm = 100;
                        strcpy(bufferCX, &SM_Buf[commacount2[2] + 1]);
                        motor_x.direction = atoi(&bufferCX);

                        strcpy(bufferY2, &SM_Buf[commacount2[0] + 1]);
                        motor_y.pwm = atoi(&bufferY2);
                        if (motor_y.pwm < 0)motor_y.pwm = 0;
                        if (motor_y.pwm > 100)motor_y.pwm = 100;
                        strcpy(bufferCX, &SM_Buf[commacount2[2] + 1]);
                        motor_y.direction = atoi(&bufferCY);

                        strcpy(bufferZ2, &SM_Buf[commacount2[1] + 1]);
                        motor_z.pwm = atoi(&bufferZ2);
                        if (motor_z.pwm < 0)motor_z.pwm = 0;
                        if (motor_z.pwm > 100)motor_z.pwm = 100;
                        strcpy(bufferCX, &SM_Buf[commacount2[2] + 1]);
                        motor_z.direction = atoi(&bufferCZ);

                        Display_state = 19;
                        log_state = TRUE;
                        break;
                    default:
                        Display_state = 5;
                        break;
                }

                break;
            case 'K':
                switch (SM_Buf[1]) {
                    case 'P':
                    {
                        char bufferK[6];
                        strcpy(bufferK, &SM_Buf[2]);
                        KPZ = atoi(&bufferK);
                        Display_state = 6;
                      //  motor_x.pGain = KP;
                        //motor_y.pGain = KP;
                        motor_z.pGain = KPZ;
                        break;
                    }
                    case 'I':
                    {
                        char bufferK[6];
                        strcpy(bufferK, &SM_Buf[2]);
                        KI = atoi(&bufferK);
                        Display_state = 7;
                        motor_x.iGain = KI;
                        motor_y.iGain = KI;
                        motor_z.iGain = KI;

                        break;
                    }
                    case 'D':
                    {
                        char bufferK[6];
                        strcpy(bufferK, &SM_Buf[2]);
                        KD = atoi(&bufferK);
                        Display_state = 8;
                        motor_x.dGain = KD;
                        motor_y.dGain = KD;
                        motor_z.dGain = KD;

                        break;
                    }
                    case 'A':
                    {
                        char bufferK[6];
                        strcpy(bufferK, &SM_Buf[2]);
                        INTEGRAL_MAX = atoi(&bufferK);
                        Display_state = 9;
                        motor_x.iMax = INTEGRAL_MAX;
                        motor_y.iMax = INTEGRAL_MAX;
                        motor_z.iMax = INTEGRAL_MAX;
                        break;
                    }
                    case 'B':
                    {
                        char bufferK[6];
                        strcpy(bufferK, &SM_Buf[2]);
                        INTEGRAL_MIN = atoi(&bufferK);
                        Display_state = 10;
                        motor_x.iMin = INTEGRAL_MIN;
                        motor_y.iMin = INTEGRAL_MIN;
                        motor_z.iMin = INTEGRAL_MIN;
                        break;
                    }
                    default:
                        Display_state = 11;
                        break;
                }

                break;
            case 'Z':
                switch (SM_Buf[1]) {
                    case 'X':
                        motor_x.encoder_count = 0;
                        motor_x.pwm = 0;
                        motor_x.target_position = 0;
                        motor_x.swing_count = 0;
                        Display_state = 12;

                        break;
                    case 'Y':
                        motor_y.encoder_count = 0;
                        motor_y.pwm = 0;
                        motor_y.target_position = 0;
                        motor_y.swing_count = 0;
                        Display_state = 13;
                        break;
                    case 'Z':
                        motor_z.encoder_count = 0;
                        motor_z.pwm = 0;
                        motor_z.target_position = 0;
                        motor_z.swing_count = 0;
                        Display_state = 14;
                        break;
                    default:
                        motor_x.encoder_count = 0;
                        motor_x.pwm = 0;
                        motor_x.target_position = 0;
                        motor_x.swing_count = 0;
                        motor_y.encoder_count = 0;
                        motor_y.pwm = 0;
                        motor_y.target_position = 0;
                        motor_y.swing_count = 0;
                        motor_z.encoder_count = 0;
                        motor_z.pwm = 0;
                        motor_z.target_position = 0;
                        motor_z.swing_count = 0;
                        gohome_state = FALSE;
                        Display_state = 15;
                        log_state = FALSE;
                        break;

                }
                break;
            case 'S':
                if (SM_Buf[1] == 'C') {
                    char bufferCrank[5];
                    strcpy(bufferCrank, &SM_Buf[2]);
                    servo_crank_pwm = (int) ((float) atoi(&bufferCrank)*11.416 + 554);
                    servo_state = 'C';
                    Display_state = 16;
                } else if (SM_Buf[1] == 'R') {
                    char bufferRotate[5];
                    strcpy(bufferRotate, &SM_Buf[2]);
                    int numR = atoi(&bufferRotate) ;
                    servo_rotate_pwm = (int) ((float) numR * 10.611 + 520);
                    servo_state = 'R';
                    Display_state = 17;
                }

                break;
            case 'M':

                char debugmode[6];
                strcpy(debugmode, &SM_Buf[1]);
                debug_mode = atoi(&debugmode);
                Display_state = 18;
                break;
            case 'L':

                Display_state = 20;
                break;
            case 'H':
                gohome_state = TRUE;
                break;
            case 'J':
                z_home_state = TRUE;
                break;
            default:
                Display_state = 21;
                break;
        }
        SM_Buf = "                    "; //reset string buffer
        SM_ID = 0;
    }
}
#INT_RDA        //Serial interrupt

void UART1_Isr() {
    char c = getc();
    SM_RxD(c);

}
#INT_EXT0

void INT_EXT_Z(void) {
    motor_z.en_direction = !input(PHASE_Z);
    if (motor_z.en_direction)motor_z.encoder_count += 1;
    else motor_z.encoder_count -= 1;
    if (motor_z.en_direction != motor_z.en_direction_old) {
        if (confirmation_state && debug_mode != 1)motor_z.swing_count++;
        motor_z.en_direction_old = motor_z.en_direction;
    }
}

#INT_EXT1

void INT_EXT_X(void) {
    motor_x.en_direction = !input(PHASE_X);
    if (!input(PHASE_X))motor_x.encoder_count += 1;
    else motor_x.encoder_count -= 1;
    if (motor_x.en_direction != motor_x.en_direction_old) {
        if (confirmation_state && debug_mode != 1)motor_x.swing_count++;
        motor_x.en_direction_old = motor_x.en_direction;
    }
}

#INT_EXT2

void INT_EXT_Y(void) {
    motor_y.en_direction = !input(PHASE_Y);
    if (!input(PHASE_Y))motor_y.encoder_count += 1;
    else motor_y.encoder_count -= 1;
    if (motor_y.en_direction != motor_y.en_direction_old) {
        if (confirmation_state && debug_mode != 1)motor_y.swing_count++;
        motor_y.en_direction_old = motor_y.en_direction;
    }
}

void Init_INT() {
    enable_interrupts(INT_EXT0);
    ext_int_edge(0, L_TO_H);
    enable_interrupts(INT_EXT1);
    ext_int_edge(0, L_TO_H);
    enable_interrupts(INT_EXT2);
    ext_int_edge(0, L_TO_H);
}

void Display_Rx() {
    if (Display_state != 0) {
        switch (Display_state) {
            case 1:
                // printf(" Changed x from %ld to %ld ", motor_x.encoder_count, motor_x.target_position);
                //printf(" Changed y from %ld to %ld ", motor_y.encoder_count, motor_y.target_position);
                // printf(" Changed z from %ld to %ld ", motor_z.encoder_count, motor_z.target_position);
                Display_state = 0;
                break;
            case 2:
                printf("Swing count x,y,z is %d,%d,%d", motor_x.swing_count, motor_y.swing_count, motor_z.swing_count);
                printf("Swing Limit Max is %d ", swing_limit_max);
                Display_state = 0;
                break;
            case 3:
                printf("Change Swing Limit Max to %d ", swing_limit_max);
                Display_state = 0;
                break;
            case 4:
                Display_state = 0;
                printf("Position x,y,z is %ld,%ld,%ld ", motor_x.encoder_count, motor_y.encoder_count, motor_z.encoder_count);
                break;
            case 5:
                printf("PWM x,y,z is %d,%d,%d ", motor_x.pwm, motor_y.pwm, motor_z.pwm);
                Display_state = 0;
                break;
            case 6:
                printf("Change KPZ to %d ", KPZ);
                Display_state = 0;
                break;
            case 7:
                printf("Change KI to %d ", (int) KI);
                Display_state = 0;
                break;
            case 8:
                printf("Change KD to %d ", (int) KD);
                Display_state = 0;
                break;
            case 9:
                printf("Change INTEGRAL_MAX to %d ", INTEGRAL_MAX);
                Display_state = 0;
                break;
            case 10:
                printf("Change INTEGRAL_MIN to %d ", INTEGRAL_MIN);
                Display_state = 0;
                break;
            case 11:
                printf("PID parameter (Kp,Ki,Kd,A,B)");
                printf("is %d,%d,%d,%d,%d ", (int) KP, (int) KI, (int) KD, INTEGRAL_MAX, INTEGRAL_MIN);
                Display_state = 0;
                break;
            case 12:
                printf("Reset x Now is %ld,%ld,%ld ", motor_x.encoder_count, motor_y.encoder_count, motor_z.encoder_count);
                Display_state = 0;
                break;
            case 13:
                printf("Reset y Now is %ld,%ld,%ld ", motor_x.encoder_count, motor_y.encoder_count, motor_z.encoder_count);
                Display_state = 0;
                break;
            case 14:
                printf("Reset z Now is %ld,%ld,%ld ", motor_x.encoder_count, motor_y.encoder_count, motor_z.encoder_count);
                Display_state = 0;
                break;
            case 15:
                printf("Reset x,y,z is %ld,%ld,%ld ", motor_x.encoder_count, motor_y.encoder_count, motor_z.encoder_count);
                Display_state = 0;
                break;
            case 16:
                // printf("Changing Gripper pwm to %d ", servo_crank_pwm);
                printf(";");
                Display_state = 0;
                break;
            case 17:
                printf(";");
                //printf("Changing Rotate pwm to %d ", servo_rotate_pwm);
                Display_state = 0;
                break;
            case 18:
                printf("Debug mode = %d ", debug_mode);
                Display_state = 0;
                break;
            case 19:
                printf("Set pwm x,y,z = %d,%d,%d ", motor_x.pwm, motor_y.pwm, motor_z.pwm);
                Display_state = 0;
                break;
            case 20:
                printf("Limit Switch x,y,z = %d,%d,%d ", motor_x.limit_switch, motor_y.limit_switch, motor_z.limit_switch);
                Display_state = 0;
                break;
            case 21:
                printf("e");
                Display_state = 0;
                break;
            default:
                break;
        }
    }
}

void UpdatePID(SPid * pid) {
    if (debug_mode != 1) {
        long error = (pid->encoder_count - pid->target_position) / 3;
        double pTerm, dTerm, iTerm;
        pTerm = pid->pGain * error;
        // calculate the proportional term
        // calculate the integral state with appropriate limiting
        pid->iState += error;
        if (pid->iState > pid->iMax) pid->iState = pid->iMax;
        else if (pid->iState < pid->iMin) pid->iState = pid->iMin;
        iTerm = pid->iGain * pid->iState; // calculate the integral term
        dTerm = pid->dGain * (pid->encoder_count - pid->dState);
        pid->dState = pid->encoder_count;

        double PID_ans = 0 - (pTerm + iTerm + dTerm);
        if (PID_ans > 0) {
            pid->pwm = (int) PID_ans;
            if (pid->pwm > 100) pid->pwm = 100;
            pid->direction = FORWARD;
        } else if (PID_ans < 0) {
            pid->pwm = (int) -PID_ans;
            if (pid->pwm > 100) pid->pwm = 100;
            pid->direction = BACKWARD;
        }
    }
}

void confirmation_feedback2() {
    long m1 = (motor_x.target_position - motor_x.encoder_count);
    long m2 = (motor_y.target_position - motor_y.encoder_count);
    long m3 = (motor_z.target_position - motor_z.encoder_count);
    if (m1 < 0)m1 = 0 - m1;
    if (m2 < 0)m2 = 0 - m2;
    if (m3 < 0)m3 = 0 - m3;

    if (m1 <= 2 && m2<= 2 && m3<= 2 && confirmation_state && debug_mode != 1) {
        confirmation_state = FALSE;
        //printf("time(%ds)", timer1);
        //  timer1 = 0;
        /*  printf(" Current Position is %d,%d,%d ", motor_x.encoder_count, motor_y.encoder_count, motor_z.encoder_count);
          printf(" Swing_count is %d,%d,%d ", motor_x.swing_count, motor_y.swing_count, motor_z.swing_count);*/
        printf(";");
        motor_x.swing_count = 0;
        motor_y.swing_count = 0;
        motor_z.swing_count = 0;

    }
}

void servo_adjust() {
    if (servo_state == 'R') {
        int pp;
        set_pwm_duty(4, servo_rotate_pwm);
        if(servo_rotate_pwm>servo_rotate_pwm_old)pp=-5;
        else if(servo_rotate_pwm<servo_rotate_pwm_old)pp=+5;
        delay_ms(500);
        set_pwm_duty(4, servo_rotate_pwm+pp);
        servo_state = 'W'; //waiting
        servo_rotate_pwm_old = servo_rotate_pwm;
    } else if (servo_state == 'C') {
        int pp2;
        set_pwm_duty(5, (servo_crank_pwm));
        if(servo_crank_pwm>servo_crank_pwm_old)pp2=-5;
        else if(servo_crank_pwm<servo_crank_pwm_old)pp2=+5;
        delay_ms(500);
        set_pwm_duty(5, servo_crank_pwm+pp2);
        servo_state = 'W'; //waiting
        servo_crank_pwm_old = servo_crank_pwm;
    }
}

void main(void) {
    delay_ms(2000);

    motor_x.pGain = KP;
    motor_x.iGain = KI;
    motor_x.dGain = KD;
    motor_x.iMax = INTEGRAL_MAX;
    motor_x.iMin = INTEGRAL_MIN;

    motor_y.pGain = KP;
    motor_y.iGain = KI;
    motor_y.dGain = KD;
    motor_y.iMax = INTEGRAL_MAX;
    motor_y.iMin = INTEGRAL_MIN;

    motor_z.pGain = KPZ;
    motor_z.iGain = KI;
    motor_z.dGain = KD;
    motor_z.iMax = INTEGRAL_MAX;
    motor_z.iMin = INTEGRAL_MIN;
    printf("START");
    motor_x.encoder_count = 0;
    motor_x.pwm = 0;
    motor_x.target_position = 0;
    motor_x.swing_count = 0;
    motor_x.limit_switch = 1;
    motor_x.limit_switch_old = 1;
    motor_x.direction = FORWARD;
    motor_x.direction_old = FORWARD;

    motor_y.encoder_count = 0;
    motor_y.pwm = 0;
    motor_y.target_position = 0;
    motor_y.swing_count = 0;
    motor_y.limit_switch = 1;
    motor_y.limit_switch_old = 1;
    motor_y.direction = FORWARD;
    motor_y.direction_old = FORWARD;

    motor_z.encoder_count = 0;
    motor_z.pwm = 0;
    motor_z.target_position = 0;
    motor_z.swing_count = 0;
    motor_z.limit_switch = 1;
    motor_z.limit_switch_old = 1;
    motor_z.direction = FORWARD;
    motor_z.direction_old = FORWARD;

    SM_Buf = "                    ";
    SM_Buf[0] = '#';

    disable_interrupts(GLOBAL);
    clear_interrupt(INT_RDA); // recommend style coding to confirm everything clear before use
    enable_interrupts(INT_RDA);
    Init_INT();
    setup_timer2(TMR_INTERNAL | TMR_DIV_BY_8, TIMER2_DIVIDE);
    enable_interrupts(INT_TIMER2);
    setup_timer3(TMR_INTERNAL | TMR_DIV_BY_8, TIMER3_DIVIDE);
    //setup_timer3(TMR_INTERNAL | TMR_DIV_BY_8, TIMER3_DIVIDE);
    //enable_interrupts(INT_TIMER3);


    setup_compare(1, COMPARE_PWM | COMPARE_TIMER2);
    setup_compare(2, COMPARE_PWM | COMPARE_TIMER2);
    setup_compare(3, COMPARE_PWM | COMPARE_TIMER2);

    setup_compare(4, COMPARE_PWM | COMPARE_TIMER3);
    setup_compare(5, COMPARE_PWM | COMPARE_TIMER3);
    enable_interrupts(GLOBAL);
    while (TRUE) {
        //update();

        if (gohome_state == TRUE) {
            delay_ms(1);
            if (motor_x.limit_switch == 1 && home_state_x == FALSE) {
                motor_x.pwm = 90;
                motor_x.direction = BACKWARD;
                motor_y.pwm = 90;
                motor_y.direction = BACKWARD;

            } else if (motor_x.limit_switch == 0 && home_state_x == FALSE) {
                home_state_x = TRUE;
                motor_x.pwm = 90;
                motor_x.direction = FORWARD;
                motor_y.pwm = 90;
                motor_y.direction = BACKWARD;

            }

            if (motor_z.limit_switch == 1 && home_state_z == FALSE) {
                motor_z.pwm = 90;
                motor_z.direction = BACKWARD;

            } else if (motor_z.limit_switch == 0) {
                home_state_z == TRUE;
                motor_z.pwm = 0;
                motor_z.target_position = 0;
                motor_z.encoder_count = 0;
                motor_z.swing_count = 0;
                z_home_state = TRUE;

            }
            if (motor_y.limit_switch == 0 && home_state_y == FALSE) {
                home_state_y = TRUE;
                motor_x.pwm = 0;
                motor_x.target_position = 0;
                motor_x.encoder_count = 0;
                motor_x.swing_count = 0;

                motor_y.pwm = 0;
                motor_y.target_position = 0;
                motor_y.encoder_count = 0;
                motor_y.swing_count = 0;
            }
            if (motor_x.limit_switch == 0 && motor_y.limit_switch == 0 && motor_z.limit_switch == 0) {
                motor_x.pwm = 0;
                motor_x.target_position = 0;
                motor_x.encoder_count = 0;
                motor_x.swing_count = 0;

                motor_y.pwm = 0;
                motor_y.target_position = 0;
                motor_y.encoder_count = 0;
                motor_y.swing_count = 0;

                motor_z.pwm = 0;
                motor_z.target_position = 0;
                motor_z.encoder_count = 0;
                motor_z.swing_count = 0;

                home_state_x = FALSE;
                home_state_y = FALSE;
                home_state_z = FALSE;
                Display_state = 15;
                gohome_state = FALSE;
            }


        } else if (z_home_state == TRUE) {
            if (motor_z.limit_switch == 1) {
                motor_z.pwm = 90;
                motor_z.direction = BACKWARD;

            } else if (motor_z.limit_switch == 0 && z_home_state == TRUE) {
                z_home_state = FALSE;
                motor_z.pwm = 0;
                motor_z.target_position = 800;
                motor_z.encoder_count = 0;
                motor_z.swing_count = 0;
                while(abs(motor_z.target_position-motor_z.encoder_count)>5){
                    UpdatePID(&motor_z);
                }
                motor_z.pwm = 0;
                motor_z.target_position = 0;
                motor_z.encoder_count = 0;
                motor_z.swing_count = 0;
                printf(";",);

            }
        } else {
            UpdatePID(&motor_x);
            UpdatePID(&motor_y);
            UpdatePID(&motor_z);
            //Display_Rx();
            servo_adjust();
            if(log_state) {
                printf("\r\n%d",motor_y.encoder_count);
                delay_ms(50);
            }
            //confirmation_feedback2();
        }

    }
}


