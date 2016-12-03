#include <24FJ48GA002.h>
#include "BL_Support.h"
#fuses FRC_PLL, OSCIO
#fuses NOIOL1WAY, NOWDT, NODEBUG, NOWRT, NOPROTECT, NOJTAG
//#device *=16 ADC = 10
#use delay(clock=16000000)
/* UART1 connection (see in schematic diagram) */
#PIN_SELECT U1RX = PIN_B12
#PIN_SELECT U1TX = PIN_B13
/* UART2 connection (see in schematic diagram) */
//#PIN_SELECT U2RX = PIN_B14
//#PIN_SELECT U2TX = PIN_B15
/*
* To map the standard io functions, e.g., printf(), kbhit() and others to
* the UART1 the UART1 must defined after UART2. The last defined UART will be
* mapped to the standard io functions.
*/
//#use rs232(baud=9600, UART2, stream=ESP)
#use fast_io(B)

#use rs232(baud=9600, UART1) // UART1 will be mapped to the standard io functions

int type ;
unsigned long int Orientation;
int state_keep ;
struct Motor{
    double pwm ;
    int state;
    int direction;
    int encoder;
    int target;
    int error ;
    int p_error ;
    double Kp ;
    double Ki ;
    double Kd ;
    double P ;
    double I ;
    double D ;
    double Sum_error ;
};
struct Motor X;
struct Motor Y;
struct Motor Z;

#PIN_SELECT OC1 = PIN_B4  //Motor1 --> x-axis
#PIN_SELECT OC2 = PIN_B6  //Motor2 --> y-axis
#PIN_SELECT OC3 = PIN_B11 //Motor3 --> z-axis
#PIN_SELECT OC4 = PIN_B14 //Servo1 --> Orientation
#PIN_SELECT OC5 = PIN_B15 //Servo2 --> Gripper

#PIN_SELECT INT1 = PIN_B8
#PIN_SELECT INT2 = PIN_B9

unsigned char data_recieve[11]={0};
int SM_BufIdx=0 ,state_serial =0;
char SM_Id = 1;
char SM_check ;
boolean flag1=0,flag2=0,flag3=0 ;
/*void SM_TxD(char c){
    putc('S');
    if(c == 'Y'){
        putc('Y');
    }
    else if(c == 'N'){
        putc('N');
    }
    else if(c == 'C'){
        putc('C');
    }
    else if(c == 'P'){
        int xh = X.encoder/256 , xl = X.encoder%256;
        int yh = Y.encoder/256 , yl = Y.encoder%256;
        int zh = Z.encoder/256 , zl = Z.encoder%256;
        unsigned char x_h = xh , x_l = xl , y_h = yh,y_l=yl,z_h=zh,z_l=zl;
        putc(x_h);putc(x_l);putc(y_h);putc(y_l);putc(z_h);putc(z_l);
    }
    else if(c=='H'){
        putc('R');
        putc('H');
    }
    putc('E');
}*/

void SM_RxD(char c){
    switch(SM_Id){
        case 1:
            if(c=='S'){
                SM_Id++;
                SM_BufIdx = 0;
            }
            break;
        case 2:
            if(c=='P'||'W'||'H'||'K'){
                SM_check = c;
                SM_Id++;
            }
            else{
                SM_Id = 1;
            }
            break;
        case 3:
            if(c=='E' && SM_BufIdx >= 10){
                if(SM_BufIdx == 10 && SM_check == 'P'){
                    //printf("\r\nGot a frame \r\n");
                    //SM_TxD('Y');
                    SM_BufIdx = 0;
                    state_serial = 1;
                    SM_Id =1;
                }
                else if(SM_check =='P' && SM_BufIdx != 10){
                    //SM_TxD('N');
                    SM_BufIdx = 0;
                    SM_Id =1;
                }
                else if(SM_check =='K'){
                    //SM_TxD('P');
                    SM_BufIdx = 0;
                    SM_Id =1;
                }
                else if(SM_check =='H'){
                    //SM_TxD('H');
                    SM_BufIdx = 0;
                    SM_Id =1;
                }
                else if(SM_check =='W'){
                    //SM_TxD('W');
                    X.encoder =0;
                    Y.encoder =0;
                    Z.encoder =0;
                    SM_BufIdx = 0;
                    SM_Id =1;
                }
                else{
                    SM_BufIdx = 0;
                    SM_Id =1;
                }
            }
            else{
                data_recieve[SM_BufIdx] = c;
                SM_BufIdx++;
            }
            break;
        case 4:
            SM_Id = 1;
            break;
    }
}

#INT_RDA
void UART1_ISR(){
    char c = getc();
    //putc(c);
    SM_RxD(c);
}

#INT_EXT0 //x-axis
void INT_EXT_INPUT0( void ) {
    if(input(PIN_A1)==0)
        X.encoder--;
    else
        X.encoder++;
   
//    printf("FUCK");
    
}
#INT_EXT1 //y-axis
void INT_EXT_INPUT1( void ) {
    if(input(PIN_A2)==0)
        Y.encoder--;
    else
        Y.encoder++;
}
#INT_EXT2 //z-axis
void INT_EXT_INPUT2( void ) {
    if(input(PIN_A4)==0)
        Z.encoder--;
    else
        Z.encoder++;
    //printf("%d\r\n",Z.encoder);
}

int PID_pwm(struct Motor A){
A.error     = A.target-A.encoder ;
A.Sum_error = (A.Sum_error + A.error) ;
A.I         = ((double)A.Sum_error)*A.Ki ;
A.D         = (double)(A.p_error - A.error)*A.Kd ;
A.pwm       = (((double)A.error*A.Kp)+A.I +A.D);
A.p_error   = A.error ;
if(A.pwm <0) {A.pwm = A.pwm*(-1) ;}
if(A.pwm >4000) {A.pwm = 4000 ;}
    return (int)A.pwm ;
}

#INT_TIMER2
void Timer2_ISR(){
    flag1 = 1;
    flag2 = 1;
    flag3 = 1;
}
void Init_Timer2(){
    setup_timer2(TMR_INTERNAL |TMR_DIV_BY_8,4000);
    enable_interrupts(INT_TIMER2);
}

void Init_Interrupts() {
    
enable_interrupts( INT_EXT0 );
ext_int_edge( 0, L_TO_H );
enable_interrupts( INT_EXT1 );
ext_int_edge( 1, L_TO_H );
enable_interrupts( INT_EXT2 );
ext_int_edge( 2, L_TO_H );
clear_interrupt(GLOBAL);
}

void MotorX(int direct){
    if(direct==0)
        output_high(PIN_B3);
    if(direct==1)
        output_low(PIN_B3);
}
void MotorY(int direct){
    if(direct==0)
        output_high(PIN_B5);
    if(direct==1)
        output_low(PIN_B5);
}
void MotorZ(int direct){
    if(direct==0)
        output_high(PIN_B10);
    if(direct==1) 
        output_low(PIN_B10);
}
void Servo_Keep(int state_keep,int type){
    if(state_keep==0)
    {
        if(type==0)//circle
            set_pwm_duty(5,1400);
        else if(type==1)//square
            set_pwm_duty(5,1000);
        else if(type==2)
            set_pwm_duty(5,1400);
        else if(type==3)
            set_pwm_duty(5,1400);
    }
    else if(state_keep==1)
    {
        if(type==1)
            set_pwm_duty(5,700);
        else
            set_pwm_duty(5,950);
    }
}
void Initial_Motor(struct Motor A){
    A.pwm = 0;
    A.state = 0;
    A.direction = 0;
    A.target = 0;
    A.error = 0 ;
    A.p_error = 0 ;
    A.Kp = 0 ;
    A.Ki = 0 ;
    A.Kd = 0 ;
    A.P = 0 ;
    A.I = 0 ;
    A.D = 0 ;
    A.Sum_error = 0;
}




void main(void){
    int first_step = 0;
    int first_command = 0;
    int state_all = 0;
    disable_interrupts(GLOBAL);
    clear_interrupt(INT_RDA);
    enable_interrupts(INT_RDA);
    Init_Timer2();
    Init_Interrupts();
    
    enable_interrupts(GLOBAL);
    
    set_tris_b(0x3387);
    set_tris_a(0xFF);
    setup_compare(1, COMPARE_PWM | COMPARE_TIMER2);
    setup_compare(2, COMPARE_PWM | COMPARE_TIMER2);
    setup_compare(3, COMPARE_PWM | COMPARE_TIMER2);
    setup_compare(4, COMPARE_PWM | COMPARE_TIMER2);
    setup_compare(5, COMPARE_PWM | COMPARE_TIMER2);
    set_pwm_duty(1,0);
    set_pwm_duty(2,0);
    set_pwm_duty(3,0);
    Initial_Motor(X);
    X.encoder = 0;
    Initial_Motor(Y);
    Y.encoder = 0;
    Initial_Motor(Z);
    Z.encoder = 0;
    X.Kp = Z.Kp = 60;
    Y.Kp = 110;
    X.Ki = 0;
    Y.Ki = 0;
    Z.Ki = 0;
    X.Kd = Y.Kd = Z.Kd = 0;
    int count_print=0;
    delay_ms(1000);
    printf("Z");
    
    set_pwm_duty(4,1400);
    set_pwm_duty(5,500);
    delay_ms(2000);
    //set_pwm_duty(4,1900);
    set_pwm_duty(5,1000);
    delay_ms(1000);
    while(1){
        if(state_serial ==1)
        {
            first_command++ ;
            state_serial = 0;
            X.target = ((data_recieve[2]*256)+ data_recieve[3])*19;
            Y.target = ((data_recieve[0]*256)+ data_recieve[1])*19;
            Z.target = ((data_recieve[4]*256)+ data_recieve[5])*19;
            Orientation = (data_recieve[6]*256)+ data_recieve[7];
            //printf("%d\r\n",Orientation);
            Orientation = 360 - Orientation ;
            Orientation = Orientation*(2168-400)/360 + 400 ;
            //printf("%d\r\n",Orientation);
            //printf("%d %d %d %d %d\r\n",X.target,Y.target,Z.target,Orientation,((Orientation*1174)/360)+512);
            //set_pwm_duty(4,Orientation);
            set_pwm_duty(4,2168);
            type = data_recieve[8];
            state_keep = data_recieve[9];
            //if(state_keep == 0)
            //    set_pwm_duty(5,850);
            state_all = 1;
            X.state = 1;
            Y.state = 1;
            Z.state = 1;
        }
        if( state_all ==1){
            int temp_x=1 , temp_y=1 ,temp_z=1;
            /*if(X.state==1 && state_keep ==1 && first_command >= 2)
            {
                    if(X.target > X.encoder)
                            X.direction = 1;
                    else if(X.target < X.encoder)
                            X.direction = 0;
                    MotorX(X.direction);
                    if(flag1 ==1){
                        set_pwm_duty(1,PID_pwm(X));
                        temp_x = PID_pwm(X);
                        flag1 = 0;
                    }
            }
            if(X.state==1 && state_keep == 0 && first_command >= 2)
            {
                if(temp_y == 0){
                    if(X.target > X.encoder)
                            X.direction = 1;
                    else if(X.target < X.encoder)
                            X.direction = 0;
                    MotorX(X.direction);
                    if(flag1 ==1){
                        set_pwm_duty(1,PID_pwm(X));
                        temp_x = PID_pwm(X);
                        flag1 = 0;
                    }
                }
                
            }
            if(Y.state==1 && state_keep == 0 && first_command >= 2)
            {
                if(temp_x == 0){
                    if(Y.target > Y.encoder)
                            Y.direction = 1;
                    else if(Y.target < Y.encoder)
                            Y.direction = 0;
                    MotorY(Y.direction);
                    if(flag2 ==1){
                        set_pwm_duty(2,PID_pwm(Y));
                        temp_y = PID_pwm(Y);
                        flag2 = 0;
                    }
                }
            }
            if(Y.state==1 && state_keep == 1 && first_command >= 2)
            {
                if(Y.target > Y.encoder)
                            Y.direction = 1;
                    else if(Y.target < Y.encoder)
                            Y.direction = 0;
                    MotorY(Y.direction);
                    if(flag2 ==1){
                        set_pwm_duty(2,PID_pwm(Y));
                        temp_y = PID_pwm(Y);
                        flag2 = 0;
                    }
            
            }*/
            if(X.state==1 /*&& first_command <= 1*/)
            {
                    if(X.target > X.encoder)
                            X.direction = 1;
                    else if(X.target < X.encoder)
                            X.direction = 0;
                    MotorX(X.direction);
                    if(flag1 ==1){
                        set_pwm_duty(1,PID_pwm(X));
                        temp_x = PID_pwm(X);
                        
                        flag1 = 0;
                    }
                
            }
            if(Y.state==1 /* && first_command <= 1*/)
            {
                    if(Y.target > Y.encoder)
                            Y.direction = 1;
                    else if(Y.target < Y.encoder)
                            Y.direction = 0;
                    MotorY(Y.direction);
                    if(flag2 ==1){
                        set_pwm_duty(2,PID_pwm(Y));
                        temp_y = PID_pwm(Y);
                       if(count_print<=1500){
                            printf("%d,%d\r\n",Y.target,Y.encoder);
                            count_print++;
                        }
                        flag2 = 0;
                    }
                
            }
            /*if(Z.state==1 )
            {
                    if(Z.target > Z.encoder)
                            Z.direction = 1;
                    else if(Z.target < Z.encoder)
                            Z.direction = 0;
                    MotorZ(Z.direction);
                    if(flag3 ==1){
                        set_pwm_duty(3,PID_pwm(Z));
                        temp_z = PID_pwm(Z);
                        if(count_print<=1500){
                            printf("%d,%d\r\n",Z.target,Z.encoder);
                            count_print++;
                        }
                        flag3 = 0;
                    }
                
            }*/
            if(Z.state==1 && temp_x ==0 && temp_y ==0)
            {
                
                set_pwm_duty(4,Orientation);
                if(state_keep == 0){
                    set_pwm_duty(5,700);
                }
                if(Z.target > Z.encoder)
                            Z.direction = 1;
                else if(Z.target < Z.encoder)
                            Z.direction = 0;
                MotorZ(Z.direction);
                if(flag3 ==1){
                    set_pwm_duty(3,PID_pwm(Z));
                    temp_z = PID_pwm(Z);
                    flag3 = 0;
                }
            }
            if(temp_z == 0)
            {
                Z.state = 0;
                Servo_Keep(state_keep,type);
                delay_ms(1000);
                while(input(PIN_B2)==1) //while Limit Switch isn't Press
                {
                    MotorZ(0);
                    set_pwm_duty(3,2000);
                }
                if(input(PIN_B2)==0)
                {
                    printf("Z");
                }
                
                if(state_keep == 1)
                {
                    set_pwm_duty(5,500);
                }
                set_pwm_duty(3,0);
                    
            }
        }
        if((input(PIN_A0)==0) || (SM_check=='H')||(first_step ==0))
        {
                    first_command = 0 ;
                    SM_check = 0 ;
                    first_step = 1;
                    state_all = 0;
                    int sim_state_x=0,sim_state_y=0,sim_state_z=0;
                    set_pwm_duty(4,2168);
                    set_pwm_duty(5,500);
                    while(sim_state_x==0||sim_state_y==0||sim_state_z==0){
                        if(sim_state_x==0)
                        {
                            if(input(PIN_B0)==0){
                                sim_state_x=1;
                                set_pwm_duty(1,0);
                            }
                            else{
                                MotorX(0);
                                set_pwm_duty(1,2000);
                            }    
                        }
                        if(sim_state_y==0)
                        {
                            if(input(PIN_B1)==0){
                                sim_state_y=1;
                                set_pwm_duty(2,0);
                            }
                            else{
                                MotorY(0);
                                set_pwm_duty(2,2000);
                            }    
                        }
                        if(sim_state_z==0)
                        {
                            if(input(PIN_B2)==0){
                                sim_state_z=1;
                                set_pwm_duty(3,0);
                            }
                            else{
                                MotorZ(0);
                                set_pwm_duty(3,2000);
                            }    
                        }
                    }
                    while(sim_state_x==1||sim_state_y==1||sim_state_z==1){
                        if(sim_state_x==1)
                        {
                            if(input(PIN_B0)==1){
                                sim_state_x=2;
                                delay_ms(200);
                                set_pwm_duty(1,0);
                            }
                            else{
                                MotorX(1);
                                set_pwm_duty(1,2000);
                            }    
                        }
                        if(sim_state_y==1)
                        {
                            if(input(PIN_B1)==1){
                                sim_state_y=2;
                                delay_ms(200);
                                set_pwm_duty(2,0);
                            }
                            else{
                                MotorY(1);
                                set_pwm_duty(2,2000);
                            }    
                        }
                        if(sim_state_z==1)
                        {
                            if(input(PIN_B2)==1){
                                sim_state_z=2;
                                delay_ms(200);
                                set_pwm_duty(3,0);
                            }
                            else{
                                MotorZ(1);
                                set_pwm_duty(3,2000);
                            }    
                        }
                    }
                    while(sim_state_x==2||sim_state_y==2||sim_state_z==2){
                        if(sim_state_x==2)
                        {
                            if(input(PIN_B0)==0){
                                sim_state_x=0;
                                set_pwm_duty(1,0);
                            }
                            else{
                                MotorX(0);
                                set_pwm_duty(1,500);
                            }    
                        }
                        if(sim_state_y==2)
                        {
                            if(input(PIN_B1)==0){
                                sim_state_y=0;
                                set_pwm_duty(2,0);
                            }
                            else{
                                MotorY(0);
                                set_pwm_duty(2,1000);
                            }    
                        }
                        if(sim_state_z==2)
                        {
                            if(input(PIN_B2)==0){
                                sim_state_z=0;
                                set_pwm_duty(3,0);
                            }
                            else{
                                MotorZ(0);
                                set_pwm_duty(3,1400);
                            }    
                        }
                    }
                    delay_ms(100);
                    X.encoder =0;
                    Y.encoder =0;
                    Z.encoder =0;
                    printf("Z");
                    
        }
        if(input(PIN_B0)==0)
        {
            X.encoder = 0;
        }
        if(input(PIN_B1)==0)
        {
            Y.encoder = 0;
        }
        if(input(PIN_B2)==0)
        {
            Z.encoder = 0;
        }
            
            
    }
}