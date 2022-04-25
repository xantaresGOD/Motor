
#include "f28x_project.h"
#include "driverlib.h"
#include "device.h"
#include "math.h"
#include "test.h"

#define sqrt3     1.732050808
#define pi        3.141592654
#define FIFO_LEVEL        3
#define Time_Coefficient  100.0e6// change to 100M
#define Half_Tpwm         50.0e-6
#define INV_PWM_HALF_TBPRD  2500
float    Uamp =  0.05;
float i_qff=0.0;
float Uff;
extern float Load;
extern float FFBdf2;

int sgn(float x);

static const unsigned char BitReverseTable256[256] =
{
# define R2(n)     n,     n + 2*64,     n + 1*64,     n + 3*64
# define R4(n) R2(n), R2(n + 2*16), R2(n + 1*16), R2(n + 3*16)
# define R6(n) R4(n), R4(n + 2*4 ), R4(n + 1*4 ), R4(n + 3*4 )
    R6(0), R6(2), R6(1), R6(3)
};

typedef struct  {
    float32_t  Ualpha;          //s Input: reference alpha-axis phase voltage
    float32_t  Ubeta;           // Input: reference beta-axis phase voltage
    float32_t  Ta;              // Output: reference phase-a switching function
    float32_t  Tb;              // Output: reference phase-b switching function
    float32_t  Tc;              // Output: reference phase-c switching function
    float32_t  tmp1;            // Variable: temp variable
    float32_t  tmp2;            // Variable: temp variable
    float32_t  tmp3;            // Variable: temp variable
    uint16_t VecSector;         // Space vector sector
    } SVGEN;


/*-----------------------------------------------------------------------------
Default initalizer for the SVGEN object.
-----------------------------------------------------------------------------*/
#define SVGEN_DEFAULTS { 0,0,0,0,0 }

extern int cyclenum ;
extern uint32_t Position;

uint32_t position = 0;
uint32_t v =0;
uint32_t joint_position;


float Us_alpha = 0.0;
float Us_beta = 0.0;
extern float Is_a ;
extern float Is_b ;
extern float Is_d ;
extern float Is_q ;
float Is_alpha = 0.0;
float Is_beta = 0.0;
extern float Us_d;
extern float Us_q;
extern uint16_t RxData[FIFO_LEVEL];
extern uint16_t RxData2[FIFO_LEVEL];
extern float speedf;

float Ide;
float I_dot_de;

extern float AD_Out[2] = {0.0, 0.0};
extern int count;       //Calculate the count of AD sample,max value is 2

extern float Theta_S ;
extern float pos32val = 0.0;
extern float temp;
extern float jFB;

int Time_C_a = 0;
int Time_C_b = 0;
int Time_C_c = 0;
int Us_quadrant;
extern float Speed;
extern float kalmanspeed;
extern float speedwe;
float jointspeed1k;
float speed1k;
float speed10k;


float p1 = 0;
float p2 = 0;
float p3 = 0.0008;
float p1_d = 0 ;
float p2_d = 0 ;
float p3_d = 0 ;
float L = 0.0;
float R = 0.0;
float Ke = 0.000;
//float L_d =0;
int cogctl = 0;
float Theta;
float DBTheta;
float Us_a;
float Us_b;
int last_timea;

int db = 1;
void SVPWM()
{

    //Theta = Theta_S + (pi/2);
    Theta = atan2f(Us_beta,Us_alpha);



   if (Theta < 0)  Theta = Theta + (2.0*pi);

    if ((Theta >= (11*pi/6)) ||  (Theta < (pi/6)))      DBTheta = 0.0;
    if ((Theta >= (pi/6))    &&  (Theta < (pi/2)))      DBTheta = pi/3;
    if ((Theta >= (pi/2))    &&  (Theta < (5*pi/6)))    DBTheta = 2*pi/3;
    if ((Theta >= (5*pi/6))  &&  (Theta < (7*pi/6)))    DBTheta = 3*pi/3;
    if ((Theta >= (7*pi/6))  &&  (Theta < (9*pi/6)))    DBTheta = 4*pi/3;
    if ((Theta >= (9*pi/6))  &&  (Theta < (11*pi/6)))   DBTheta = 5*pi/3;

    if(db == 1)
    {
        Us_alpha = Us_alpha + (Uamp * cos(DBTheta));
        Us_beta  = Us_beta  + (Uamp * sin(DBTheta));
    }

    float Va,Vb,Vc;
    int A,B,C;
    float X,Y,Z;
    float Time_1,Time_2,Time_0;
    float Time_a,Time_b,Time_c;
    float Sum_Time1_Time2;

    float Udc = 48;

//-------------------debug------------------------------//

  //-------------------debug------------------------------//
    X = (sqrt3*Half_Tpwm*Us_beta)/Udc;
    Y = (sqrt3*Half_Tpwm*Us_beta)/(2*Udc)+(1.5*Half_Tpwm*Us_alpha)/Udc;
    Z = (sqrt3*Half_Tpwm*Us_beta)/(2*Udc)-(1.5*Half_Tpwm*Us_alpha)/Udc;

     Va = Us_beta;
     Vb = 0.5*(-sqrt3*Us_alpha-Us_beta);
     Vc = 0.5*(sqrt3*Us_alpha-Us_beta);


    if(Va > 0)
        A = 1;
    else
        A = 0;

    if(Vb > 0)
        B = 1;
    else
        B = 0;

    if(Vc > 0)
        C = 1;
    else
        C = 0;

    Us_quadrant = 4*A+2*B+C;

    if(Us_quadrant == 0)
        Us_quadrant = 1;

    switch(Us_quadrant)
    {
        case 1:
            Time_1 = Y;
            Time_2 = -X;


            if(Time_1+Time_2 > Half_Tpwm)
            {

                Sum_Time1_Time2 = Time_1+Time_2;
                Time_1 = Time_1*Half_Tpwm/Sum_Time1_Time2;
                Time_2 = Time_2*Half_Tpwm/Sum_Time1_Time2;
            }

            Time_0 = (Half_Tpwm-Time_1-Time_2)/2.0;
//          Time_7 = (Half_Tpwm-Time_1-Time_2)/2.0;

            Time_a = Time_0;
            Time_c = Time_0+Time_1;
            Time_b = Time_0+Time_1+Time_2;

            break;

        case 2:
            Time_1 = -X;
            Time_2 = Z;

            if(Time_1+Time_2 > Half_Tpwm)
            {
                Sum_Time1_Time2 = Time_1+Time_2;
                Time_1 = Time_1*Half_Tpwm/Sum_Time1_Time2;
                Time_2 = Time_2*Half_Tpwm/Sum_Time1_Time2;
            }

            Time_0 = (Half_Tpwm-Time_1-Time_2)/2.0;
//          Time_7 = (Half_Tpwm-Time_1-Time_2)/2.0;

            Time_c = Time_0;
            Time_b = Time_0+Time_1;
            Time_a = Time_0+Time_1+Time_2;

            break;

        case 3:
            Time_1 = -Y;
            Time_2 = -Z;

            if(Time_1+Time_2 > Half_Tpwm)
            {
                Sum_Time1_Time2 = Time_1+Time_2;
                Time_1 = Time_1*Half_Tpwm/Sum_Time1_Time2;
                Time_2 = Time_2*Half_Tpwm/Sum_Time1_Time2;
            }

            Time_0 = (Half_Tpwm-Time_1-Time_2)/2.0;
//          Time_7 = (Half_Tpwm-Time_1-Time_2)/2.0;

            Time_c = Time_0;
            Time_a = Time_0+Time_1;
            Time_b = Time_0+Time_1+Time_2;

            break;

        case 4:
            Time_1 = Z;
            Time_2 = Y;

            if(Time_1+Time_2 > Half_Tpwm)
            {
                Sum_Time1_Time2 = Time_1+Time_2;
                Time_1 = Time_1*Half_Tpwm/Sum_Time1_Time2;
                Time_2 = Time_2*Half_Tpwm/Sum_Time1_Time2;
            }

            Time_0 = (Half_Tpwm-Time_1-Time_2)/2.0;
//          Time_7 = (Half_Tpwm-Time_1-Time_2)/2.0;

            Time_b = Time_0;
            Time_a = Time_0+Time_1;
            Time_c = Time_0+Time_1+Time_2;

            break;

        case 5:
            Time_1 = -Z;
            Time_2 = X;

            if(Time_1+Time_2 > Half_Tpwm)
            {
                Sum_Time1_Time2 = Time_1+Time_2;
                Time_1 = Time_1*Half_Tpwm/Sum_Time1_Time2;
                Time_2 = Time_2*Half_Tpwm/Sum_Time1_Time2;
            }

            Time_0 = (Half_Tpwm-Time_1-Time_2)/2.0;
//          Time_7 = (Half_Tpwm-Time_1-Time_2)/2.0;

            Time_a = Time_0;
            Time_b = Time_0+Time_1;
            Time_c = Time_0+Time_1+Time_2;

            break;

        case 6:
            Time_1 = X;
            Time_2 = -Y;

            if(Time_1+Time_2 > Half_Tpwm)
            {
                Sum_Time1_Time2 = Time_1+Time_2;
                Time_1 = Time_1*Half_Tpwm/Sum_Time1_Time2;
                Time_2 = Time_2*Half_Tpwm/Sum_Time1_Time2;
            }

            Time_0 = (Half_Tpwm-Time_1-Time_2)/2.0;
//          Time_7 = (Half_Tpwm-Time_1-Time_2)/2.0;

            Time_b = Time_0;
            Time_c = Time_0+Time_1;
            Time_a = Time_0+Time_1+Time_2;

            break;
    }
//    if(Us_q != 0)
//    {
    Time_C_a = (int)(Time_a*Time_Coefficient) ;
    Time_C_b = (int)(Time_b*Time_Coefficient) ;
    Time_C_c = (int)(Time_c*Time_Coefficient) ;
//    }
//    else
//    {
//        Time_C_a = (int)(Time_a*Time_Coefficient);
//        Time_C_b = (int)(Time_b*Time_Coefficient);
//        Time_C_c = (int)(Time_c*Time_Coefficient);
//    }

//  Time_C_a = 4000;
//  Time_C_b = 4000;
//  Time_C_c = 4000;

    if(Time_C_a > 4999 ) Time_C_a = 4999;
    else if(Time_C_a < 1 ) Time_C_a = 1;

    if(Time_C_b > 4999 ) Time_C_b = 4999;
    else if(Time_C_b < 1 ) Time_C_b = 1;

    if(Time_C_c > 4999 ) Time_C_c = 4999;
    else if(Time_C_c < 1 ) Time_C_c = 1;
}

void ePWMGen()
{
//
        EPwm1Regs.CMPA.bit.CMPA=Time_C_a;
        EPwm2Regs.CMPA.bit.CMPA=Time_C_b;
        EPwm3Regs.CMPA.bit.CMPA=Time_C_c;
        EPwm1Regs.CMPB.bit.CMPB=Time_C_a;
        EPwm2Regs.CMPB.bit.CMPB=Time_C_b;
        EPwm3Regs.CMPB.bit.CMPB=Time_C_c;

    // ----------------------------------------------------------------------------

}

void CLARK(void)
{
    Is_alpha = 1.5 * Is_a;
    Is_beta = sqrt3/2.0 * Is_a+sqrt3 * Is_b;
}


void PARK()
{
    Is_d = cos(Theta_S)*Is_alpha + sin(Theta_S)*Is_beta;


    Is_q = -sin(Theta_S)*Is_alpha + cos(Theta_S)*Is_beta;
}

void Inverse_PARK()
{
        Us_alpha = cos(Theta_S)*Us_d - sin(Theta_S)*Us_q;
        Us_beta  = sin(Theta_S)*Us_d + cos(Theta_S)*Us_q;
}

void angle_single_change(float current_ang, float last_ang, int* n )
{

    if(current_ang - last_ang > pi){
        *n = *n - 1;
    }else if(current_ang - last_ang < - pi){
        *n = *n + 1;
    }else{
        *n = *n;
    }
}

void Position_Calculate(void)
    {
      float Present_Motor_Position = 0.0;
      float Position_TEMP = 0.0;
      Present_Motor_Position = v;
//
    //1
        if ((Present_Motor_Position >=46980) && (Present_Motor_Position < 99390))
              {
                           Position_TEMP = Present_Motor_Position -46980;
                           Theta_S = (2.0*pi*Position_TEMP/52410.0);
              }
    //2
        if ((Present_Motor_Position >=99390) && (Present_Motor_Position < 151870))
              {
                           Position_TEMP = Present_Motor_Position -99390;
                           Theta_S = (2.0*pi*Position_TEMP/52480.0);
              }
    //3
        if ((Present_Motor_Position >= 151870) && (Present_Motor_Position <204109 ))
        {
            Position_TEMP = Present_Motor_Position-151870;
            Theta_S = (2.0*pi*Position_TEMP/52239.0);
        }
    //4
        if ((Present_Motor_Position >= 204109) && (Present_Motor_Position <256725))
        {
            Position_TEMP = Present_Motor_Position -204109;
           Theta_S = (2.0*pi*Position_TEMP/52616.0);
        }
    //5
        if ((Present_Motor_Position >= 256725) && (Present_Motor_Position < 309259))
        {
            Position_TEMP = Present_Motor_Position - 256725;
           Theta_S = (2.0*pi*Position_TEMP/52534.0);
        }
    //6
        if ((Present_Motor_Position >= 309259) && (Present_Motor_Position < 361520))
        {
            Position_TEMP = Present_Motor_Position - 309259;
            Theta_S =(2.0*pi*Position_TEMP/52261.0);
        }
    //7
        if ((Present_Motor_Position >= 361520) && (Present_Motor_Position <413857))
        {
            Position_TEMP = Present_Motor_Position - 361520;
           Theta_S =(2.0*pi*Position_TEMP/52337.0);
        }
    //8
        if ((Present_Motor_Position >= 413857) && (Present_Motor_Position < 466253))
        {
            Position_TEMP = Present_Motor_Position -413857;
            Theta_S = (2.0*pi*Position_TEMP/52396.0);
        }
   //9
        if ((Present_Motor_Position >= 466253) && (Present_Motor_Position <518767))
        {
           Position_TEMP = Present_Motor_Position - 466253;
           Theta_S = (2.0*pi*Position_TEMP/52514.0);
        }
   //10
        if ((Present_Motor_Position >=518767) && (Present_Motor_Position < 524287))
        {
             Position_TEMP = Present_Motor_Position - 518767;
             Theta_S = (2.0*pi*Position_TEMP/52503.0);
        }
        if ((Present_Motor_Position >=0) && (Present_Motor_Position < 46983))
        {
             Position_TEMP = Present_Motor_Position + 524287- 518767;
             Theta_S = (2.0*pi*Position_TEMP/52503.0);
        }

        //1
//                if ((Present_Motor_Position >=23980) && (Present_Motor_Position < 76453))
//                      {
//                                   Position_TEMP = Present_Motor_Position -23980;
//                                   Theta_S = (2.0*pi*Position_TEMP/52473.0);
//                      }
//            //2
//                if ((Present_Motor_Position >=76453) && (Present_Motor_Position < 128890))
//                      {
//                                   Position_TEMP = Present_Motor_Position -76453;
//                                   Theta_S = (2.0*pi*Position_TEMP/52437.0);
//                      }
//            //3
//                if ((Present_Motor_Position >= 128890) && (Present_Motor_Position <181173))
//                {
//                    Position_TEMP = Present_Motor_Position-128890;
//                    Theta_S = (2.0*pi*Position_TEMP/52283.0);
//                }
//            //4
//                if ((Present_Motor_Position >= 181173) && (Present_Motor_Position <233653))
//                {
//                    Position_TEMP = Present_Motor_Position -181173;
//                   Theta_S = (2.0*pi*Position_TEMP/52480.0);
//                }
//            //5
//                if ((Present_Motor_Position >= 233653) && (Present_Motor_Position < 286134))
//                {
//                    Position_TEMP = Present_Motor_Position - 233653;
//                   Theta_S = (2.0*pi*Position_TEMP/52481.0);
//                }
//            //6
//                if ((Present_Motor_Position >= 286134) && (Present_Motor_Position < 338594))
//                {
//                    Position_TEMP = Present_Motor_Position - 286134;
//                    Theta_S =(2.0*pi*Position_TEMP/52460.0);
//                }
//            //7
//                if ((Present_Motor_Position >= 338594) && (Present_Motor_Position <391061))
//                {
//                    Position_TEMP = Present_Motor_Position - 338594;
//                   Theta_S =(2.0*pi*Position_TEMP/52467.0);
//                }
//            //8
//                if ((Present_Motor_Position >= 391061) && (Present_Motor_Position < 443330))
//                {
//                    Position_TEMP = Present_Motor_Position -391061;
//                    Theta_S = (2.0*pi*Position_TEMP/52269.0);
//                }
//           //9
//                if ((Present_Motor_Position >= 443330) && (Present_Motor_Position <495795))
//                {
//                   Position_TEMP = Present_Motor_Position - 443330;
//                   Theta_S = (2.0*pi*Position_TEMP/52465.0);
//                }
//           //10
//                if ((Present_Motor_Position >=495795) && (Present_Motor_Position < 524287))
//                {
//                     Position_TEMP = Present_Motor_Position - 495795;
//                     Theta_S = (2.0*pi*Position_TEMP/52472.0);
//                }
//                if ((Present_Motor_Position >=0) && (Present_Motor_Position < 23980))
//                {
//                     Position_TEMP = Present_Motor_Position + 524287 - 495795;
//                     Theta_S = (2.0*pi*Position_TEMP/52472.0);
//                }

   }



/******************************************************************************************/
//Speed Calculate.
/******************************************************************************************/
Uint32 Position_Rota10k[2] = {0,0};
float Speed_Current_temp10k;
void sCalculation10k(void)
{


        Position_Rota10k[1] = v;
        Speed_Current_temp10k = ((float)(Position_Rota10k[1])) -((float) Position_Rota10k[0]);

        if(Speed_Current_temp10k >262143)
            {
            Speed_Current_temp10k -= 524287;
           }
         if(Speed_Current_temp10k <-262143)
            {
            Speed_Current_temp10k +=524287;
           }
         speed10k = (Speed_Current_temp10k * 10000.0) / 524287.0;
        // speed10k =speed10k * 2 * pi;
        Speed = speed10k;
         Position_Rota10k[0] = Position_Rota10k[1];
}
/******************************************************************************************/
//Position PID Controller Here.
/******************************************************************************************/

float pKP = 0.0;//0.05;
float pKI = 0.0;//0.00625;
float pKD = 0.0;
float pREF = 0.0;//Current Reference
float pFB = 0.0;//Current Feedback
float pERR = 0.0;//snput Error of Current PsD
float P_intergral = 0.0;
float pERR1 = 0.0;
float pERR2 = 0.0;
float pOUT = 0.0;
float pOUT_LAST = 0.0;
float pOUT_UP_LIM = 3.0;
float pOUT_DOWN_LIM = -3.0;
int   testp = 0;
float pp = 1.0;
float t = 0 ;
float p_temp = 0;
float pFB_last = 0;
float pOUT1;

float J = 0.0;
float b0 = 25.0;
float b1 = 10.0;
float b2 = 0.0;
float b3 = 0.0;
float b4 = 0.0;
float C  = 0.0;
float Fs = 0.0;

float Jd = 0.0;
float Cd = 0.0;
float Fsd = 0.0;
float b2d = 0.0;
float b1d =0.0;
float b3d =0.0;
float b4d =0.0;
float ppp = 0.0;

float c1 = 0.7;
float c2 = 0.3;

  float z;
  float X1;
  float X2;
  float X3;
  float X4;

float pd =0.0;
float pdd =0.0;

float t_interval = 0.0001;
float pCONTROLLER(float qref,float q,float qd,float gq)
{
//    if(testp == 1)
//    {


    pERR = q-qref;
    pOUT = (-pKP*pERR) + (pKD*qd) + gq;

    pERR1 = pERR;
    pERR2 = pERR1;
    pOUT_LAST = pOUT;

    return pOUT;
}
//float pCONTROLLER(void)
//{
////    if(testp == 1)
////    {
//float m0 = (0.02*pERR + 0.052*(Speed - pd));
////    float m0 = (0.0013*pERR + 0.0125*(Speed - pd));
//
//
//
//        //float t_interval = 0.0001;
//
//
//      //  if (pREF < 0) pREF =  pREF +  (2 *pi);
////    }//
//  //  t = t + t_interval;
// //   pREF = ppp*sinf(2*pi*pp*t) ;
     // angle_single_change(pFB, pFB_last,&cyclenum);
//    pFB_last = pFB;
//   //if (( pi < pFB) && (pFB < 2*pi))
//    p_temp = 2*pi* cyclenum + pFB;
//   // p_temp = pFB -2*pi;
//    pERR = p_temp - pREF;
//   // pERR = pFB - pREF;
////    if(pERR > 300.0) pERR -= 360.0;
////    else if(pERR < -300.0) pERR += 360;
//    // pOUT = pOUT_LAST + ((pKP * (pERR - pERR1)) + (pKI * pERR) + (pKD * (pERR - 2 * pERR1 + pERR2)));
////    P_intergral =P_intergral + pERR;
////    if (P_intergral > 150)        P_intergral = 150;
////   else if(P_intergral < -150)  P_intergral = -150;
////    pOUT = (pKP * pERR)  +  (pKI * P_intergral) + (pKD * (pERR -  pERR1 ));
//
//
////
////
////
//  //  pd =((ppp*2*pi*pp)*cosf(2*pi*pp*t));
//    pdd = -(ppp*(2*pi*pp)*(2*pi*pp)*sinf(2*pi*pp*t));
//
//    z = pdd - (b1 * (Speed - pd)) - (b0*pERR);
//    X1 = Speed;
//    X2 = sgn(pd);
//
//
//    Jd = -0.00001*z*m0;
//    Cd = -0.001*Speed*m0;
//    Fsd = -0.1*sgn(Speed)*m0;
//
//
//    J = J + Jd *t_interval;
//    C = C + Cd *t_interval;
//    Fs =Fs +Fsd *t_interval;
//
//
//    pOUT1 = (J*z) + (C * X1) + (Fs * X2);
//    b1d = J *z;
//    b2d = C * X1;
//    b3d = Fs * X2;
//    pOUT = c1 * pOUT1 + c2 * pOUT_LAST;
//
//
//
//    if ( pOUT > pOUT_UP_LIM )         pOUT = pOUT_UP_LIM;
//    else if ( pOUT < pOUT_DOWN_LIM )  pOUT = pOUT_DOWN_LIM;
//    //if  ((-1 < pERR) && (pERR < 1))                   pOUT = 0.0;
//
//
//    pERR1 = pERR;
//    pERR2 = pERR1;
//    pOUT_LAST = pOUT;
//
//    return pOUT;
//}


/******************************************************************************************/
//Speed PID Controller Here.
/***********S*******************************************************************************/
float sKP = 0.3;//0.05;
float sKI = 0.006;//0.00625;
float sKD = 0.01;
float sREF;//Speed Reference
float sFB;// Speed Feedback
float sERR = 0.0;//snput Error of Current PsD
float sERR1 = 0.0;
float sERR2 = 0.0;
float s_intergral = 0.0;
float sOUT = 0.0;
float sOUT_LAST = 0.0;
float sOUT_UP_LIM = 10.0;
float sOUT_DOWN_LIM = -10.0;

float sCONTROLLER(void)
{
//  sFB = pFB - pFB_LAST;

    sERR = sREF - sFB;
    s_intergral += sERR;
    sOUT =  ((sKP * sERR ) + (sKI * s_intergral) + (sKD * (sERR - sERR1 )));
    if ( sOUT > sOUT_UP_LIM )         sOUT = sOUT_UP_LIM;
    else if ( sOUT < sOUT_DOWN_LIM )  sOUT = sOUT_DOWN_LIM;

    sERR1 = sERR;
    sERR2 = sERR1;
    sOUT_LAST = sOUT;
    return sOUT;
}

/******************************************************************************************/
//Force PID Controller Here.
/******************************************************************************************/
float Kk = 1.6e4;
float Dam = 7.5;
float BBd = 1.1;
float Bd = 0.0;
float F_KP = 0.0;
float Q = 0.0;//0.00625;
float F_KD = 0.0;
float F_REF =0.0;//Force Reference
float F_REFd =0.0;
float F_REFdd =0.0;
float Qd;
float Qdd;
float Qddd;
float F_FB;//Force Feedback
float F_FBd;


float F_ERR  = 0.0;//Input Error of Current PID
float F_OUT ;
float F_OUT_LAST = 0.0;
float FOUT_UP_LIM= 4;
float FOUT_DOWN_LIM = -4;
float ff;
float Jm = 0.6;
float Jc = 0.1;
float L1 = 0.01;
float L2 = 0.01;
float Ufm =0.0;
float Ucm =0.0;
float Fm_ERR;
float Fm_intergral =0.0;
float Fc_ERR;
float Fc_intergral =0.0;
float Kq=0.0;
float Kd=0.0;
float fff=0.0;
float fref;
float  f_d;

float Speedf =0.0;
float Speedlast=0.0;
float fCONTROLLER(float F_REF,float FFB,float F_FBdf,float loadd)
{


   // F_REF = -F_KP
    fref = (0.9995*fref) + (9.78e-5*f_d) + (0.0004927*F_REF);
    f_d = ( -9.78* fref) + (0.9562 * f_d) + (9.78*F_REF);

    F_KP = BBd - 1.0;//0.5;
    F_KD = (Dam * BBd)/Kk;
    Q = jFB;
  //  if (FFB<0.15 && FFB>-0.15) FFB=0;
    F_ERR = F_REF - FFB;


//    F_FBd = (F_FB - F_FB_LAST)*2000.0;
//    F_FBdf = (0.611*F_FBd) +(0.389*F_FBd_LAST);

    //CalculateQd(Q,Q_last);
  //  Qdddf = (0.239 * Qddd) +(0.761 * Qddd);
//     ff = ((0.6/Kk)*(F_REFdd - (Dam * Qddd)));
    ff = F_KD * (f_d - F_FBdf);
    F_OUT = (F_REF*0.10204) + (F_KP*F_ERR) + ff + fff; //+((0.5/Kk)*(F_REFdd - (Dam * Qddd)));

    fff = (Bd/Kk)*(-Dam*10.0*(FFBdf2-loadd));
    Fm_ERR = (F_OUT*9.8) - FFB -Ufm;
    Fm_intergral += (Fm_ERR/Jm);
    if (Fm_intergral > 150)        Fm_intergral = 150;
    else if(Fm_intergral < -150)   Fm_intergral = -150;
    Ufm =L1*Jm*(Fm_intergral - speed1k);



    //ff = ((0.6/Kk)*(F_REFdd - (Dam * Qddd)));
    if ( F_OUT> FOUT_UP_LIM)           F_OUT = FOUT_UP_LIM;
    else if ( F_OUT < FOUT_DOWN_LIM )  F_OUT = FOUT_DOWN_LIM;


//    F_FB_LAST = F_FB;
//    F_FBd_LAST = F_FBd;
    Speedlast = Speedf;
    F_OUT_LAST = F_OUT;

    return F_OUT;
}

/******************************************************************************************/
//d axis Current PID Controller Here.
/******************************************************************************************/
float iKP_d = 0.4;//0.5;
float iKI_d = 0.05;//0.00625;
float iKD_d = 0.0;

float iREF_d = 0.0;//Current Reference
float iFB_d = 0.0;//Current Feedback
float iERR_d = 0.0;//Input Error of Current PID
float iERR1_d = 0.0;
float iERR2_d = 0.0;
float iOUT_d = 0.0;
float iOUT_LAST_d = 0.0;
float iOUT_UP_LIM_d = 12;
float iOUT_DOWN_LIM_d = -12;
float id_intergral = 0.0;
float iCONTROLLER_d(float iREF_d,float iFB_d)
{

    iERR_d = iREF_d - iFB_d;
    id_intergral += iERR_d;
    if (id_intergral > 150)        id_intergral = 150;
    else if(id_intergral < -150)  id_intergral = -150;
   // iERR_d = 0.0 - iFB_d;
    //iOUT_d = iOUT_LAST_d + ((iKP_d * (iERR_d - iERR1_d)) + (iKI_d * iERR_d) + (iKD_d * (iERR_d - 2 * iERR1_d + iERR2_d)));
    iOUT_d = (iKP_d * iERR_d )+ (iKI_d * id_intergral)+ (iKD_d * (iERR_d - iERR1_d));
    if ( iOUT_d > iOUT_UP_LIM_d )         iOUT_d = iOUT_UP_LIM_d;
    else if ( iOUT_d < iOUT_DOWN_LIM_d )  iOUT_d = iOUT_DOWN_LIM_d;

    iERR1_d = iERR_d;
    //iERR2_d = iERR1_d;
    iOUT_LAST_d = iOUT_d;
    return iOUT_d;
}


/******************************************************************************************/
//q axis Current PID Controller Here.
/******************************************************************************************/
float iKP_q = 0.7;//0.05;
float iKI_q = 0.05;//0.00625;
float iKD_q = 0.0;

float iREF_q = 0.0;//Current Reference
float iREF_Q =0.0;
float iFB_q = 0.0;//Current Feedback
float iERR_q = 0.0;//Input Error of Current PID
float iERR1_q = 0.0;
float iERR2_q = 0.0;
float iOUT_q;
float iOUT_LAST_q = 0.0;
float iOUT_UP_LIM_q = 12;
float iOUT_DOWN_LIM_q = -12;
float iq_intergral = 0.0;
int cogtemp = 0;
float iOUTq=0.0;
float iCONTROLLER_q(float iREF_q,float iFB_q)
{




    Fc_ERR =(F_FB*0.1024) -iREF_q -Ucm;
    Fc_intergral += (Fc_ERR/Jc);
    if (Fc_intergral > 150)        Fc_intergral = 150;
    else if(Fc_intergral < -150)   Fc_intergral = -150;
   // Speedf =(0.5* speed1k) + (0.5*Speedlast);
    Ucm =L2*Jc*( Fc_intergral-jointspeed1k);
    iREF_Q =iREF_q  +  Ufm + Ucm;
    iERR_q = iREF_Q - iFB_q;
//    iERR_q = 2.0 - iFB_q;
    iq_intergral += iERR_q;
    if (iq_intergral >150)        iq_intergral = 150;
    else if(iq_intergral < - 150)  iq_intergral = -150;

   // iOUT_q = iOUT_LAST_q + ((iKP_q * (iERR_q - iERR1_q)) + (iKI_q * iERR_q) + (iKD_q * (iERR_q - 2 * iERR1_q + iERR2_q)));
    iOUT_q = (iKP_q * iERR_q )+ (iKI_q * iq_intergral) + (iKD_q * (iERR_q - iERR1_q));

    if ( iOUT_q > iOUT_UP_LIM_q )        iOUT_q = iOUT_UP_LIM_q;
    else if ( iOUT_q < iOUT_DOWN_LIM_q )  iOUT_q = iOUT_DOWN_LIM_q;

    iERR1_q = iERR_q;
  //  iERR2_q = iERR1_q;
    iOUT_LAST_q = iOUT_q;

    return iOUT_q;
}

/******************************************************************************************/
// Adaptive Controller Here.
/******************************************************************************************/

float i_de = 0;
float i_qe = 0;
float i_dot_de = 0;
float i_dot_qe = 0;
float i_q = 0;
float i_d = 0;
float q_d = 0;

float L_dd = 0;
float L_qd = 0;
float R_d = 0;
float Ke_d= 0;


float L_d = 0.0000;
float L_q = 0.0000;



float beta  = 1;
float alpha = 1;

float a1 = 0;
float a2 = 0;
float x1=0;
float x2=0;
float aaa = 0.5;
 float aa = 2;
float Uq = 0;
int testr1 = 1;
float last_i_qe;



void iCONTROLLER(void)

{
   // int n = 400;
   //float sgn;
   float t_interval = 0.0001;


   //参考电流(正弦）
   t = t + t_interval ;
   i_de  = 0;
   i_dot_de = 0 ;
   i_d = iFB_d;
   i_q = iFB_q;
   q_d = speedf;
  // q_d = - 2*pi*speedwe;
   if (testr1 == 0)
   {
   i_qe = aaa*sinf(2*pi*aa*t);
   iREF_q =i_qe;
   i_dot_qe = aaa*2*aa*pi*cosf(2*pi*aa*t);


   //
        a1 = i_dot_de - beta  * (i_d - i_de);
        a2 = i_dot_qe - alpha * (i_q - i_qe);

     //   a1 = i_dot_de - beta  * (i_d - i_de) + p1*i_de - 10*q_d*i_qe;
     //   a2 = i_dot_qe - alpha * (i_q - i_qe) + p1*i_qe+ 10*q_d*i_de+ p2*q_d;
     //   if (a2 > 1000)
     //   {
     //       L_d = a2;
     //   }

        x1 = i_d - i_de;
        x2 = i_q - i_qe;


     //   if(x2 > 0 )
     //   {
     //       L_d = x1;
     //   }

        L_qd = -0.001*(((a1-(q_d*i_qe))*x1) + ((a2+(q_d*i_de))*x2));
       // L_dd = -0.003*((a1*x1) + (q_d*i_de*x2));
        R_d  = -0.1*((i_de*x1) +(i_qe*x2));
        Ke_d = -0.1*(q_d*x2);

     //
     //   L_d = L_d + L_dd*t_interval;
     //   L_q = L_q + L_qd*t_interval;
     //   R   = R + R_d*t_interval;
     //   Ke  = Ke + Ke_d*t_interval;
     //
     //   p1_d =  (-i_qe*x2) + (-i_de*x1) ;
     //   p2_d =  (-q_d*x2);
     //   p3_d =  (-a2*x2)+(-a1*x1);
     //
     //   p1 = p1+p1_d * t_interval;
     //   p2 = p2+p2_d * t_interval;
     //   p3 = p3+p3_d * t_interval;
     //
     //
     //   L = p3;
     //   R = p1*p3;
     //   Ke = p2*p3;

       // L_d = L_d + L_dd*t_interval;
        L_q = L_q + L_qd*t_interval;
        R = R + R_d*t_interval;
        Ke = Ke + Ke_d*t_interval;



  //   i_qe  = aaa ;
  //   i_dot_qe = 0 ;
        Us_d = L_q * (i_dot_de - beta*(i_d - i_de)) + R*i_de- 10*q_d*L_q*i_qe;
        Us_q = L_q * (i_dot_qe - alpha*(i_q - i_qe)) + R*i_qe + Ke*10*q_d + 10*q_d*L_q*i_de;


   }

   else if(testr1 == 1)
          {
           i_qe = iREF_q;
           //i_dot_qe = (i_qe - last_i_qe) * 10000;
//         i_qe = aaa;//*sinf(2*pi*aa*t);
//         iREF_q =i_qe;
           i_dot_qe = 0;//aaa*2*aa*pi*cosf(2*pi*aa*t);
           i_de = 0 ;

//           Us_d = 0;

           a1 = i_dot_de - beta  * (i_d - i_de);
           a2 = i_dot_qe - alpha * (i_q - i_qe);
//
                  x1 = i_d - i_de;
                  x2 = i_q - i_qe;
//

//             if((-0.1 > Speed) && (Speed > 0.1))
//            {
//                  if((F_FB > 0.2) || (F_FB < -0.2))
//                  {
//                   i_qff =-sgn(F_FB)*Uff + (0.065 * Speed);
//                  }
//                  else
//                  {
//                      i_qff=0.0;
//                  }
//             }

            // i_qe = iREF_q + Ufm ;
             Us_d = L_q * (i_dot_de - beta*(i_d - i_de)) + R*i_de- 10*q_d*L_q*i_qe;
             Us_q=  L_q  * ( i_dot_qe- alpha*(i_q - i_qe)) + R*i_qe + Ke*10*q_d + 10*q_d*L_q*i_de;
             //Us_q = Us_q + (sgn*0.4);
             //Us_d = 0;


//             Us_d = L_q *i_dot_de + R*i_de - 10*q_d*L_q*i_qe;
//             Us_q= L_q * i_dot_qe  + R*i_qe + Ke*10*q_d + 10*q_d*L_q*i_de;
//             if  ((pFB >=0 ) && (pFB <= 72))
//             {

//                 int cogtemp;
//                 cogtemp = pFB;
//                 Us_q=Uq + cogUq[cogtemp];
////             }
              // else  Us_q =Uq;
          }

          last_i_qe = i_qe;





//   Us_d = L * (i_dot_de - beta  * (i_d - i_de))  + R*i_de - q_d*L*i_qe;
//   Us_q = L * (i_dot_qe - alpha * (i_q - i_qe))  + R*i_qe + q_d*L*i_de + Ke*q_d;
//   Uq = Us_q;
//   if(i_q > )
//
   if(Us_d > 24)
   {
       Us_d = 24;
   }

   if(Us_q > 24)
     {
         Us_q = 24;
     }

   if(Us_q < -24)
       {
           Us_q = -24;
       }

   if(Us_d < -24)
       {
           Us_d =  -24;
       }

}

uint8_t tableCRC6[64] = {
 0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
 0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
 0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
 0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
 0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
 0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02};
int crcBiSS(uint32_t bb)
 {
 uint8_t crc;
 uint32_t t;
 t = (bb >> 18) & 0x00000003;
 crc = ((bb >> 12) & 0x0000003F);
 t = crc ^ tableCRC6[t];
 crc = ((bb >> 6) & 0x0000003F);
 t = crc ^ tableCRC6[t];
 crc = (bb & 0x0000003F);
 t = crc ^ tableCRC6[t];
 crc = tableCRC6[t];
 return crc;
 }
  uint32_t CRC = 0;
  uint32_t datapkg = 0;
void recivePosition(void)
{
       uint32_t temp1 = 0;
       uint32_t temp2 = 0;

       temp1 = RxData[1]&0x00007fff;
       temp2 = (RxData[2]>>12)&0x0000000f;

       position = (temp1<<4)|temp2;
       v=position;
       v = ((v >> 1) & 0x55555555) | ((v & 0x55555555) << 1);
       // 交换连续数对
       v = ((v >> 2) & 0x33333333) | ((v & 0x33333333) << 2);
       // 一点点的交换
       v = ((v >> 4) & 0x0F0F0F0F) | ((v & 0x0F0F0F0F) << 4);
       // 交换字节
       v = ((v >> 8) & 0x00FF00FF) | ((v & 0x00FF00FF) << 8);
       // 交换2字节长数对
       v = (v >> 16) | (v << 16);
       v = (v >> 13);
      //  position =(position >> 13)


}

void reciveJointPosition(void)
{
       uint32_t temp1 = 0;
       uint32_t temp2 = 0;

       temp1 = RxData2[1]&0x00007fff;
       temp2 = (RxData2[2]>>11)&0x0000001f;

       datapkg = (temp1<<5)|temp2;

       datapkg = ((datapkg >> 1) & 0x55555555) | ((datapkg & 0x55555555) << 1);
             // 交换连续数对
       datapkg = ((datapkg >> 2) & 0x33333333) | ((datapkg & 0x33333333) << 2);
             // 一点点的交换
       datapkg = ((datapkg >> 4) & 0x0F0F0F0F) | ((datapkg & 0x0F0F0F0F) << 4);
             // 交换字节
       datapkg = ((datapkg >> 8) & 0x00FF00FF) | ((datapkg & 0x00FF00FF) << 8);
             // 交换2字节长数对
       datapkg = (datapkg >> 16) | (datapkg << 16);
       datapkg = (datapkg >> 12);
       joint_position= datapkg;

}
 //
float Position_Rota1k[2] = {0,0};
float Speed_Current_temp1k;
void sCalculation1k(void)
{

    Position_Rota1k[1] = pFB;
    Speed_Current_temp1k = ((float)(Position_Rota1k[1])) -((float) Position_Rota1k[0]);

    if(Speed_Current_temp1k > pi)
        {
        Speed_Current_temp1k -= 2*pi;
       }
     if(Speed_Current_temp1k <-pi)
        {
        Speed_Current_temp1k +=2*pi;
       }
     speed1k = (Speed_Current_temp1k * 1000.0);
    // speed1k = speed1k*2*pi;
     Position_Rota1k[0] = Position_Rota1k[1];
     //sign()

}

float Lpf_in_prev[2];
float Lpf_out_prev[2];
float Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
float lpf_out;

float digital_lp_filter(float w_c, float t_s, float lpf_in)
{

    float den = 2500*t_s*t_s*w_c*w_c  + 7071*t_s*w_c + 10000;

    Lpf_in1 = 2500*t_s*t_s*w_c*w_c / den;
    Lpf_in2 = 5000*t_s*t_s*w_c*w_c / den;
    Lpf_in3 = 2500*t_s*t_s*w_c*w_c / den;
    Lpf_out1 = -(5000*t_s*t_s*w_c*w_c  - 20000) / den;
    Lpf_out2 = -(2500*t_s*t_s*w_c*w_c  - 7071*t_s*w_c + 10000) / den;

    lpf_out = Lpf_in1*lpf_in + Lpf_in2*Lpf_in_prev[0] + Lpf_in3*Lpf_in_prev[1] + //input component
    Lpf_out1*Lpf_out_prev[0] + Lpf_out2*Lpf_out_prev[1]; //output component
    Lpf_in_prev[1] = Lpf_in_prev[0];
    Lpf_in_prev[0] = lpf_in;
    Lpf_out_prev[1] = Lpf_out_prev[0];
    Lpf_out_prev[0] = lpf_out;
    return lpf_out;

}



int sgn(float x)
{
  if(x > 0)
    return 1;
  else if(x == 0)
    return 0;
  else
    return -1;
}

float jointPosition_Rota1k[2] = {0,0};
float jointSpeed_Current_temp1k;

void jsCalculation1k(void)
{

    jointPosition_Rota1k[1] = jFB;
    jointSpeed_Current_temp1k = ((float)(jointPosition_Rota1k[1])) -((float) jointPosition_Rota1k[0]);

    if(jointSpeed_Current_temp1k >pi)
        {
        jointSpeed_Current_temp1k -= 2*pi;
       }
     if(jointSpeed_Current_temp1k <-pi)
        {;
        jointSpeed_Current_temp1k +=2*pi;
       }
     jointspeed1k = (jointSpeed_Current_temp1k * 1000.0);

    // speed1k = speed1k*2*pi;
     jointPosition_Rota1k[0] =jointPosition_Rota1k[1];
     //sign()

}


float fst(float x1, float x2,float v, float r,float h)
{
    float deta = r * h;
    float deta0 = deta * h;
    float y = x1-v+(h*x2);
   // if (x1 < 0) x1+=2*pi;
//    if ((x1-v) > pi) y = y-(2 *pi);
//    if ((x1-v) < -pi)  y = y+(2 *pi);


    float a0 = sqrt((deta*deta) + 8*r*fabs(y));
    float a;
    if(fabs(y) <= deta0)  a = x2+(y/h);
    else a = x2+(0.5*(a0 -deta)*sgn(y));

     if (fabs(a)<=deta)  y=-r*a/deta;
     else y=-r*sgn(a);

     return y;
}

