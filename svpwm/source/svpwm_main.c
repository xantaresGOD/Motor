//#############################################################################
//
// FILE:   empty_bitfield_driverlib_main.c
//
// TITLE:  Empty Example
//
// Empty Bit-Field & Driverlib Example
//
// This example is an empty project setup for Bit-Field and Driverlib 
// development.
//
//#############################################################################
// $TI Release: F2838x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 19:08:49 IST 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 

// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//

#include "f28x_project.h"
#include "driverlib.h"
#include "device.h"
#include "math.h"
#include "IQmathLib.h"
#include "clb_config.h"
#include "clb.h"
#include "stdio.h"
#include "stdlib.h"
#include "speed_observer.h"
#include "svgen.h"
//#include "test.h"
//#include "test.h"
//#include "printUART.h"
//#include "PM_bissc_Include.h"

// IPC ADD
#define IPC_CMD_READ_MEM   0x1001
#define IPC_CMD_RESP       0x2001
#define TEST_PASS          0x5555
#define TEST_FAIL          0xAAAA
uint32_t pass;

#pragma DATA_SECTION(readData, "MSGRAM_CPU_TO_CM")
int dataready = 0;
float coguq[3600];
float cogpos[3600];
float tempuq[1000];
#define INV_PWM_HALF_TBPRD  2500
float iq_err = 0.0;
extern float p_temp;
extern float fff;
// data struct

//struct Data
//{
//    float
//}
int32_t readData[10];
int32_t writeData[10];

//#constant pragrma
#define sqrt3     1.732050808
#define pi        3.141592654
#define EN_GPIO   73
int  cyclenum = 0;
float j_init =0.0;
#define FIFO_LEVEL     3
// AD sample coeff
#define  V_offset       1.568
#define  Sensor_Coeff  10.0
#define  Force_Sensor_Coeff  0.01719
float tt=0;
float ww=0;
float ss =0;
extern float ppp;
extern float t;
float rd = 80;
double djFB;


uint16_t RxData[FIFO_LEVEL]  = {0};
uint16_t RxData2[FIFO_LEVEL] = {0};
float h[5] = {-3,12,17,12,-3};
//uint16_t RxData[FIFO_LEVEL] ;
 uint32_t TxData[FIFO_LEVEL] ;
uint32_t Position = 0;
int re = 0;
int fc = 0;

uint16_t AdcaResult0;
uint16_t AdcaResult1;
uint16_t AdcaResult2;
uint16_t AdcaResult3;

uint32_t AdcaResult4;
uint32_t AdcaResult5;

uint16_t AdcbResult0;
uint16_t AdcbResult1;
uint16_t AdcbResult2;
uint16_t AdcbResult3;
uint16_t AdcbResult4;
uint16_t AdcbResult5;
float force1;
float force2;
float force3;
float force4;

uint16_t AdccResult0;
uint16_t AdccResult1;
uint16_t AdccResult2;
uint16_t AdccResult3;
uint16_t AdccResult4;
uint16_t AdccResult5;
uint16_t AdccResult6;
uint16_t AdccResult7;
uint16_t AdccResult8;

uint16_t AdcdResult0;
uint16_t AdcdResult1;
uint16_t AdcdResult2;
uint16_t AdcdResult3;
uint16_t AdcdResult4;
uint16_t AdcdResult5;
uint16_t AdcdResult6;
uint16_t AdcdResult7;
uint16_t AdcdResult8;
float ADCC;
float ADCB;
float ADCD;
float F_FB2;
float Loadd;
//uint16_t interruptCount = 0;
//uint32_t count =0;
#define SVGEN_DEFAULTS { 0,0,0,0,0 }
//ADvariables define
SVGEN svgen1 =  SVGEN_DEFAULTS;
uint16_t Ia = 0;
uint16_t Ib = 0;


float Va_fb=0;
float Vdc_fb = 0;
extern float Us_alpha ;
extern float Us_beta ;
float speedf;

float Ibus = 0;
extern float iREF_d;
extern float iFB_d;
extern float iREF_q;
extern float iFB_q;
extern float L_q;
extern float F_REF;
float fbb;

extern float sREF;//Current Reference
extern float sFB;//Current Feedback
extern float speed1k;
extern float speed10k;
extern float Uamp;
float kalmanspeed = 0.0;

extern float pREF;
extern float pFB;
extern float pERR;


float Is_a = 0;
float Is_b = 0;
float Is_q = 0;
float Is_d = 0;

int ADcnt = 0 ;

float Ia_av = 0;
float Ib_av = 0;
extern uint32_t position;
extern uint32_t joint_position;
extern float aa;
extern float aaa;
float jFB;
float ia =0;
float ib =0;
extern int Time_C_a ;
extern int Time_C_b ;
extern int Time_C_c ;

extern float Us_d = 0;
extern float Us_q = 0;
extern float i_qe;
 float FFBdf2;
 float FFBdf1;
extern float F_FBd;
float F_FB_LAST;
float F_FB2_LAST;
float F_FBd_LAST;
float Theta_S = 0;
float Theta_fb = 0;
float temp = 0;
int cnt = 0;

float absjFB =0.0;
float jREF = 0.0;
float Fcoefficient = 32440;
float jERR1= 0.0;
float pFBnew =0.0;
float pFBold =0.0;

extern float F_REF;
extern float F_FB;
float eFB;
float eFBd;
int cogtest;
float  FFBdf2_LAST;

//}

static int ready =0;

SPD_OBSERVER Speed1 = SPD_OBSERVER_DEFAULTS;

float Speed = 0;
extern float Fff;
float tmr1,tmr2,tmr3;
extern float t_interval;
extern float pp;
extern float cogctl;
extern float pERR;

// Globals
//
//  Function Prototypes
//
void Kalmanspeed(void);
void bissc_configEPWM4(void);
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
__interrupt void epwm1_isr(void);

void Inverse_PARK(void);
void SVPWM(void);
void ePWMGen(void);
void initEQEP(void);

void InitGPIOset(void);
void ConfigureADC(void);
void SetupADCSoftware(void);
void initPosE(void);

void recivePosition(void);
int  crcBiSS(uint32_t bb);

void endat1_resetCLB(void);
void endat1_setupCommand(void);
void endat1_startOperation(void);
void endat1_setupGPIO(void);
void endat1_configXBAR(void);
void endat1_initSPIFIFO(void);
void endat1_configEPWM5(void);
void endat1_init(void);
__interrupt void endat1_spiRxFifoIsr(void);

void endat2_resetCLB(void);
void endat2_setupCommand(void);
void endat2_startOperation(void);
void endat2_setupGPIO(void);
void endat2_configXBAR(void);
void endat2_initSPIFIFO(void);
void endat2_configEPWM4(void);
void endat2_init(void);
__interrupt void endat2_spiRxFifoIsr(void);

void angle_single_change(float current_ang, float last_ang, int* n );
float iCONTROLLER_q(float iREF_q,float iFB_q);
float iCONTROLLER_d(float iREF_d,float iFB_d);
float sCONTROLLER(void);
float pCONTROLLER(float qref,float q,float qd,float gq);
float fCONTROLLER(float F_REF,float FFB,float F_FBdf,float loadd);
float digital_lp_filter(float w_c, float t_s, float lpf_in);
void jsCalculation1k(void);
float fst(float x1, float x2,float v, float r,float h);


extern float pos32val;
extern float q_d;
extern float R;
extern float Ke;
extern uint32_t v;
extern int last_timea;
int erra;
float fREF  ;
extern float pOUT;
extern float pOUT1;
//float PosM;
//int ik = 0;
//float ia_av;
//float ia_a;
//float ThetaE;
int flag;
//char TempChar ;
int cogcal = 0;
int slect = 0;
float Ia_Buffer[3] = {0};
float Ib_Buffer[3] = {0};
float force1_Buffer[10] = {0};
float force2_Buffer[10] = {0};
float Ia_Current = 0;
int posflag = 0;
char str[40];
float F_FB1;
int ctlcnt;
float eFBd_LAST;
float ff2;
extern float F_OUT;
extern float F_ERR;
//float Ufw[360];
//float Ubw[360];
//float actfw[360];
//float actbw[360];
//
// Main
//
int16_t testIq;
int16_t testIqe;
int16_t speedtest;
int16_t speedref;
//uint32_t positiontest;
int32_t positionref;
int32_t positiontest;
int16_t forceref;
int16_t forcetest;
int16_t testUq;
int16_t testUd;
int16_t Ta;
int16_t Tb;
int16_t Tc;

uint64_t counts  = 0;
uint64_t counts1 = 0;
uint64_t counts2 = 0;

float pFBE = 0.0;
float speedwe = 0.0;
extern float Uff;
extern float pd;
extern float F_KD;
float Load;
float jSpeedf;
float jFB_last;
extern float jointspeed1k;
extern float Speedf;
extern float Ufm;
extern float ff;
float espeed =0;
float epos = 0;
float pspeed;
extern float F_KP;
float F[5] ={0};
extern float fref;
extern float f_d;
extern float i_dot_qe;
extern float Ucm;
void main(void)
   {

    IPC_MessageQueue_t messageQueue;
    IPC_Message_t      TxMsg, RxMsg;
     Us_alpha = 0;
     Us_beta  = 0;
   //InitSysCtrl();
     Device_init();
     Device_initGPIO();

     // Boot CM core
     //
// #ifdef _FLASH
//     Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
// #else
//     Device_bootCM( BOOTMODE_BOOT_TO_S0RAM);
// #endif
     //
     Interrupt_initModule();
     Interrupt_initVectorTable();

//
// Initialize GPIO:
// This example function is found in the f2838x_gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
    InitGpio();
    InitGPIOset();
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    scic_init();
//    bissc_setupGPIO();
//    bissc_configXBAR();

    //Device_bootCPU2(BOOTMODE_BOOT_TO_M0RAM);
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM3=1;
    CpuSysRegs.PCLKCR2.bit.EPWM4=1;
    CpuSysRegs.PCLKCR2.bit.EPWM5=1;

//

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 1000000);
    CpuTimer0Regs.TCR.bit.TSS = 0;
//


    DINT;

    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;


//
   Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
    InitPieVectTable();

//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
//
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm1_isr;
   //PieVectTable.SPIB_RX_INT = &bissc_spibRxFifoIsr;
    PieVectTable.SPIA_RX_INT = &endat1_spiRxFifoIsr;
    PieVectTable.SPIB_RX_INT = &endat2_spiRxFifoIsr;
   // PieVectTable.SPIA_RX_INT = &bissc_spiaRxFifoIsr;
    //Interrupt_register(INT_SPIB_RX, &bissc_spiRxFifoIsr);
        // Enable interrupts required for this example

   // PieVectTable.EPWM2_INT = &epwm2_isr;
   // PieVectTable.EPWM3_INT = &epwm3_isr;
    EDIS;   // This is needed to disable write to EALLOW protected registers

//
// For this example, only initialize the ePWM
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPwm1Example();
    InitEPwm2Example();
    InitEPwm3Example();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
//
// Step 4. User specific code, enable interrupts:
//
// Enable CPU INT3 which is connected to EPWM1-3 INT:
    Interrupt_enable(INT_SPIA_RX);
    Interrupt_enable(INT_SPIB_RX);
    Interrupt_enable(INT_EPWM1 );
   //  IER |= M_INT3;


//
// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
//
      PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
   // PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
   // PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
      //initEQEP();
      ConfigureADC();
      SetupADCSoftware();
//      R = 0.82;
//      L_q = 0.0078;
//      Ke = 0.008;


//         IPC_clearFlagLtoR(IPC_CPU1_L_CM_R, IPC_FLAG_ALL);
//
//
//           //Initialize message queue
//
//          IPC_initMessageQueue(IPC_CPU1_L_CM_R, &messageQueue, IPC_INT1, IPC_INT1);
//
//
//         //  Synchronize both the cores
//
//          IPC_sync(IPC_CPU1_L_CM_R, IPC_FLAG31);


// Enable global Interrupts and higher priority real-time debug events:
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);

//        bissc_setupGPIO();
//        bissc_configXBAR();
          endat1_init();
          endat2_init();
          CLB_enableCLB(CLB5_BASE);
          CLB_enableCLB(CLB4_BASE);
          initTILE1(CLB5_BASE);
          initTILE0(CLB4_BASE);
         // initSPIBSlave();
//        initSPIASlave();
         // bissc_configEPWM4();
     // CLB_enableCLB(CLB4_BASE);

                   // Send a message without message queue
                   // Since C28x and CM does not share the same address space for shared RAM,
                   // ADDRESS_CORRECTION is enabled
                   // Length of the data to be read is passed as data.
                   //
//                            readData[0] = 2;
//                            readData[1] = F_FB;
//                            readData[2] = iFB_q;
//
        while(1)
           {

//           float temp1;
//           float temp2;
//           int errorflag;
//
//          if (cogtest ==1)
//          {
//              for (temp1=0; temp1<360; temp1++)
//              {
//
//                  pREF = temp1;
//                  temp2 = temp1;
//                  DELAY_US(50);
//                  while(Speed != 0 );
//                  Ufw[temp2] =Us_q;
//                  actfw[temp2] =pFB;
//              }
//
//          }



            //    if(dataready == 1)
               //DELAY_US(1000L);
            // while(ready == 0){}


            //while (ready != 1);


            //DEVICE_DELAY_US(1000L);

            //     dataready = 0;
//            EALLOW;
//            GpioDataRegs.GPESET.bit.GPIO143 = 1;
//            GpioDataRegs.GPESET.bit.GPIO144 = 1;
//            DELAY_US(100000L);
//            GpioDataRegs.GPECLEAR.bit.GPIO143 = 1;
//            GpioDataRegs.GPECLEAR.bit.GPIO144 = 1;
//            DELAY_US(100000L);
//            EDIS;

                 // CLB_disableCLB(CLB4_BASE);
                    //  DEVICE_DELAY_US(1000L);


           if (GpioDataRegs.GPCDAT.bit.GPIO70 == 0)

           {
               EALLOW;
               GPIO_WritePin(EN_GPIO, 0);
               Us_q = 0.0;
//               CpuSysRegs.PCLKCR2.bit.EPWM1=0;
//               CpuSysRegs.PCLKCR2.bit.EPWM2=0;
//               CpuSysRegs.PCLKCR2.bit.EPWM3=0;
//               EPwm1Regs.CMPA.bit.CMPA=0;
//               EPwm2Regs.CMPA.bit.CMPA=0;
//               EPwm3Regs.CMPA.bit.CMPA=0;
//               EPwm1Regs.CMPB.bit.CMPB=0;
//               EPwm2Regs.CMPB.bit.CMPB=0;
//               EPwm3Regs.CMPB.bit.CMPB=0;


               EDIS;
           }
           else if(GpioDataRegs.GPCDAT.bit.GPIO70 == 1)
           {
               EALLOW;
               GPIO_WritePin(EN_GPIO, 1);
              // Us_q = 0.5;
               Us_d = 0;
//               CpuSysRegs.PCLKCR2.bit.EPWM1=1;
//               CpuSysRegs.PCLKCR2.bit.EPWM2=1;
//               CpuSysRegs.PCLKCR2.bit.EPWM3=1;
               EDIS;
           }

           //      Update the message

//                TxMsg.command = IPC_CMD_READ_MEM;
//                TxMsg.address = (uint32_t *)readData;
//                TxMsg.dataw1  = 10;  // Using dataw1 as data length
//                TxMsg.dataw2  = 1;   // Message identifier
//
//                //
//                // Send message to the queue
//                // Since C28x and CM does not share the same address space for shared RAM,
//                // ADDRESS_CORRECTION is enabled
//                //
//                IPC_sendMessageToQueue(IPC_CPU1_L_CM_R, &messageQueue, IPC_ADDR_CORRECTION_ENABLE,
//                                       &TxMsg, IPC_BLOCKING_CALL);
//
//                //
//                // Read message from the queue
//                // Return message from CM does not use the address field, hence
//                // ADDRESS_COREECTION feature is not used
//                //
//                IPC_readMessageFromQueue(IPC_CPU1_L_CM_R, &messageQueue, IPC_ADDR_CORRECTION_DISABLE,
//                                         &RxMsg, IPC_BLOCKING_CALL);
//
//                if((RxMsg.command == IPC_CMD_RESP) && (RxMsg.dataw1 == TEST_PASS) && (RxMsg.dataw2 == 1))
//                    pass = 1;
//                else
//                    pass = 0;  //

//            // Fill i+n the data to be sent
//            //
            if (GpioDataRegs.GPEDAT.bit.GPIO149 == 0)
            {
                int error_count;
                error_count++;
                if(error_count > 100)

                {
                    EALLOW;
                    CpuSysRegs.PCLKCR2.bit.EPWM1=0;
                    CpuSysRegs.PCLKCR2.bit.EPWM2=0;
                    CpuSysRegs.PCLKCR2.bit.EPWM3=0;
                    GPIO_WritePin(EN_GPIO, 0);
                    EDIS;
                  ESTOP0;

                }
                else
                {
                    error_count = 0;
                }

              }


            testIq  = fREF * 1000;
            testIqe = fref * 1000;
            speedtest = speedf * 100;
            speedref = F_FB2 * 100;
           // positionref =  Speed*10;
            //forceref = Ke *1000;
            forceref =  Ucm*1000;
            forcetest = F_FB*1000;
           // positiontest = position;//jFB*10;
             positionref = FFBdf2*1000;
             positiontest = speedf*1000;
             testUq = pOUT * 100;
             testUd = pOUT1 * 100;
          sprintf(str,"Ia:%d,%d,%d,%d\n",testIq,testIqe,forceref, forcetest);
//            //sprintf(str,"Ia:%d,%d,%d\n",speedref,D,testIq);
           // sprintf(str,"Ia:%u,%u,%d,%d,%d\n",Ia,Ib,speedtest,speedref,forceref);
           //sprintf(str,"Ia:%d,%d,%d,%d\n",testIq,testUd,forcetest,forceref);
            //sprintf(str,"Ia:%u,%u,%d\n",AdccResult0,AdccResult1,forcetest);
            SCI_writeCharArray(SCIC_BASE, (uint16_t*)str,32);
           }
}

//
// epwm1_isr - EPWM1 ISR to update compare values
//
int iflag = 0;
int ixi = 0;
int fcnt =0;

uint16_t tempInt;
float iq_temp;
float jointSpeedf;
float JSpeedlast;

//signed char str[80]={0};

__interrupt void epwm1_isr(void)
{



    int TempPIEIER;
    TempPIEIER = PieCtrlRegs.PIEIER6.all; // Save PIEIER register for later
     PieCtrlRegs.PIEIER6.bit.INTx3= 1;
     PieCtrlRegs.PIEACK.all = 0xFFFF;
      asm("       NOP");
      EINT;
//----------Start AbsENC----------//
//    if (posflag == 0)
//     {
    //   posflag++;

//     }
//     else if (posflag == 1)
//     {
      counts2=counts1;
      counts1 = CpuTimer0Regs.TIM.all;

             counts = counts2 -counts1;

      // bissc_configEPWM4();



                       ready = 0;
                      //uint16_t endatcf = 0x700;

//                           if (posflag == 0)
//                            {
//                              posflag++;
                              SPI_resetRxFIFO(SPIB_BASE);
                              SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_RX_OVERRUN |
                                                SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RXFF |
                                               SPI_INT_RXFF_OVERFLOW | SPI_INT_TXFF);
                              endat2_setupCommand ();
                              endat2_startOperation();



                      SPI_resetRxFIFO(SPIA_BASE);
                      SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RX_OVERRUN |
                                                SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RXFF |
                                                SPI_INT_RXFF_OVERFLOW | SPI_INT_TXFF);
                       endat1_setupCommand ();
                       endat1_startOperation();

//                            }
                      //DELAY_US(30);
                     // while(ready == 0);


       // DELAY_US(20);
//       // DELAY_US(10);
//        //while(dataready != 1){}
//       // dataready = 0;
        //CLB_disableCLB(CLB4_BASE);
//      SPI_resetRxFIFO(SPIB_BASE);
//      SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_RX_OVERRUN |
//                               SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RXFF |
//                               SPI_INT_RXFF_OVERFLOW | SPI_INT_TXFF);
//      endat_setupCommand ();
//      endat_startOperation();
//        posflag =03
//      }
        pFB  = ((v * 0.00068665)/360.0) * 2 * pi;
        jFB  = ((joint_position * 0.00034332308)/360)*2*pi;
        absjFB = (-jFB + (2*pi))+5.03185129 ;
        if(absjFB > (2*pi))   absjFB -=  2*pi;
        Load = sin(absjFB)*3.40;
        Loadd =cos(absjFB)*3.40;
         angle_single_change(pFB, jFB_last,&cyclenum);
         jFB_last = pFB;
         djFB = 2*pi* cyclenum + pFB;
        // pFBE =pFB * 10.0;
//        float tep = fst(epos,espeed,djFB,4000,0.0001);
//        epos = epos + (0.0001*espeed);
//        espeed = espeed + (0.0001*tep);
       // pFBnew = (0.9608*pFBold) + (0.0392*F_FB);
        pFBnew = (0.9704*pFBold) + (0.0296*pFB);
       float sder =pFB-pFBnew;
//       if( sder > pi)
//           {
//           sder -=6.26;
//          }
//        if(sder <-pi)
//           {
//           sder +=6.26;
//        }
       speedf = 300*(sder);
       pFBold = pFBnew;
//        if (fcnt == 5)
//            {
//
//            int o = 0;
//            float fmax = F[0];
//            float fmin = F[0];
//                            for ( o = 0; o < 5; o++)
//                           {
//                               if (fmax < F[o])
//                               {
//                                   fmax = F[o];
//
//                               }
//                               if (fmin > F[o])
//                                {
//                                    fmin = F[o];
//
//                                 }
//                           }
//            F_FB2 = (F[4]+F[3]+F[2] +F[1] +F[0]-fmax-fmin)*0.33333;
//            F[0] =0;
//            F[1]=0;
//            F[2]=0;
//            F[3]=0;
//            F[4]=0;
//            fcnt=0;
//            }
//        else
//        {
//         F[4] =F[3];
//         F[3] =F[2];
//         F[2] =F[1];
//         F[1] =F[0];
//         F[0] =speedf;
//         fcnt++;
//        }


//----------Speed Calculate---------//

      if( cnt ==9  )
      {
       cnt = 0;
      sCalculation1k();
      jsCalculation1k();
//      jointSpeedf =(0.01 * jointspeed1k) + (0.99*JSpeedlast);
//     // Theta_S = Theta_S + 0.8;
//       JSpeedlast = jointSpeedf;
      }
      else
      {
        cnt++;
      }


  //----------Software start ADsample---------//


              // 开始SOC1和2采样
               //counts1 = CpuTimer0Regs.TIM.all;
               //AdcbRegs.ADCSOCFRC1.all = 0xff;
              // AdccRegs.ADCSOCFRC1.all = 0xff;
      AdcaRegs.ADCSOCFRC1.all = 0x01ff;
      AdcbRegs.ADCSOCFRC1.all = 0x01ff;
      AdccRegs.ADCSOCFRC1.all = 0x01ff;
                    while( AdccRegs.ADCINTFLG.bit.ADCINT1 != 1)
                               {};
      //AdccRegs.ADCSOCFRC1.all = 0x01ff;
     // AdcdRegs.ADCSOCFRC1.all = 0x01ff;
//      while( AdcbRegs.ADCINTFLG.bit.ADCINT1 != 1)
//                 {};



               //CpuTimer0Regs.TCR.bit.TSS = 1;
               //counts2 =  CpuTimer0Regs.TIM.all;
               //counts = counts1 -counts2;

            //  Vdc_fb = AdcaResult0;
// AD - Tempture
              AdcaResult4 = AdcaResultRegs.ADCRESULT4 * 3.0/65535;
              AdcaResult5 = AdcaResultRegs.ADCRESULT5 * 3.0/65535;

// AD - force
               AdcbResult0 = AdcbResultRegs.ADCRESULT0 ;
               AdcbResult1 = AdcbResultRegs.ADCRESULT1 ;
               AdcbResult2 = AdcbResultRegs.ADCRESULT2 ;
               AdcbResult3 = AdcbResultRegs.ADCRESULT3 ;
               AdcbResult4 = AdcbResultRegs.ADCRESULT4 ;
               AdcbResult5 = AdcbResultRegs.ADCRESULT5 ;

               AdccResult0 = AdccResultRegs.ADCRESULT0 ;
               AdccResult1 = AdccResultRegs.ADCRESULT1 ;
               force3 = AdccResult0;
               force4 = AdccResult1;
//               ADCB = force3 -force4;
               ADCD = AdcbResult3;
//               F_FB1 = ((ADCB)/65536.0) * 3 * 39.1083;
               F_FB1 = ((ADCD-32440)/65536.0) * 3 * 39.1083;
              // F_FB2 = digital_lp_filter(1000, 0.0001, F_FB1);
              // F_FB2= (0.00036788 *F_FB2_LAST) + (0.99963212 * F_FB1);
              // F_FB2_LAST = F_FB2;
               if (ADcnt == 10)
               {
               float max = force1_Buffer[0];
               float min = force1_Buffer[0];
               int size = 10;
               int io = 0;
                for ( io = 0; io < size; io++)
               {
                   if (max < force1_Buffer[io])
                   {
                       max = force1_Buffer[io];

                   }
                   if (min > force1_Buffer[io])
                    {
                         min = force1_Buffer[io];

                     }
               }
              // force1 =((force1_Buffer[0]+force1_Buffer[1]+force1_Buffer[2]+force1_Buffer[3]+force1_Buffer[4]) - max - min)*0.3333;
              force1 =((force1_Buffer[0]+force1_Buffer[1]+force1_Buffer[2]+force1_Buffer[3]+force1_Buffer[4]+force1_Buffer[5]+force1_Buffer[6]+force1_Buffer[7]+force1_Buffer[8]+force1_Buffer[9]) - max - min)*0.125;
//               max = force2_Buffer[0];
//               min = force2_Buffer[0];
//               for ( io = 0; io < size; io++)
//                              {
//                                  if (max < force2_Buffer[io])
//                                  {
//                                      max = force2_Buffer[io];
//
//                                  }
//                                  if (min > force2_Buffer[io])
//                                   {
//                                        min = force2_Buffer[io];
//
//                                   }
//                              }
//
//               force2 =((force2_Buffer[0]+force2_Buffer[1]+force2_Buffer[2]+force2_Buffer[3]+force2_Buffer[4]) -max - min)*0.3333;
               ADCC = force1 - Fcoefficient ;
                F_FB = (ADCC/65536.0) * 3 * 39.1083;

                  F_FBd = (F_FB - F_FB_LAST)*1000.0;

//
//               float tempF = fst(eFB,eFBd,F_FB,50000,0.001);
//                     eFB = eFB+(0.001 * eFBd);
//                     eFBd = eFBd + (0.001 * tempF);
                    // F_FBd = eFBd;
                     FFBdf1 = (0.758*F_FBd) +(0.242*F_FBd_LAST);
                     FFBdf2 = digital_lp_filter(500, 0.001, F_FBd);
                     //FFBdf2 = 1000*(F_FB1 - F_FB2);
                     ff2 = -F_KD *FFBdf2;
               ADcnt = 0;
               F_FB_LAST = F_FB;
               F_FBd_LAST = FFBdf1;
               eFBd_LAST = eFBd;
               FFBdf2_LAST = FFBdf2;
               }
               else
               {

               force1_Buffer[ADcnt] =  AdcbResult3;

              // force2_Buffer[ADcnt] =  AdccResult1;
               ADcnt++;
               }

// Ia_fb
//
               AdcdResult0 = AdcaResultRegs.ADCRESULT0 ;
               AdcdResult1 = AdcaResultRegs.ADCRESULT1 ;
               AdcdResult2 = AdcaResultRegs.ADCRESULT2 ;
               AdcdResult3 = AdcdResultRegs.ADCRESULT3 ;
               AdcdResult4 = AdcdResultRegs.ADCRESULT4 ;
               AdcdResult5 = AdcdResultRegs.ADCRESULT5 ;
               AdcdResult6 = AdcdResultRegs.ADCRESULT6 ;
               AdcdResult7 = AdcdResultRegs.ADCRESULT7 ;
               AdcdResult8 = AdcdResultRegs.ADCRESULT8 ;
//               AdccResult0 = (uint16_t)(((uint32_t)AdccResult0 + (uint32_t)AdccResult1 + (uint32_t)AdccResult2)/3);
//               AdccResult1 = (uint16_t)(((uint32_t)AdccResult3 + (uint32_t)AdccResult4 + (uint32_t)AdccResult5)/3);
//               AdccResult2 = (uint16_t)(((uint32_t)AdccResult6 + (uint32_t)AdccResult7 + (uint32_t)AdccResult8)/3);
////
     if((( AdcdResult0 >=  AdcdResult1) &&(  AdcdResult0 <=  AdcdResult2)) ||(( AdcdResult0 <= AdcdResult1) && ( AdcdResult0 >=  AdcdResult2)))
            {
                 Ia =  AdcdResult0;
            }
     else if((( AdcdResult1>= AdcdResult0) &&(  AdcdResult1 <=  AdcdResult2)) || (( AdcdResult1<=  AdcdResult0) && ( AdcdResult1 >=  AdcdResult2)))
            {
                 Ia =  AdcdResult1;
            }
     else if((( AdcdResult2>=  AdcdResult0) &&(  AdcdResult2<=  AdcdResult1)) || (( AdcdResult2<=  AdcdResult0) && ( AdcdResult2>= AdcdResult1)))
            {
                 Ia =  AdcdResult2;
            }


//     Ib =(uint16_t)(((uint32_t)AdcdResult0 + (uint32_t)AdcdResult1 + (uint32_t)AdcdResult2)/3);
                if((( AdcdResult6 >=  AdcdResult7) &&(  AdcdResult6 <=  AdcdResult8)) ||(( AdcdResult6 <= AdcdResult7) && ( AdcdResult6 >=  AdcdResult8)))
             {
                 Ib =  AdcdResult6;
             }
     else if((( AdcdResult7>= AdcdResult6) &&( AdcdResult7 <=  AdcdResult8)) || (( AdcdResult7<=  AdcdResult6) && ( AdcdResult7 >=  AdcdResult8)))
             {
                 Ib =  AdcdResult7;
             }
      else if((( AdcdResult8>=  AdcdResult6) &&( AdcdResult8<=  AdcdResult7)) || (( AdcdResult8<=  AdcdResult6) && ( AdcdResult8>= AdcdResult7)))
             {
                 Ib =  AdcdResult8;
             }

//         if(iflag <=2)
//         {
         Ia_Buffer[2] =  Ia_Buffer[1];
         Ia_Buffer[1] =  Ia_Buffer[0];
         Ia_Buffer[0] =  Ia;

         Ib_Buffer[2] =  Ib_Buffer[1];
         Ib_Buffer[1] =  Ib_Buffer[0];
         Ib_Buffer[0] =  Ib;

         ia = (Ia_Buffer[0] + Ia_Buffer[1] + Ia_Buffer[2]) * 0.333333;
         ib = (Ib_Buffer[0] + Ib_Buffer[1] + Ib_Buffer[2]) * 0.333333;
////         if(ia < 30000)
////         {
////             ia =ia + 0.001;
////         }

//         Ia_Buffer[2] =  0;
//         Ia_Buffer[1] =  0;
//         Ia_Buffer[0] =  0;
//
//         Ib_Buffer[2] =  0;
//         Ib_Buffer[1] =  0;
//         Ib_Buffer[0] =  0;
//         }
//         ia = Ia;
//         ib = Ib;

         Is_a = Sensor_Coeff*(3 * (Ia - 32750.0)/65535.0);
         Is_b = Sensor_Coeff*(3 * (Ib - 32720.0)/65535.0);

        Position_Calculate();
        sCalculation10k();
        //speedf = digital_lp_filter(200, 0.001, speed1k);
        sFB = kalmanspeed;

         CLARK();
         PARK();
         iFB_d = Is_d;
         iFB_q = Is_q;
         iq_err = iREF_q - iFB_q ;
//         if(cogctl ==1)
//                      {
//                      t = t + t_interval;
//                      F_REF = ppp*sinf(2*pi*pp*t) ;
//                      pd =((ppp*2*pi*pp)*cosf(2*pi*pp*t));
//                      }

         float fFB  = F_FB;
         float FBdf = FFBdf1;

       if(fc == 1)
       {


           if (ctlcnt == 9 )

           {
//               fREF = pCONTROLLER(jREF,jFB,jointspeed1k,Load);
               iREF_q  = fCONTROLLER(fREF,fFB,FBdf,Loadd);
              // iREF_q  =
               iREF_d  = 0;
               ctlcnt = 0;
         }
           else
           {
               ctlcnt++;
               //iREF_q = pCONTROLLER();
               //iREF_d = 0;
       }
           }
//////////
   //   }


         //sCalculation1k();
       //speedwe = SPD_OBSERVER_run(&Speed1, pFB, iq_err ,0.0001, 0.1) * (-2.8127);
        Kalmanspeed();
        Speed = kalmanspeed;
 if(re == 1){
       float tt_interval = 0.0001;
       //参考电流(正弦）

       tt = tt + tt_interval ;
          fREF = ss*sinf(2*pi*ww*tt);
        // iCONTROLLER();
        // iREF_q = sCONTROLLER();
//           if( cnt ==9  )
//           {
//            cnt = 0;
//           //sCalculation10k();
//           Theta_S = Theta_S - 0.1;
//
//           }
//           else
//           {
//             cnt++;
         }
//
//
//     }
      Us_q = iCONTROLLER_q(iREF_q , iFB_q);
      Us_d = iCONTROLLER_d(iREF_d , iFB_d);
//    iREF_q   = fCONTROLLER();
   // float Uq;
Inverse_PARK();



//     erra = Time_C_a - last_timea;
//     last_timea = Time_C_a;
     SVPWM();
     //
//
     ePWMGen();

    // Clear INT flag for this timer
     AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;//等待采样完成
     AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
     AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
     AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    //
     EPwm1Regs.ETCLR.bit.INT = 1;



    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
     DINT;
     PieCtrlRegs.PIEIER6.all = TempPIEIER;
     PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;





}

void initPos(void)
{

    for (;;)
    {
        Us_q =   0;
        Us_d =   0;
        pos32val = 0;
      if(EQEP_getPositionLatch(EQEP3_BASE) == 0)
       {
          asm("       NOP");
       }

       if(EQEP_getIndexPositionLatch(EQEP3_BASE) != 0)
       {

           break;
       }
      }
}

int j =0;
//__interrupt void bissc_spibRxFifoIsr(void)
//{
//
//
//

//    uint16_t i = 0;
////
//
////    if (j != 2)
////    {
////
////           j++;
////    }
////    else if (j == 2)
////    {
////
////            CLB_disableCLB(CLB5_BASE);
////            initSPIASlave();
////            CLB_enableCLB(CLB5_BASE);
////        j =0;
////    }
//
//    for (i=0;i<=FIFO_LEVEL;i++)
//    {RxData[i]= SPI_readDataNonBlocking(SPIB_BASE); }
//    //CLB_disableCLB(CLB4_BASE);
//   recivePosition();
//    dataready = 1;
//    //DELAY_US(30L);
//    //Position_Calculate();
////    sCalculation();
//
//    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_RXFF_OVERFLOW);
//    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_RXFF);
//    SPI_resetRxFIFO(SPIB_BASE);
//
////    PieCtrlRegs.PIEIER6.all = TempPIEIER;
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);
//
//
//
//
////
//}
//
//int init = 0;
//__interrupt void bissc_spiaRxFifoIsr(void)
//{
//
//    uint16_t i = 0;
//    int j =0;
//
//    for (i=0;i<=FIFO_LEVEL;i++)
//    {RxData2[i]= SPI_readDataNonBlocking(SPIA_BASE); }
//    //CLB_disableCLB(CLB5_BASE);
//    reciveJointPosition();
//    jFB =(joint_position * 0.001373);
//    if (init == 0)
//        {
//         j_init = jFB;
//         jREF   = jFB;
//         init++;
//        }
//
//    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF_OVERFLOW);
//    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF);
//    SPI_resetRxFIFO(SPIA_BASE);
//
//  // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);
//
////
//}

//
float x_last = 0;
float p_last = 0.02;
float q = 0.001;
float r = 0.001;
float kg;
float x_mid;
float x_now;
float p_mid;
float p_now;
float z_real;
float z_measure;
void Kalmanspeed(void)
{
    x_last = z_real ;
    x_mid = (x_last + speed10k);
    p_mid = p_last + q;
    kg = p_mid / (p_mid + r);
    z_measure = 0;//测量值
    x_now = x_mid + kg*(z_measure - x_mid);//估计出的最有值
    p_now = (1 - kg)*p_mid;
    kalmanspeed = x_now;
    x_last = x_now;
    p_last = p_now;

}

__interrupt void endat1_spiRxFifoIsr(void)
{

    int TempPIEIER;
         TempPIEIER = PieCtrlRegs.PIEIER6.all; // Save PIEIER register for later
         //Interrupt_enable(INT_SPIA_RX);
         // IER |= 0x0020;                         // Set global priority by adjusting IER
         //IER &= 0x0020;
          PieCtrlRegs.PIEIER6.bit.INTx1= 1;
         // PieCtrlRegs.PIEIER6.bit.INTx1= 1; // Set group priority by adjusting PIEIER2 to allow INT2.2 to interrupt current ISR
          PieCtrlRegs.PIEACK.all = 0xFFFF;      // Enable PIE interrupts
           asm("       NOP");                    // Wait one cycle
           EINT;
    uint16_t i = 0;

//    RxData[0] = 0;
//    RxData[1] = 0;
//    RxData[2] = 0;

//    CLB_disableCLB(CLB1_BASE);
    // SPI_resetRxFIFO(SPIB_BASE);
    for (i=0;i<=FIFO_LEVEL;i++)
    {RxData[i]= SPI_readDataNonBlocking(SPIA_BASE); }


//    for (i=0;i<=FIFO_LEVEL;i++){RxData[i]= SPI_readDataBlockingFIFO(SPIB_BASE);}
      recivePosition();
    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF_OVERFLOW);
    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF);
//    SPI_resetTxFIFO(SPIB_BASE);
//    SPI_resetRxFIFO(SPIB_BASE);

   Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);      // Issue PIE ack    // Issue PIE ack
    ready = 1;
//    SPI_disableModule(SPIB_BASE);
   PieCtrlRegs.PIEIER6.all = TempPIEIER;
//    CLB_enableCLB(CLB1_BASE);
//
}

__interrupt void endat2_spiRxFifoIsr(void)
{
//    int TempPIEIER;
//             TempPIEIER = PieCtrlRegs.PIEIER6.all; // Save PIEIER register for later
//             //Interrupt_enable(INT_SPIA_RX);
//             // IER |= 0x0020;                         // Set global priority by adjusting IER
//             //IER &= 0x0020;
//              PieCtrlRegs.PIEIER6.bit.INTx1= 1;
//             // PieCtrlRegs.PIEIER6.bit.INTx1= 1; // Set group priority by adjusting PIEIER2 to allow INT2.2 to interrupt current ISR
//              PieCtrlRegs.PIEACK.all = 0xFFFF;      // Enable PIE interrupts
//               asm("       NOP");                    // Wait one cycle
//               EINT;
    uint16_t i = 0;
    int j =0;
//    RxData[0] = 0;
//    RxData[1] = 0;
//    RxData[2] = 0;

    // SPI_resetRxFIFO(SPIB_BASE);
    for (i=0;i<=FIFO_LEVEL;i++)
    {RxData2[i]= SPI_readDataNonBlocking(SPIB_BASE); }

    reciveJointPosition();
//    for (i=0;i<=FIFO_LEVEL;i++){RxData[i]= SPI_readDataBlockingFIFO(SPIB_BASE);}

    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_RXFF_OVERFLOW);
    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_RXFF);
//    SPI_resetTxFIFO(SPIB_BASE);
//    SPI_resetRxFIFO(SPIB_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);      // Issue PIE ack    // Issue PIE ack
    ready = 1;
//    PieCtrlRegs.PIEIER6.all = TempPIEIER;
//    SPI_disableModule(SPIB_BASE);

//
}








