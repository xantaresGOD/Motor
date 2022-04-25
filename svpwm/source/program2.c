
#include "f28x_project.h"
#include "driverlib.h"
#include "device.h"
#include "math.h"
//#include "PM_bissc_Include.h"
//#include "bissc.h"


#define EN_GPIO 121
#define PRD  5000      //
#define pi 3.1415926

#define FIFO_LEVEL     3
extern uint32_t TxData[FIFO_LEVEL] ;

void InitEPwm1Example()
{
   //
   // Setup TBCLK
   //
   EPwm1Regs.TBCTL.bit.CTRMODE =0x02; // Count up
   EPwm1Regs.TBPRD = PRD;       // Set timer period
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   //
   // Setup shadow register load on ZERO
   //
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   //
   // Set Compare values
   //
   EPwm1Regs.CMPA.bit.CMPA = 0;     // Set compare A value
   EPwm1Regs.CMPB.bit.CMPB = 0;     // Set Compare B value

   //
   // Set actions
   //
   //EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;            // Set PWM1A on Zero
   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; // Clear PWM1A on event A,
   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;                                        // up count

   //EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
   EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B,
   EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;
   //
   // Interrupt where we will change the Compare Values
   //
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;  // Generate INT on 3rd event


   EPwm1Regs.ETSEL.bit.SOCBEN  = 1;    //
   EPwm1Regs.ETSEL.bit.SOCBSEL = 3;   // Select SOC on up-count
   EPwm1Regs.ETPS.bit.SOCBPRD  = 1;

   //
   EPWM_setRisingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
   EPWM_setFallingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);

          EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_RED, true);
          EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_FED, true);
          EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
          EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
          EPWM_setRisingEdgeDelayCount(EPWM1_BASE, 80);
          EPWM_setFallingEdgeDelayCount(EPWM1_BASE, 80);
}

//
// InitEPwm2Example - Initialize EPWM2 values
//
void InitEPwm2Example()
{
   //
   // Setup TBCLK
   //
   EPwm2Regs.TBCTL.bit.CTRMODE = 0x02; // Count up
   EPwm2Regs.TBPRD = PRD;       // Set timer period
   EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   //
   // Setup shadow register load on ZERO
   //
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   //
   // Set Compare values
   //
   EPwm2Regs.CMPA.bit.CMPA = 0;      // Set compare A value
   EPwm2Regs.CMPB.bit.CMPB = 0;      // Set Compare B value

   //
   // Set actions
   //
   //EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;            // Set PWM1A on Zero
     EPwm2Regs.AQCTLA.bit.CAU = AQ_SET; // Clear PWM1A on event A,
     EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;                                        // up count

     //EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
     EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B,
     EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;
     //

   //
   // Interrupt where we will change the Compare Values
   //
   EPwm2Regs.ETSEL.bit.INTSEL =0x0;       // Select INT on Zero event
   EPwm2Regs.ETSEL.bit.INTEN = 0;                  // Enable INT
   EPwm2Regs.ETPS.bit.INTPRD = 0x0;             // Generate INT on 3rd event

   EPwm2Regs.ETSEL.bit.SOCBEN  = 1;    //
   EPwm2Regs.ETSEL.bit.SOCBSEL = 3;   // Select SOC on up-count
   EPwm2Regs.ETPS.bit.SOCBPRD  = 1;

          EPWM_setRisingEdgeDeadBandDelayInput(EPWM2_BASE,EPWM_DB_INPUT_EPWMA);
          EPWM_setFallingEdgeDeadBandDelayInput(EPWM2_BASE,EPWM_DB_INPUT_EPWMA);

          EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_RED, true);
          EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_FED, true);
          EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
          EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
          EPWM_setRisingEdgeDelayCount(EPWM2_BASE,  80);
          EPWM_setFallingEdgeDelayCount(EPWM2_BASE, 80);
}

//
// InitEPwm3Example - Initialize EPWM3 values
//
void InitEPwm3Example(void)
{
   //
    // Setup TBCLK
       //
       EPwm3Regs.TBCTL.bit.CTRMODE = 0x02; // Count up
       EPwm3Regs.TBPRD = PRD;       // Set timer period
       EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
       EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
       EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
       EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
       EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

       //
       // Setup shadow register load on ZERO

       EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
       EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
       EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
       EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

       //
       // Set Compare values
       //
       EPwm3Regs.CMPA.bit.CMPA = 0;      // Set compare A value
       EPwm3Regs.CMPB.bit.CMPB = 0;      // Set Compare B value

       //
       // Set actions
       //
       //EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;            // Set PWM1A on Zero
         EPwm3Regs.AQCTLA.bit.CAU = AQ_SET; // Clear PWM1A on event A,
         EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;                                        // up count

         //EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
         EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B,
         EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;
         //
         EPwm3Regs.ETSEL.bit.SOCBEN  = 1;    //
         EPwm3Regs.ETSEL.bit.SOCBSEL = 3;   // Select SOC on up-count
         EPwm3Regs.ETPS.bit.SOCBPRD  = 1;
       //
       // Interrupt where we will change the Compare Values
       //95
       EPwm3Regs.ETSEL.bit.INTSEL =0x0;       // Select INT on Zero event
       EPwm3Regs.ETSEL.bit.INTEN = 0;                  // Enable INT
       EPwm3Regs.ETPS.bit.INTPRD = 0x0;             // Generate INT on 3rd event

                EPWM_setRisingEdgeDeadBandDelayInput(EPWM3_BASE,EPWM_DB_INPUT_EPWMA);
                EPWM_setFallingEdgeDeadBandDelayInput(EPWM3_BASE,EPWM_DB_INPUT_EPWMA);

                EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_RED, true);
                EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, true);
                EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
                EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
                EPWM_setRisingEdgeDelayCount(EPWM3_BASE, 80);
                EPWM_setFallingEdgeDelayCount(EPWM3_BASE, 80);

}



void InitGPIOset(void)
{
       GPIO_SetupPinMux(EN_GPIO, GPIO_MUX_CPU1, 0);
       GPIO_SetupPinOptions(EN_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
       //GPIO_WritePin(EN_GPIO, 1);


       GPIO_SetupPinMux(121, GPIO_MUX_CPU1, 0);
       GPIO_SetupPinOptions(121, GPIO_OUTPUT, GPIO_PUSHPULL);
       GPIO_WritePin(121, 1);

       GPIO_SetupPinMux(70, GPIO_MUX_CPU1, 0);
       GPIO_SetupPinOptions(70, GPIO_INPUT, GPIO_PULLUP);

//       GPIO_SetupPinMux(149, GPIO_MUX_CPU1, 0);
//       GPIO_SetupPinOptions(149, GPIO_INPUT, GPIO_PULLUP );
////
//      GPIO_SetupPinMux(90, GPIO_MUX_CPU1, 0);
//      GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP );
//
//      GPIO_SetupPinMux(92, GPIO_MUX_CPU1, 0);
//     GPIO_SetupPinOptions(92, GPIO_INPUT, GPIO_PULLUP );

       EALLOW;


       GPIO_setPinConfig(GPIO_73_GPIO73);
       GPIO_setDirectionMode(73, GPIO_DIR_MODE_OUT);

       GPIO_setPinConfig(GPIO_38_GPIO38);
       GPIO_setDirectionMode(38, GPIO_DIR_MODE_IN);

       GPIO_setPinConfig(GPIO_6_GPIO6);
       GPIO_setDirectionMode(6, GPIO_DIR_MODE_IN);
       GPIO_setPadConfig(6,GPIO_PIN_TYPE_INVERT );

       GPIO_setPinConfig(GPIO_90_GPIO90);
       GPIO_setDirectionMode(90, GPIO_DIR_MODE_IN);
       GPIO_setPadConfig(90, GPIO_PIN_TYPE_INVERT );

       GPIO_setPinConfig(GPIO_92_GPIO92);
       GPIO_setDirectionMode(92, GPIO_DIR_MODE_IN);
       GPIO_setPadConfig(92, GPIO_PIN_TYPE_INVERT );

//       GPIO_setPinConfig(GPIO_70_GPIO70);
//       GPIO_setDirectionMode(70, GPIO_DIR_MODE_IN);
//       GPIO_setPadConfig(92, GPIO_PIN_TYPE_INVERT );

       GPIO_setPinConfig(GPIO_104_EQEP3_A);
       GPIO_setPadConfig(104, GPIO_PIN_TYPE_STD);

       GPIO_setPinConfig(GPIO_105_EQEP3_B);
       GPIO_setPadConfig(105, GPIO_PIN_TYPE_STD);

       GPIO_setPinConfig(GPIO_107_EQEP3_INDEX);
       GPIO_setPadConfig(107, GPIO_PIN_TYPE_STD);
       EDIS;
}

void ConfigureADC(void)
{
    EALLOW;

    //
    // Write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 2; //set ADCCLK divider to /1
    AdcbRegs.ADCCTL2.bit.PRESCALE = 2; //set ADCCLK divider to /1
    AdccRegs.ADCCTL2.bit.PRESCALE = 2; //set ADCCLK divider to /1
    AdcdRegs.ADCCTL2.bit.PRESCALE = 4; //set ADCCLK divider to /1
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_16BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_16BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_16BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_16BIT, ADC_SIGNALMODE_SINGLE);

    //
    // Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    // Power up the ADCs
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    // Delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

void SetupADCSoftware(void)
{
    Uint16 acqps;

    //
    // Determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //300ns
    }

    //
    // Select the channels to convert and end of conversion flag
    // ADCA
    //
    EALLOW;

    //InitTempSensor(3.0);
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 6;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps +
                                //1 SYSCLK cycles

    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 6;
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 4;  //SOC1 will convert pin A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps +
     //1 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 6;
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 5;  //SOC0 will convert pin A4
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycle
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;  //SOC0 will convert pin A5
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps +
                                              //1 SYSCLK cycles
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;  //SOC0 will convert pin A5
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps; //sample window is acqps +
                                                 //1 SYSCLK cycles
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 5;  //SOC0 will convert pin A5
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps; //sample window is acqps +
                                                    //1 SYSCLK cycles

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //end of SOC1 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC1 will convert pin B1
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;  //SOC1 will convert pin B2
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;  //SOC1 will convert pin B3
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcbRegs.ADCSOC4CTL.bit.CHSEL = 4;  //SOC1 will convert pin B4
    AdcbRegs.ADCSOC4CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcbRegs.ADCSOC5CTL.bit.CHSEL = 5;  //SOC1 will convert pin B5
    AdcbRegs.ADCSOC5CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 5; //end of SOC1 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCC
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin c2
    AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps +
                                              //1 SYSCLK cycles

    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin c3
    AdccRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps +
                                         //1 SYSCLK cycles

    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 2;  //SOC1 will convert pin c3
    AdccRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is acqps +
                                                  //1 SYSCLK cycles
//
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC3CTL.bit.CHSEL =3;  //SOC0 will convert pin c2
    AdccRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps +
                                            //1 SYSCLK cycles

    AdccRegs.ADCSOC4CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC4CTL.bit.CHSEL = 3;  //SOC1 will convert pin c3
    AdccRegs.ADCSOC4CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles

    AdccRegs.ADCSOC5CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC5CTL.bit.CHSEL = 4;  //SOC1 will convert pin c3
    AdccRegs.ADCSOC5CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles


//    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 7; //end of SOC1 will set INT1 flag
//    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
  EDIS;
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);


///////////////////////////////////////////////////////////////////////////////////

  EALLOW;

    //ADCD
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0;
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin B
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps +
       //1 SYSCLK cycles
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0;
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC1 will convert pin B3
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps +
      //1 SYSCLK cycles
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 0;
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 1;  //SOC1 will convert pin B3
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is acqps +

    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 6;
    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 2;  //SOC0 will convert pin B
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps +
       //1 SYSCLK cycles
    AdcdRegs.ADCSOC4CTL.bit.TRIGSEL = 6;
    AdcdRegs.ADCSOC4CTL.bit.CHSEL = 2;  //SOC1 will convert pin B3
    AdcdRegs.ADCSOC4CTL.bit.ACQPS = acqps; //sample window is acqps +
      //1 SYSCLK cycles
    AdcdRegs.ADCSOC5CTL.bit.TRIGSEL = 6;
    AdcdRegs.ADCSOC5CTL.bit.CHSEL = 2;  //SOC1 will convert pin B3
    AdcdRegs.ADCSOC5CTL.bit.ACQPS = acqps; //sample window is acqps +

    AdcdRegs.ADCSOC6CTL.bit.TRIGSEL = 6;
    AdcdRegs.ADCSOC6CTL.bit.CHSEL = 3;  //SOC0 will convert pin B
    AdcdRegs.ADCSOC6CTL.bit.ACQPS = acqps; //sample window is acqps +
       //1 SYSCLK cycles
    AdcdRegs.ADCSOC7CTL.bit.TRIGSEL = 6;
    AdcdRegs.ADCSOC7CTL.bit.CHSEL = 4;  //SOC1 will convert pin B3
    AdcdRegs.ADCSOC7CTL.bit.ACQPS = acqps; //sample window is acqps +
      //1 SYSCLK cycles
    AdcdRegs.ADCSOC8CTL.bit.TRIGSEL = 6;
    AdcdRegs.ADCSOC8CTL.bit.CHSEL = 5;  //SOC1 will convert pin B3
    AdcdRegs.ADCSOC8CTL.bit.ACQPS = acqps; //sample window is acqps +
     EDIS;                                                //1 SYSCLK cycles
       ADC_setInterruptSource(ADCD_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER8);
       ADC_enableInterrupt(ADCD_BASE, ADC_INT_NUMBER1);
       ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);



}


//void bissc_setupGPIO(void)
//{
//
//    EALLOW;
//
//
//         GPIO_setMasterCore(16, GPIO_CORE_CPU1);
//         GPIO_setPinConfig(GPIO_16_SPIA_SIMO);
//         GPIO_setPadConfig(16, GPIO_PIN_TYPE_PULLUP);
//         GPIO_setQualificationMode(16, GPIO_QUAL_ASYNC);
//
//           //
//           // GPIO25 is the SPISOMIB.
//           //
//         GPIO_setMasterCore(17, GPIO_CORE_CPU1);
//         GPIO_setPinConfig(GPIO_17_SPIA_SOMI);
//         GPIO_setPadConfig(17, GPIO_PIN_TYPE_PULLUP);
//         GPIO_setQualificationMode(17, GPIO_QUAL_ASYNC);
//
//           //
//           // GPIO26 is the SPICLKB.
//           //
//         GPIO_setMasterCore(18, GPIO_CORE_CPU1);
//         GPIO_setPinConfig(GPIO_18_SPIA_CLK);
//         GPIO_setPadConfig(18, GPIO_PIN_TYPE_PULLUP);
//         GPIO_setQualificationMode(18, GPIO_QUAL_ASYNC);
//
//           //
//           // GPIO27 is the SPISTEB.
//           //
//         GPIO_setMasterCore(19, GPIO_CORE_CPU1);
//         GPIO_setPinConfig(GPIO_19_SPIA_STEN);
//         GPIO_setPadConfig(19, GPIO_PIN_TYPE_PULLUP);
//         GPIO_setQualificationMode(19, GPIO_QUAL_ASYNC);
//
////     SPIB
//         GPIO_setMasterCore(24, GPIO_CORE_CPU1);
//         GPIO_setPinConfig(GPIO_24_SPIB_SIMO);
//         GPIO_setPadConfig(24, GPIO_PIN_TYPE_PULLUP);
//         GPIO_setQualificationMode(24, GPIO_QUAL_ASYNC);
//
//         GPIO_setMasterCore(25, GPIO_CORE_CPU1);
//         GPIO_setPinConfig(GPIO_25_SPIB_SOMI);
//         GPIO_setPadConfig(25, GPIO_PIN_TYPE_PULLUP);
//         GPIO_setQualificationMode(25, GPIO_QUAL_ASYNC);
//
//         GPIO_setMasterCore(26, GPIO_CORE_CPU1);
//         GPIO_setPinConfig(GPIO_26_SPIB_CLK);
//         GPIO_setPadConfig(26, GPIO_PIN_TYPE_PULLUP);
//         GPIO_setQualificationMode(26, GPIO_QUAL_ASYNC);
//
//         GPIO_setMasterCore(27, GPIO_CORE_CPU1);
//         GPIO_setPinConfig(GPIO_27_SPIB_STEN);
//         GPIO_setPadConfig(27, GPIO_PIN_TYPE_PULLUP);
//         GPIO_setQualificationMode(27, GPIO_QUAL_ASYNC);
//
//  //   DevCfgRegs.CPUSEL6.bit.SPI_B = 1;
//  //    DevCfgRegs.CPUSEL15.bit.CLB4 = 1;
//
//    EDIS;
//}
//
//
//void bissc_configXBAR(void)
//{
//    EALLOW;
////*************************************************************************************//
//      GPIO_setPinConfig(GPIO_8_EPWM5A);
//      GPIO_setDirectionMode(8,GPIO_DIR_MODE_OUT);
//      GPIO_setPadConfig(8,GPIO_PIN_TYPE_STD);
//
//      GPIO_setPinConfig(GPIO_9_EPWM5B);
//      GPIO_setDirectionMode(9,GPIO_DIR_MODE_OUT);
//      GPIO_setPadConfig(9,GPIO_PIN_TYPE_STD);
////*************************************************************************************//
//      GPIO_setPinConfig(GPIO_151_EPWM4A);
//      GPIO_setDirectionMode(151,GPIO_DIR_MODE_OUT);
//      GPIO_setPadConfig(151,GPIO_PIN_TYPE_STD);
//
//
//      GPIO_setPinConfig(GPIO_152_EPWM4B);
//      GPIO_setDirectionMode(152,GPIO_DIR_MODE_OUT);
//      GPIO_setPadConfig(152,GPIO_PIN_TYPE_STD);
////*************************************************************************************//
//      CLB_setOutputMask(CLB4_BASE, CLB_OUTPUT_00, true);
//      CLB_setOutputMask(CLB4_BASE, CLB_OUTPUT_02, true);
//
//    //Choose SPISIMO GPIO as INPUTXBAR Input1 - SPISIMOB in this case
//    XBAR_setInputPin(INPUTXBAR_BASE,XBAR_INPUT1,24);// GPTRIP XBAR TRIP1 -> GPIO63
//   // XBAR_setInputPin(XBAR_INPUT2,0);
//    XBAR_setCLBMuxConfig(XBAR_AUXSIG0,XBAR_CLB_MUX01_INPUTXBAR1);
//    XBAR_enableCLBMux(XBAR_AUXSIG0,XBAR_MUX01);
//
//    CLB_configLocalInputMux(CLB4_BASE,CLB_IN0,CLB_LOCAL_IN_MUX_GLOBAL_IN);
//    CLB_configGlobalInputMux(CLB4_BASE,CLB_IN0,CLB_GLOBAL_IN_MUX_CLB_AUXSIG0);
//    CLB_configGPInputMux(CLB4_BASE,CLB_IN0,CLB_GP_IN_MUX_EXTERNAL);
//   // SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL6_SPI, 1, SYSCTL_CPUSEL_CPU2);
////*************************************************************************************//
//    CLB_setOutputMask(CLB5_BASE, CLB_OUTPUT_00, true);
//    CLB_setOutputMask(CLB5_BASE, CLB_OUTPUT_02, true);
//
//
//  XBAR_setInputPin(INPUTXBAR_BASE,XBAR_INPUT2,16);
// // XBAR_setInputPin(XBAR_INPUT2,0);
//  XBAR_setCLBMuxConfig(XBAR_AUXSIG1, XBAR_CLB_MUX03_INPUTXBAR2);
//  XBAR_enableCLBMux(XBAR_AUXSIG1, XBAR_MUX03);
////
//  CLB_configLocalInputMux(CLB5_BASE,CLB_IN1,CLB_LOCAL_IN_MUX_GLOBAL_IN);
//  CLB_configGlobalInputMux(CLB5_BASE,CLB_IN1,CLB_GLOBAL_IN_MUX_CLB_AUXSIG1);
//  CLB_configGPInputMux(CLB5_BASE,CLB_IN1,CLB_GP_IN_MUX_EXTERNAL);
////*************************************************************************************//
//    EDIS;
//}
//
//
//void initSPIBSlave(void)
//{
//    //
//    // Must put SPI into reset before configuring it
//    //
//    SPI_disableModule(SPIB_BASE);
//
//    //
//    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size.
//    //
//    SPI_setConfig(SPIB_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA1,
//                  SPI_MODE_SLAVE, 5000000, 16);
//   // SPI_disableLoopback(SPIB_BASE);
//    SPI_setEmulationMode(SPIB_BASE, SPI_EMULATION_FREE_RUN);
//
//    //
//    // FIFO and interrupt configuration
//    //
//    SPI_enableFIFO(SPIB_BASE);
//    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_RXFF);
//    SPI_setFIFOInterruptLevel(SPIB_BASE, SPI_FIFO_TX2, SPI_FIFO_RX3);
//    SPI_enableInterrupt(SPIB_BASE, SPI_INT_RXFF);
//
//    //
//    // Configuration complete. Enable the module.
//    //
//    SPI_enableModule(SPIB_BASE);
//
//}
//
////void initSPIASlave(void)
////{
////    //
////    // Must put SPI into reset before configuring it
////    //
////    SPI_disableModule(SPIA_BASE);
////
////    //
////    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size.
////    //
////    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA1,
////                  SPI_MODE_SLAVE, 5000000, 16);
////   // SPI_disableLoopback(SPIB_BASE);
////    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_FREE_RUN);
////
////    //
////    // FIFO and interrupt configuration
////    //
////    SPI_enableFIFO(SPIA_BASE);
////    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF);
////    SPI_setFIFOInterruptLevel(SPIA_BASE, SPI_FIFO_TX2, SPI_FIFO_RX3);
////    SPI_enableInterrupt(SPIA_BASE, SPI_INT_RXFF);
////
////    //
////    // Configuration complete. Enable the module.
////    //
////    SPI_enableModule(SPIA_BASE);
////
////}

void scic_init()
{
    //
    // GPIO28 is the SCI Rx pin.
    //
    GPIO_setMasterCore(38, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_38_SCIC_TX);
    GPIO_setDirectionMode(38, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(38, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(38, GPIO_QUAL_ASYNC);

    //
    // GPIO29 is the SCI Tx pin.
    //
    GPIO_setMasterCore(39, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_39_SCIC_RX);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(39, GPIO_QUAL_ASYNC);

    // in the InitSysCtrl() function
    //
    SCI_performSoftwareReset(SCIC_BASE);

    //
    // Configure SCIA for echoback.
    //
    SCI_setConfig(SCIC_BASE, DEVICE_LSPCLK_FREQ,500000, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIC_BASE);
    SCI_resetRxFIFO(SCIC_BASE);
    SCI_resetTxFIFO(SCIC_BASE);
    SCI_clearInterruptStatus(SCIC_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableFIFO(SCIC_BASE);
    SCI_enableModule(SCIC_BASE);
    SCI_performSoftwareReset(SCIC_BASE);
}

void endat2_setupGPIO(void) {

    //
    // GPIO7 is SPI Clk slave
    GPIO_setPinConfig(GPIO_151_EPWM4A);
    GPIO_setDirectionMode(151,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(151,GPIO_PIN_TYPE_STD);


    GPIO_setPinConfig(GPIO_152_EPWM4B);
    GPIO_setDirectionMode(152,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(152,GPIO_PIN_TYPE_STD);

    //
    // GPIO63 is the SPISIMOB
    //
    GPIO_setMasterCore(24, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_24_SPIB_SIMO);
    GPIO_setQualificationMode(24, GPIO_QUAL_ASYNC);

    //
    // GPIO64 is the SPISOMIB
    //
    GPIO_setMasterCore(25, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_25_SPIB_SOMI);
    GPIO_setQualificationMode(25, GPIO_QUAL_ASYNC);

    //
    // GPIO65 is the SPICLKB
    //
    GPIO_setMasterCore(26, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_26_SPIB_CLK);
    GPIO_setQualificationMode(26, GPIO_QUAL_ASYNC);

    //
    // GPIO66 is the SPISTEB
    //
    GPIO_setMasterCore(27, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_27_SPIB_STEN);
    GPIO_setQualificationMode(27, GPIO_QUAL_ASYNC);

    //
    // GPIO9 is tformat TxEN
    //
    GPIO_setMasterCore(14, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_14_OUTPUTXBAR3);

    //
    // GPIO139 is PwrEN
    //
    GPIO_setMasterCore(139, GPIO_CORE_CPU1);
    GPIO_setDirectionMode(139, GPIO_DIR_MODE_OUT);
}

void endat2_configXBAR(void)
{
    //
    // Connect InputXbar-INPUT1 to GPIO63 - SPISIMO
       CLB_configLocalInputMux(CLB4_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB4_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB4_BASE, CLB_IN2, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB4_BASE, CLB_IN3, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB4_BASE, CLB_IN4, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB4_BASE, CLB_IN5, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB4_BASE, CLB_IN6, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB4_BASE, CLB_IN7, CLB_LOCAL_IN_MUX_GLOBAL_IN);

       CLB_configGlobalInputMux(CLB4_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB4_BASE, CLB_IN1,
                                CLB_GLOBAL_IN_MUX_CLB_AUXSIG0);
       CLB_configGlobalInputMux(CLB4_BASE, CLB_IN2, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB4_BASE, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB4_BASE, CLB_IN4, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB4_BASE, CLB_IN5, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB4_BASE, CLB_IN6, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB4_BASE, CLB_IN7, CLB_GLOBAL_IN_MUX_EPWM1A);

       CLB_configGPInputMux(CLB4_BASE, CLB_IN0, CLB_GP_IN_MUX_GP_REG);
       CLB_configGPInputMux(CLB4_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
       CLB_configGPInputMux(CLB4_BASE, CLB_IN2, CLB_GP_IN_MUX_EXTERNAL);
       CLB_configGPInputMux(CLB4_BASE, CLB_IN3, CLB_GP_IN_MUX_EXTERNAL);
       CLB_configGPInputMux(CLB4_BASE, CLB_IN4, CLB_GP_IN_MUX_EXTERNAL);
       CLB_configGPInputMux(CLB4_BASE, CLB_IN5, CLB_GP_IN_MUX_EXTERNAL);
       CLB_configGPInputMux(CLB4_BASE, CLB_IN6, CLB_GP_IN_MUX_GP_REG);
       CLB_configGPInputMux(CLB4_BASE, CLB_IN7, CLB_GP_IN_MUX_GP_REG);

       CLB_enableSynchronization(CLB4_BASE, CLB_IN0);
       CLB_enableSynchronization(CLB4_BASE, CLB_IN1);

       CLB_selectInputFilter(CLB4_BASE, CLB_IN0, CLB_FILTER_RISING_EDGE);
       CLB_selectInputFilter(CLB4_BASE, CLB_IN1, CLB_FILTER_RISING_EDGE);

    XBAR_setInputPin(INPUTXBAR_BASE,XBAR_INPUT1, 24);

    XBAR_setCLBMuxConfig(XBAR_AUXSIG0, XBAR_CLB_MUX01_INPUTXBAR1);
    XBAR_enableCLBMux(XBAR_AUXSIG0, XBAR_MUX01);
    XBAR_setOutputMuxConfig(OUTPUTXBAR_BASE,XBAR_OUTPUT3,  XBAR_OUT_MUX13_CLB4_OUT4);
    XBAR_enableOutputMux(OUTPUTXBAR_BASE,XBAR_OUTPUT3, XBAR_MUX13);
    //    XBAR_setCLBMuxConfig(XBAR_AUXSIG0,XBAR_CLB_MUX01_INPUTXBAR1);
    //    XBAR_enableCLBMux(XBAR_AUXSIG0,XBAR_MUX01);
//         CLB_setOutputMask(CLB4_BASE, CLB_OUTPUT_00, true);
//         CLB_setOutputMask(CLB4_BASE, CLB_OUTPUT_02, true);
//         CLB_setOutputMask(CLB4_BASE, CLB_OUTPUT_04, true);

}


void endat2_initSPIFIFO(void) {


//
// Initialize SPI FIFO registers
//
    SPI_disableModule(SPIB_BASE);
    SPI_disableLoopback(SPIB_BASE);

    SPI_setConfig(SPIB_BASE, DEVICE_LSPCLK_FREQ , SPI_PROT_POL1PHA0,
                SPI_MODE_SLAVE, 2000000, 16);

    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_RX_OVERRUN |
                             SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RXFF |
                             SPI_INT_RXFF_OVERFLOW | SPI_INT_TXFF);
    SPI_enableFIFO(SPIB_BASE);
    SPI_setFIFOInterruptLevel(SPIB_BASE, SPI_FIFO_TX3, SPI_FIFO_RX3);
    SPI_setEmulationMode(SPIB_BASE, SPI_EMULATION_FREE_RUN);
    SPI_enableModule(SPIB_BASE);
    SPI_resetTxFIFO(SPIB_BASE);
    SPI_resetRxFIFO(SPIB_BASE);

//    SPI_enableInterrupt(SPIB_BASE, SPI_INT_RX_OVERRUN |
//                        SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RXFF
//                        | SPI_INT_RXFF_OVERFLOW);
}

void endat2_init(void) {

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);

    //
    //Configure EPWM4 to drive default values on GPIO7
    //
    endat2_configEPWM4();

    //GPIO configuration for tformat operation
    //
    endat2_setupGPIO();
 //
    //XBAR configuration for tformat operation
    //
    endat2_configXBAR();

    endat2_initSPIFIFO();

    //
    // Must put SPI into reset before configuring it
    //
    SPI_disableModule(SPIB_BASE);
    SPI_disableInterrupt(SPIB_BASE, SPI_INT_RXFF);

    //
    // FIFO and interrupt configuration
    //
    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_RXFF);
    SPI_enableInterrupt(SPIB_BASE, SPI_INT_RXFF);

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(SPIB_BASE);

    //
    // Power up tformat 5v supply through GPIO139
    //

}

void endat2_startOperation(void)
{
    EALLOW;
    HWREG(CLB4_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) |= CLB_LOAD_EN_GLOBAL_EN
            | CLB_LOAD_EN_STOP;

    __asm(" RPT #10 || NOP");
    CLB_setOutputMask(CLB4_BASE, 0xF, true);
    __asm(" RPT #10 || NOP");
    CLB_setGPREG(CLB4_BASE, 0x83);

    EDIS;
}

void endat2_configEPWM4(void) {

    //
    // Set the PWMA and B high as default values of tformat clk.
    // Action on TZ1
    //
    EPWM_setTripZoneAction(EPWM4_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_HIGH);


    //
    // Action on TZ1
    //
    EPWM_setTripZoneAction(EPWM4_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_HIGH);

    //
    // Forces a Trip Zone event
    //
    EPWM_forceTripZoneEvent(EPWM4_BASE, EPWM_TZ_FORCE_EVENT_OST);
}




void endat2_setupCommand(void)
{
    SPI_resetRxFIFO(SPIB_BASE);
    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_RX_OVERRUN |
                             SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RXFF |
                             SPI_INT_RXFF_OVERFLOW | SPI_INT_TXFF);
    endat2_resetCLB();
    TxData[0] = 0x00000700;
    TxData[1] = 0x00000000;
    TxData[2] = 0x00000000;
    int i;
    for (i = 0;i < FIFO_LEVEL;i++)
        {
            SPI_writeDataNonBlocking(SPIB_BASE,TxData[i]);
        }
}

void endat2_resetCLB(void)
{

    CLB_setGPREG(CLB4_BASE, 0);

    //
    // Turn OFF the CLB functionality
    //
    EALLOW;
    HWREG(CLB4_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) = 0;
    EDIS;



    CLB_setOutputMask(CLB4_BASE, 0, false);

}

//


//endat 2.2 2

//
// scia_xmit - Transmit a character from the SCI
//
void endat1_setupGPIO(void) {

    //
    // GPIO7 is SPI Clk slave
    GPIO_setPinConfig(GPIO_8_EPWM5A);
    GPIO_setDirectionMode(8,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(8,GPIO_PIN_TYPE_STD);


    GPIO_setPinConfig(GPIO_9_EPWM5B);
    GPIO_setDirectionMode(9,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9,GPIO_PIN_TYPE_STD);

    //
    // GPIO63 is the SPISIMOB
    //
    GPIO_setMasterCore(16, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_16_SPIA_SIMO);
    GPIO_setQualificationMode(16, GPIO_QUAL_ASYNC);

    //
    // GPIO64 is the SPISOMIB
    //
    GPIO_setMasterCore(17, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_17_SPIA_SOMI);
    GPIO_setQualificationMode(17, GPIO_QUAL_ASYNC);

    //
    // GPIO65 is the SPICLKB
    //
    GPIO_setMasterCore(18, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_18_SPIA_CLK);
    GPIO_setQualificationMode(18, GPIO_QUAL_ASYNC);

    //
    // GPIO66 is the SPISTEB
    //
    GPIO_setMasterCore(19, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_19_SPIA_STEN);
    GPIO_setQualificationMode(19, GPIO_QUAL_ASYNC);

    //
    // GPIO9 is tformat TxEN
    //
    GPIO_setMasterCore(15, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_15_OUTPUTXBAR4);

    //
    // GPIO139 is PwrEN
    //
    GPIO_setMasterCore(139, GPIO_CORE_CPU1);
    GPIO_setDirectionMode(139, GPIO_DIR_MODE_OUT);
}

void endat1_configXBAR(void)
{
    //
    // Connect InputXbar-INPUT1 to GPIO63 - SPISIMO
       CLB_configLocalInputMux(CLB5_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB5_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB5_BASE, CLB_IN2, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB5_BASE, CLB_IN3, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB5_BASE, CLB_IN4, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB5_BASE, CLB_IN5, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB5_BASE, CLB_IN6, CLB_LOCAL_IN_MUX_GLOBAL_IN);
       CLB_configLocalInputMux(CLB5_BASE, CLB_IN7, CLB_LOCAL_IN_MUX_GLOBAL_IN);

       CLB_configGlobalInputMux(CLB5_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB5_BASE, CLB_IN1,
                                CLB_GLOBAL_IN_MUX_CLB_AUXSIG0);
       CLB_configGlobalInputMux(CLB5_BASE, CLB_IN2, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB5_BASE, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB5_BASE, CLB_IN4, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB5_BASE, CLB_IN5, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB5_BASE, CLB_IN6, CLB_GLOBAL_IN_MUX_EPWM1A);
       CLB_configGlobalInputMux(CLB5_BASE, CLB_IN7, CLB_GLOBAL_IN_MUX_EPWM1A);

       CLB_configGPInputMux(CLB5_BASE, CLB_IN0, CLB_GP_IN_MUX_GP_REG);
       CLB_configGPInputMux(CLB5_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
       CLB_configGPInputMux(CLB5_BASE, CLB_IN2, CLB_GP_IN_MUX_EXTERNAL);
       CLB_configGPInputMux(CLB5_BASE, CLB_IN3, CLB_GP_IN_MUX_EXTERNAL);
       CLB_configGPInputMux(CLB5_BASE, CLB_IN4, CLB_GP_IN_MUX_EXTERNAL);
       CLB_configGPInputMux(CLB5_BASE, CLB_IN5, CLB_GP_IN_MUX_EXTERNAL);
       CLB_configGPInputMux(CLB5_BASE, CLB_IN6, CLB_GP_IN_MUX_GP_REG);
       CLB_configGPInputMux(CLB5_BASE, CLB_IN7, CLB_GP_IN_MUX_GP_REG);

       CLB_enableSynchronization(CLB5_BASE, CLB_IN0);
       CLB_enableSynchronization(CLB5_BASE, CLB_IN1);

       CLB_selectInputFilter(CLB5_BASE, CLB_IN0, CLB_FILTER_RISING_EDGE);
       CLB_selectInputFilter(CLB5_BASE, CLB_IN1, CLB_FILTER_RISING_EDGE);

    XBAR_setInputPin(INPUTXBAR_BASE,XBAR_INPUT1, 16);

    XBAR_setCLBMuxConfig(XBAR_AUXSIG0, XBAR_CLB_MUX01_INPUTXBAR1);
    XBAR_enableCLBMux(XBAR_AUXSIG0, XBAR_MUX01);
    XBAR_setOutputMuxConfig(OUTPUTXBAR_BASE,XBAR_OUTPUT4,  XBAR_OUT_MUX17_CLB5_OUT4);
    XBAR_enableOutputMux(OUTPUTXBAR_BASE,XBAR_OUTPUT4, XBAR_MUX17);
    //    XBAR_setCLBMuxConfig(XBAR_AUXSIG0,XBAR_CLB_MUX01_INPUTXBAR1);
    //    XBAR_enableCLBMux(XBAR_AUXSIG0,XBAR_MUX01);
//         CLB_setOutputMask(CLB4_BASE, CLB_OUTPUT_00, true);
//         CLB_setOutputMask(CLB4_BASE, CLB_OUTPUT_02, true);
//         CLB_setOutputMask(CLB4_BASE, CLB_OUTPUT_04, true);

}


void endat1_initSPIFIFO(void) {


//
// Initialize SPI FIFO registers
//
    SPI_disableModule(SPIA_BASE);
    SPI_disableLoopback(SPIA_BASE);

    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ , SPI_PROT_POL1PHA0,
                SPI_MODE_SLAVE, 10000000, 16);

    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RX_OVERRUN |
                             SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RXFF |
                             SPI_INT_RXFF_OVERFLOW | SPI_INT_TXFF);
    SPI_enableFIFO(SPIA_BASE);
    SPI_setFIFOInterruptLevel(SPIA_BASE, SPI_FIFO_TX3, SPI_FIFO_RX3);
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_FREE_RUN);
    SPI_enableModule(SPIA_BASE);
    SPI_resetTxFIFO(SPIA_BASE);
    SPI_resetRxFIFO(SPIA_BASE);

    SPI_enableInterrupt(SPIA_BASE, SPI_INT_RX_OVERRUN |
                        SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RXFF
                        | SPI_INT_RXFF_OVERFLOW);
}

void endat1_init(void) {

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);

    //
    //Configure EPWM4 to drive default values on GPIO7
    //
    endat1_configEPWM5();

    //GPIO configuration for tformat operation
    //
    endat1_setupGPIO();
 //
    //XBAR configuration for tformat operation
    //
    endat1_configXBAR();

    endat1_initSPIFIFO();

    //
    // Must put SPI into reset before configuring it
    //
    SPI_disableModule(SPIA_BASE);
    SPI_disableInterrupt(SPIA_BASE, SPI_INT_RXFF);

    //
    // FIFO and interrupt configuration
    //
    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF);
    SPI_enableInterrupt(SPIA_BASE, SPI_INT_RXFF);

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(SPIA_BASE);

    //
    // Power up tformat 5v supply through GPIO139
    //

}

void endat1_startOperation(void)
{
    EALLOW;
    HWREG(CLB5_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) |= CLB_LOAD_EN_GLOBAL_EN
            | CLB_LOAD_EN_STOP;

    __asm(" RPT #10 || NOP");
    CLB_setOutputMask(CLB5_BASE, 0xF, true);
    __asm(" RPT #10 || NOP");
    CLB_setGPREG(CLB5_BASE, 0x83);

    EDIS;
}

void endat1_configEPWM5(void) {

    //
    // Set the PWMA and B high as default values of tformat clk.
    // Action on TZ1
    //
    EPWM_setTripZoneAction(EPWM5_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_HIGH);


    //
    // Action on TZ1
    //
    EPWM_setTripZoneAction(EPWM5_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_HIGH);

    //
    // Forces a Trip Zone event
    //
    EPWM_forceTripZoneEvent(EPWM5_BASE, EPWM_TZ_FORCE_EVENT_OST);
}




void endat1_setupCommand(void)
{
    SPI_resetRxFIFO(SPIA_BASE);
    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RX_OVERRUN |
                             SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RXFF |
                             SPI_INT_RXFF_OVERFLOW | SPI_INT_TXFF);
    endat1_resetCLB();
    TxData[0] = 0x00000700;
    TxData[1] = 0x00000000;
    TxData[2] = 0x00000000;
    int i;
    for (i = 0;i < FIFO_LEVEL;i++)
        {
            SPI_writeDataNonBlocking(SPIA_BASE,TxData[i]);
        }
}

void endat1_resetCLB(void)
{

    CLB_setGPREG(CLB5_BASE, 0);

    //
    // Turn OFF the CLB functionality
    //
    EALLOW;
    HWREG(CLB5_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) = 0;
    EDIS;



    CLB_setOutputMask(CLB5_BASE, 0, false);

}

