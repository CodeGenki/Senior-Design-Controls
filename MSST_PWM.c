/*
 * MSST_Pwm.c
 *
 *  Created on: Oct 18, 2016
 *      Author: Yang Lei
 */

#include "F28x_Project.h"

// Macros to switch between testing interfaces
#define LAUNCHPAD_PWM   0
#define BOARD_PWM       1
#define DEFAULT_PWM     0

#define PWM_PRD     2000
#define PWM_DB_INIT 1980
#define PWM_DB      40

#define DAB_PRD     2000
#define DAB_OFFSET  29

#define EPWM2_TIMER_TBPRD  500  // Period register
#define EPWM2_CMPA         EPWM2_TIMER_TBPRD/2
#define EPWM2_MAX_CMPA     1950
#define EPWM2_MIN_CMPA       50
#define EPWM2_MAX_CMPB     1950
#define EPWM2_MIN_CMPB       50

#define EPWM2_MAX_DB   0x03FF
#define EPWM2_MIN_DB   0
#define DB_UP          1
#define DB_DOWN        0

#define EPWM6_TIMER_TBPRD   500
#define EPWM6_CMPA          EPWM6_TIMER_TBPRD/2

int EPWM7_TIMER_TBPRD  = 0;
int EPWM7_CMPA         = 0;

int EPWM8_TIMER_TBPRD  = 0;
int EPWM8_CMPA         = 0;


Uint32 EPwm2TimerIntCount;
Uint16 EPwm2_DB_Direction;

Uint16 dab_prd = PWM_PRD;
int16 dab_phs = 0;

void Epwm1Init()
{
    EPwm1Regs.TBPRD = DAB_PRD;

    EPwm1Regs.TBCTL.bit.SYNCOSEL = 1; // Sync when TBCTR = 0
    EPwm1Regs.TBCTL.bit.PHSEN = 1; // Enable phase shift
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)

//    EPwm1Regs.TBCTL.bit.CTRMODE = 2;

    EPwm1Regs.ETSEL.bit.SOCASEL = 1; // SOCA at counter equals to 0
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  // Enable SOCA
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;  // Pulse at every event
}

void Epwm2Init()
{
    EPwm2Regs.TBPRD = 3999;

    EPwm2Regs.TBCTL.bit.SYNCOSEL = 0; // Sync when TBCTR = 0
    EPwm2Regs.TBCTL.bit.PHSEN = 1; // Enable phase shift
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; // TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)

    EPwm2Regs.AQCTLA.bit.CAU = 1; // Clear output A at TBCTR = CMPA
    EPwm2Regs.AQCTLA.bit.ZRO = 2; // Set output A at TBCTR = 0
    EPwm2Regs.DBRED.bit.DBRED = PWM_DB-1;
    EPwm2Regs.DBFED.bit.DBFED = PWM_DB-1;
    EPwm2Regs.DBCTL.bit.IN_MODE = 0;
    EPwm2Regs.DBCTL.bit.POLSEL = 2;

    EPwm2Regs.CMPA.bit.CMPA = 2000;

//    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Count-up mode, start the counter

    EALLOW;
    EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EDIS;
    EPwm2Regs.DBCTL.bit.OUT_MODE = 3;
}

void Epwm6Init()
{
    EPwm6Regs.TBPRD = 3999;

    EPwm6Regs.TBCTL.bit.SYNCOSEL = 0; // Sync when TBCTR = 0
    EPwm6Regs.TBCTL.bit.PHSEN = 1; // Enable phase shift
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = 0; // TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)

    EPwm6Regs.AQCTLA.bit.CAU = 1; // Clear output A at TBCTR = CMPA
    EPwm6Regs.AQCTLA.bit.ZRO = 2; // Set output A at TBCTR = 0
    EPwm6Regs.DBRED.bit.DBRED = PWM_DB-1;
    EPwm6Regs.DBFED.bit.DBFED = PWM_DB-1;
    EPwm6Regs.DBCTL.bit.IN_MODE = 0;
    EPwm6Regs.DBCTL.bit.POLSEL = 2;
    EPwm6Regs.DBCTL.bit.OUTSWAP = 3;

    EPwm6Regs.CMPA.bit.CMPA = 2000;

//    EPwm6Regs.TBCTL.bit.CTRMODE = 0; // Count-up mode, start the counter


    EALLOW;
    EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm6Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPwm6Regs.TZFRC.bit.OST = 1;
    EDIS;
    EPwm6Regs.DBCTL.bit.OUT_MODE = 3;
}

void Epwm7Init()
{
    EPwm7Regs.TBPRD = EPWM7_TIMER_TBPRD;          // Set timer period 6000 = 3.84ms
    EPwm7Regs.TBPHS.bit.TBPHS = 0;           // Phase is 0
    EPwm7Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm7Regs.TBCTL.bit.PHSDIR = 0;
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    //EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    //
    // Setup compare
    //
    EPwm7Regs.CMPA.bit.CMPA = EPWM7_CMPA;

    //
    // Set actions
    //
    EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
    EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR;


    //
    // Active Low complementary PWMs - setup the deadband
    //
    EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm7Regs.DBRED.bit.DBRED = 10;
    EPwm7Regs.DBFED.bit.DBFED = 10;
}

void Epwm8Init()
{
    EPwm8Regs.TBPRD = EPWM8_TIMER_TBPRD;          // Set timer period 6000 = 3.84ms
    EPwm8Regs.TBPHS.bit.TBPHS = EPWM8_TIMER_TBPRD;           // Phase is 0
    EPwm8Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm8Regs.TBCTL.bit.PHSDIR = 0;
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    //EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV2;
    //
    // Setup compare
    //
    EPwm8Regs.CMPA.bit.CMPA = EPWM8_CMPA;

    //
    // Set actions
    //
    EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
    EPwm8Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    //
    // Active Low complementary PWMs - setup the deadband
    //
    EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm8Regs.DBCTL.bit.OUTSWAP = 3;
    EPwm8Regs.DBRED.bit.DBRED = 10;
    EPwm8Regs.DBFED.bit.DBFED = 10;
}

void Epwm9Init()
{
    EPwm9Regs.TBPRD = 3999;

    EPwm9Regs.TBCTL.bit.SYNCOSEL = 0; // Sync when TBCTR = 0
    EPwm9Regs.TBCTL.bit.PHSEN = 1; // Enable phase shift
    EPwm9Regs.TBCTL.bit.HSPCLKDIV = 0; // TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)

    EPwm9Regs.AQCTLA.bit.CAU = 1; // Clear output A at TBCTR = CMPA
    EPwm9Regs.AQCTLA.bit.ZRO = 2; // Set output A at TBCTR = 0

    EPwm9Regs.DBRED.bit.DBRED = 0;
    EPwm9Regs.DBFED.bit.DBFED = 0;
    EPwm9Regs.DBCTL.bit.IN_MODE = 0;
    EPwm9Regs.DBCTL.bit.POLSEL = 2;

    EPwm9Regs.CMPA.bit.CMPA = 30;
    EPwm9Regs.TBPHS.bit.TBPHS = DAB_OFFSET;

//    EPwm9Regs.TBCTL.bit.CTRMODE = 0; // Up-down count mode, start the counter

    EALLOW;
    EPwm9Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm9Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EDIS;
    EPwm9Regs.DBCTL.bit.OUT_MODE = 3;
}


void ECapInit()
{
    EALLOW;
    InputXbarRegs.INPUT7SELECT = 17; // GPIO-17 to ECAP1
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;
    ECap1Regs.ECCTL1.bit.CTRRST4 = 1;
    ECap1Regs.ECCTL1.bit.CAP3POL = 1;
    ECap1Regs.ECCTL1.bit.CTRRST2 = 1;
    ECap1Regs.ECCTL1.bit.CAP1POL = 1;
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;
    EDIS;
}

void PwmInit()
{
    EALLOW;
    DESIRED_FREQ = 36;
    EPWM7_TIMER_TBPRD  = 100000/(2*DESIRED_FREQ);
    EPWM7_CMPA         = EPWM7_TIMER_TBPRD/2;

    EPWM8_TIMER_TBPRD  = 100000/(2*DESIRED_FREQ);
    EPWM8_CMPA         = EPWM8_TIMER_TBPRD/2;
    InputXbarRegs.INPUT5SELECT = 17;
    EDIS;
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;
    EDIS;
    ECapInit();
    Epwm1Init();
    Epwm2Init();
    Epwm6Init();
    Epwm7Init();
    Epwm8Init();
    Epwm9Init();

    EPwm1Regs.TBCTL.bit.CTRMODE = 2;
//    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
//    EPwm6Regs.TBCTL.bit.CTRMODE = 0;
    EPwm7Regs.TBCTL.bit.CTRMODE = 2;
    EPwm8Regs.TBCTL.bit.CTRMODE = 2;
    EPwm9Regs.TBCTL.bit.CTRMODE = 0;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;
    EDIS;
}

void Rectifier_EN()
{
    EALLOW;
    //EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm6Regs.TZCLR.bit.OST = 1;
    EDIS;
}

void Rectifier_DIS()
{
    EALLOW;
    //EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm6Regs.TZFRC.bit.OST = 1;
    EDIS;
}

// dual active bridge primary side
void DabPri_EN()
{
    EALLOW;
    EPwm7Regs.TZCLR.bit.OST = 1;
    EPwm8Regs.TZCLR.bit.OST = 1;
    EDIS;
}

void DabPri_DIS()
{
    EALLOW;
    EPwm7Regs.TZFRC.bit.OST = 1;
    EPwm8Regs.TZFRC.bit.OST = 1;
    EDIS;
}

void Pwm_EN()
{
    EALLOW;
//    EPwm2Regs.TZCLR.bit.OST = 1;
//    EPwm6Regs.TZCLR.bit.OST = 1;
//    EPwm7Regs.TZCLR.bit.OST = 1;
//    EPwm8Regs.TZCLR.bit.OST = 1;
//    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM7=1;
    CpuSysRegs.PCLKCR2.bit.EPWM8=1;
    EDIS;
}


void Pwm_DIS()
{
    EALLOW;
//    EPwm2Regs.TZFRC.bit.OST = 1;
//    EPwm6Regs.TZFRC.bit.OST = 1;
//    EPwm7Regs.TZFRC.bit.OST = 1;
//    EPwm8Regs.TZFRC.bit.OST = 1;
//    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM7=0;
    CpuSysRegs.PCLKCR2.bit.EPWM8=0;
    EDIS;
}

void Change_Freq(int freq_num) {
    Pwm_DIS();
    DESIRED_FREQ = freq_num;
    EPWM7_TIMER_TBPRD  = 100000/(2*DESIRED_FREQ);
    EPWM7_CMPA         = EPWM7_TIMER_TBPRD/2;

    EPWM8_TIMER_TBPRD  = 100000/(2*DESIRED_FREQ);
    EPWM8_CMPA         = EPWM8_TIMER_TBPRD/2;

    InitEPwm7Example();
    InitEPwm8Example();
    Pwm_EN();
}

void RectDuty_SET(float duty)
{
    if(duty > 0.98)
        duty = 0.98;
    if(duty < -0.98)
        duty = -0.98;
    Uint16 cmp_value = (Uint16)(2000 * (1 + duty));
    EPwm2Regs.CMPA.bit.CMPA = cmp_value;
    EPwm6Regs.CMPA.bit.CMPA = cmp_value;
}

void RectCmp_SET(Uint16 cmp)
{
    if(cmp > 3500)
        cmp = 3500;
    if(cmp < 500)
        cmp = 500;
    EPwm2Regs.CMPA.bit.CMPA = cmp;
    EPwm6Regs.CMPA.bit.CMPA = cmp;
}
