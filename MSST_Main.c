/*
 * main.c
 */

#include <MSST_PWM.h>
#include "MSST_GlobalFunctions.h"
#include "F28x_Project.h"
#include "Syncopation_SCI.h"

#include "Syncopation_Data.h"
#include "MSST_PWM.h"

#pragma CODE_SECTION(deadloop, ".TI.ramfunc");
void deadloop();
void CpuTimerInit();
void CpuTimerIsr();

extern float I_dc1;
extern float I_dc2;

#define CPU_INT_MSEC 20

void main(void) {
	InitSysCtrl();

	EALLOW;
	ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;
	ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;
	EDIS;

    MSSTGpioConfig();
    GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;  // Not prepared yet.
    CPU_LED_BIT = 0;
    DINT;
    InitPieCtrl();
    InterruptInit();

    SCI_Config();
    AdcInit();
    PwmInit();

    // Disable PWM at first
    Pwm_DIS();

    CpuTimerInit();
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;  // Enable the PIE block
    IER = M_INT1 | M_INT9;

    // PWM interrupts
    IER |= M_INT3;

    EINT;
	deadloop();
}


Uint16 log_send_count = 0;

extern float Iac;
extern float Vdc;
extern float Idc;
extern Uint16 State;
extern Uint16 Status;
extern Uint16 Prd;
extern Uint16 Duty;

#pragma CODE_SECTION(deadloop, ".TI.ramfunc");
void deadloop()
{
    while(1)
    {

        SCI_UpdatePacketFloat(0, I_dc1);
        SCI_UpdatePacketFloat(1, Vdc);
        SCI_UpdatePacketFloat(2, I_dc2);

        SCI_UpdatePacketInt16(0, State);
        SCI_UpdatePacketInt16(1, Status);
        SCI_UpdatePacketInt16(2, (int16_t)Prd);
        SCI_UpdatePacketInt16(3, (int16_t)Duty);
        SCI_UpdatePacketInt16(0, 44);
        SCI_UpdatePacketInt16(1, 45);
        SCI_UpdatePacketInt16(2, 46);
        SCI_UpdatePacketInt16(3, 47);
        SCI_SendPacket();

        DELAY_US(4000);
    }
}

void CpuTimerInit()
{
    CpuTimer1Regs.TCR.all = 0x4010;
    CpuTimer1Regs.PRD.all = 200000 * CPU_INT_MSEC;
    EALLOW;
    PieVectTable.TIMER1_INT = &CpuTimerIsr;
    EDIS;

    DELAY_US(1);
//    CpuTimer1Regs.TCR.all = 0x4000;
}

#pragma CODE_SECTION(CpuTimerIsr, ".TI.ramfunc");
__interrupt void CpuTimerIsr()
{
    CPU_LED_TOGGLE = 1;

    CpuTimer1Regs.TCR.bit.TIF = 1;
}
