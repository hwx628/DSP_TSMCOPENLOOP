/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "main.h"

/******************************************************************************
| local variable definitions
|----------------------------------------------------------------------------*/
Uint16 invprd1 = period / 2, invprd2 = period / 2;  // ���������׶ζ�Ӧʱ����
Uint16 Tinv1[3] = {period / 4, period / 4, period / 4};  // ��һ�׶��������PWM�Ƚ�ֵ
Uint16 Tinv2[3] = {period / 4, period / 4, period / 4};  // �ڶ��׶��������PWM�Ƚ�ֵ
double mSample[4];  // AD������ѹֵ
double Uab, Ubc, Uca;
double Ud1, Ud2;
Uint16 dutycycle;  // ����ռ�ձ�
Uint16 recsector;  // ��������

double Angle = 0;

/******************************************************************************
@brief   Main

@param   N/A

@return  N/A
******************************************************************************/
void main()
{
   InitSysCtrl();  // ��ʼ��ϵͳ����

   DINT;  // ��ֹ�ж�

   InitPieCtrl();  // ��ʼ��PIE

   IER = 0x0000;
   IFR = 0x0000;

   InitPieVectTable();

   EALLOW;
   PieVectTable.EPWM1_INT = &epwm1_timer_isr;  // ePWM1�жϺ������
   PieVectTable.TINT0 = &ISRTimer0;;
   EDIS;

   InitPORT();
   InitPWM();
   InitADC();
   InitSCI();
   InitSPI();
   InitCpuTimers();  // ����ת�ٺ�ת�ٸ���ֵ

   ConfigCpuTimer(&CpuTimer0, 150, 100000);  // 100ms
   CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0

   IER |= M_INT3;  // enable ePWM CPU_interrupt
   IER |= M_INT1;
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // enable ePWM1 pie_interrupt
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

   EINT;   // ���ж� INTM ʹ��
   ERTM;   // Enable Global realtime interrupt DBGM

   //int msg[9] = {15, 00, 5, 30, -15, 0, 5, 5, 5}; // {n1, n2, torque, ia, ib, id, iq, lambdard, lambdarq};
   //sciNumber(msg);

   int i;
   for(; ;)
   {
	   asm("NOP");
	   for(i=1;i<=10;i++)
	   {}
   }

}

interrupt void epwm1_timer_isr(void)
{

	int Sa, Sb, Sc;  // ��ѹ��ֵ��

	/* ǿ�������ʼֵ */
	EPwm4Regs.AQSFRC.bit.OTSFA = 1;
	EPwm5Regs.AQSFRC.bit.OTSFA = 1;
	EPwm6Regs.AQSFRC.bit.OTSFA = 1;

	EPwm1Regs.AQSFRC.bit.OTSFA = 1;
	EPwm1Regs.AQSFRC.bit.OTSFB = 1;
	EPwm2Regs.AQSFRC.bit.OTSFA = 1;
	EPwm2Regs.AQSFRC.bit.OTSFB = 1;
	EPwm3Regs.AQSFRC.bit.OTSFA = 1;
	EPwm3Regs.AQSFRC.bit.OTSFB = 1;

	/* �ڶ��׶μĴ������ã� ʱ�� < invprd1 */
	EPwm4Regs.TBPRD = invprd2;  // �������ںͱȽ�ֵ
	EPwm5Regs.TBPRD = invprd2;
	EPwm6Regs.TBPRD = invprd2;
	EPwm4Regs.CMPA.half.CMPA = Tinv2[0];
	EPwm5Regs.CMPA.half.CMPA = Tinv2[1];
	EPwm6Regs.CMPA.half.CMPA = Tinv2[2];

	while(EPwm4Regs.ETFLG.bit.INT == 1)  // ����жϱ�־
		EPwm4Regs.ETCLR.bit.INT = 1;

	/*================== �������� =====================*/

	/* ��ѹ�������� */
	ParallelRD(mSample, 4);
	Uab = mSample[0] * HallRatioV1;
	Uca = mSample[1] * HallRatioV2;
	Ubc = -Uab - Uca;
	iabc.a = mSample[2] * HallRatioC1;
	iabc.b = mSample[3] * HallRatioC2;
	iabc.c = -iabc.a - iabc.b;
	uabc.a = (Uab - Uca) / 3.0;
	uabc.b = -(Uab * 2 + Uca) / 3.0;
	uabc.c = -uabc.a - uabc.b;

	//DACout(1, Uab / 50.0);
	//DACout(2, Ubc / 50.0);
	//DACout(3, Uca / 50.0);

	/* ���������ж� */
	Sa = sign(uabc.a);
	Sb = sign(uabc.b);
	Sc = sign(uabc.c);

	if (Sa == 1 && Sb == 0 && Sc == 0)
		recsector = 1;
	else if (Sa == 1 && Sb == 1 && Sc == 0)
		recsector = 2;
	else if (Sa == 0 && Sb == 1 && Sc == 0)
		recsector = 3;
	else if (Sa == 0 && Sb == 1 && Sc == 1)
		recsector = 4;
	else if (Sa == 0 && Sb == 0 && Sc == 1)
		recsector = 5;
	else if (Sa == 1 && Sb == 0 && Sc == 1)
		recsector = 6;
	else
		recsector = 0;

	/* ������ռ�ձȼ��� */
   	switch (recsector)
   	{
       case 1:
       {
    	   dutycycle = (int)(-uabc.b / uabc.a * period);
    	   if (dutycycle  <= limitclk)  // ��ֹ����ʱ�����
    		   dutycycle = limitclk;
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    		   dutycycle = period - limitclk;

    	   Ud1 = Uab;
    	   Ud2 = -Uca;

    	   /* �������ڳ�ʼ��ƽ���� */
    	   EPwm1Regs.AQSFRC.bit.ACTSFA = AQ_SET;  // AU, BL
    	   EPwm1Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
    	   EPwm2Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
    	   EPwm2Regs.AQSFRC.bit.ACTSFB = AQ_SET;
    	   EPwm3Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
    	   EPwm3Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;

    	   /* �����رȽ�ֵ���� */
    	   EPwm1Regs.CMPA.half.CMPA = period + 1;  // BL CL
    	   EPwm1Regs.CMPB = period + 1;
    	   EPwm2Regs.CMPA.half.CMPA = period + 1;
    	   EPwm2Regs.CMPB = dutycycle;
    	   EPwm3Regs.CMPA.half.CMPA = period + 1;
    	   EPwm3Regs.CMPB = dutycycle + DT;  // ��ֹ�����·
    	   break;
       }

       case 2:
       {
    	   dutycycle = (int)(-uabc.b / uabc.c * period);
    	   if (dutycycle  <= limitclk)
    		   dutycycle = limitclk;
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    		   dutycycle = period - limitclk;

    	   Ud1 = Ubc;
    	   Ud2 = -Uca;

     	   EPwm1Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;  // BU, CL
     	   EPwm1Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
     	   EPwm2Regs.AQSFRC.bit.ACTSFA = AQ_SET;
     	   EPwm2Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
     	   EPwm3Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
     	   EPwm3Regs.AQSFRC.bit.ACTSFB = AQ_SET;

     	   EPwm1Regs.CMPA.half.CMPA = dutycycle + DT;  //BU, AU
     	   EPwm1Regs.CMPB = period + 1;
     	   EPwm2Regs.CMPA.half.CMPA = dutycycle;
     	   EPwm2Regs.CMPB = period + 1;
     	   EPwm3Regs.CMPA.half.CMPA = period + 1;
     	   EPwm3Regs.CMPB = period + 1;
     	   break;
        }

        case 3:
        {
     	   dutycycle = (int)(-uabc.c / uabc.b * period);
    	   if (dutycycle  <= limitclk)
    		   dutycycle = limitclk;
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    		   dutycycle = period - limitclk;

    	   Ud1 = Ubc;
    	   Ud2 = -Uab;

     	   EPwm1Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;  // BU, CL
     	   EPwm1Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
     	   EPwm2Regs.AQSFRC.bit.ACTSFA = AQ_SET;
     	   EPwm2Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
     	   EPwm3Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
     	   EPwm3Regs.AQSFRC.bit.ACTSFB = AQ_SET;

     	   EPwm1Regs.CMPA.half.CMPA = period + 1;  // CL, AL
     	   EPwm1Regs.CMPB = dutycycle + DT;
     	   EPwm2Regs.CMPA.half.CMPA = period + 1;
     	   EPwm2Regs.CMPB = period+1;
     	   EPwm3Regs.CMPA.half.CMPA = period + 1;
     	   EPwm3Regs.CMPB = dutycycle;
     	   break;
        }

        case 4:
        {
     	   dutycycle = (int)(-uabc.c / uabc.a * period);
    	   if (dutycycle  <= limitclk)
    		   dutycycle = limitclk;
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    		   dutycycle = period - limitclk;

    	   Ud1 = Uca;
    	   Ud2 = -Uab;

     	   EPwm1Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;  // AL, CU
     	   EPwm1Regs.AQSFRC.bit.ACTSFB = AQ_SET;
     	   EPwm2Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
     	   EPwm2Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
     	   EPwm3Regs.AQSFRC.bit.ACTSFA = AQ_SET;
     	   EPwm3Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;

     	   EPwm1Regs.CMPA.half.CMPA = period + 1;  // CU, BU
     	   EPwm1Regs.CMPB = period + 1;
     	   EPwm2Regs.CMPA.half.CMPA = dutycycle + DT;
     	   EPwm2Regs.CMPB = period + 1;
     	   EPwm3Regs.CMPA.half.CMPA = dutycycle;
     	   EPwm3Regs.CMPB = period + 1;
     	   break;
        }

        case 5:
        {
     	   dutycycle = (int)(-uabc.a / uabc.c * period);
    	   if (dutycycle  <= limitclk)
    		   dutycycle = limitclk;
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    		   dutycycle = period - limitclk;

    	   Ud1 = Uca;
    	   Ud2 = -Ubc;

     	   EPwm1Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;  // AL, CU
     	   EPwm1Regs.AQSFRC.bit.ACTSFB = AQ_SET;
     	   EPwm2Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
     	   EPwm2Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
     	   EPwm3Regs.AQSFRC.bit.ACTSFA = AQ_SET;
     	   EPwm3Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;

     	   EPwm1Regs.CMPA.half.CMPA = period + 1;  // AL BL
     	   EPwm1Regs.CMPB = dutycycle;
     	   EPwm2Regs.CMPA.half.CMPA = period + 1;
     	   EPwm2Regs.CMPB = dutycycle + DT;
     	   EPwm3Regs.CMPA.half.CMPA = period + 1;
     	   EPwm3Regs.CMPB = period + 1;
     	   break;
        }

        case 6:
        {
     	   dutycycle = (int)(-uabc.a / uabc.b * period);
    	   if (dutycycle  <= limitclk)
    		   dutycycle = limitclk;
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    		   dutycycle = period - limitclk;

    	   Ud1 = Uab;
    	   Ud2 = -Ubc;

     	   EPwm1Regs.AQSFRC.bit.ACTSFA = AQ_SET;  // AU, BL
     	   EPwm1Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
     	   EPwm2Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
     	   EPwm2Regs.AQSFRC.bit.ACTSFB = AQ_SET;
     	   EPwm3Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
     	   EPwm3Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;

     	   EPwm1Regs.CMPA.half.CMPA = dutycycle;  // AU, CU
     	   EPwm1Regs.CMPB = period + 1;
     	   EPwm2Regs.CMPA.half.CMPA = period + 1;
     	   EPwm2Regs.CMPB = period + 1;
     	   EPwm3Regs.CMPA.half.CMPA = dutycycle + DT;
     	   EPwm3Regs.CMPB = period + 1;
     	   break;
        }
        default:
        {
           Ud1 = 0;
           Ud2 = 0;
      	   EPwm1Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
      	   EPwm1Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
      	   EPwm2Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
      	   EPwm2Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
      	   EPwm3Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
      	   EPwm3Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;

      	   EPwm1Regs.CMPA.half.CMPA = period + 1;
      	   EPwm1Regs.CMPB = period + 1;
      	   EPwm2Regs.CMPA.half.CMPA = period + 1;
      	   EPwm2Regs.CMPB = period + 1;
      	   EPwm3Regs.CMPA.half.CMPA =  period + 1;
      	   EPwm3Regs.CMPB = period + 1;
        }
   	}

   	/*================== ��䲿�� ===================*/

   	/* ��������������� */
   	invprd1 = dutycycle;
   	invprd2 = period - invprd1;

   	u_cmd = RAMP(VSpdramp, 0, spd_cmd, Voltlimit_H, Voltlimit_L);
    theta += 0.0000418879 * spd_cmd; // theta += 2 * pi * (spd_cmd / 30.0) * 0.0002;
    if (theta > 6.2831852)  // 2 * pi = 6.2831852
      theta -= 6.2831852;
    ualbe_cmd.al = u_cmd * cosIn;
    ualbe_cmd.be = u_cmd * sinIn;

   	/* ����������ѹ */
    ualbe_cmd.al = 40 * cos(2*pi*20 * (period_count/10000.0));
    ualbe_cmd.be = 40 * cos(2*pi*20 * (period_count/10000.0) - 0.5*pi);

    period_count++;
    if (period_count > 10000)
    {
      period_count = 0;
    }

    /* SVM */
   	ualbeSVM(ualbe_cmd.al, ualbe_cmd.be, Ud, invprd1, invprd2, Tinv1, Tinv2);

   	while(EPwm4Regs.ETFLG.bit.INT != 1){}  // ��һ�׶ν���

   	/* �������ںͱȽ�ֵ */
	EPwm4Regs.TBPRD = invprd1;
	EPwm5Regs.TBPRD = invprd1;
	EPwm6Regs.TBPRD = invprd1;
   	EPwm4Regs.CMPA.half.CMPA = Tinv1[0];
   	EPwm5Regs.CMPA.half.CMPA = Tinv1[1];
   	EPwm6Regs.CMPA.half.CMPA = Tinv1[2];

   // Clear INT flag for this timer
   	while(EPwm1Regs.ETFLG.bit.INT == 1)
   		EPwm1Regs.ETCLR.bit.INT = 1;

   // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

interrupt void ISRTimer0(void)
{
	CpuTimer0.InterruptCount ++;
	if (CpuTimer0.InterruptCount  > 15) CpuTimer0.InterruptCount -= 16;

	// SCI
/*	int temp;
	temp = floor(sin(Angle) * 100 + 100);
	Angle += 2 * pi * 50 * 0.001;
	if (Angle > 2 * pi)      Angle -= 2*pi;

	int msg[SCINumSend]; // {n1, n2, torque, ia, ib, id, iq, lambdard, lambdarq};
	int i;
	for (i = 0; i < SCINumSend; i++)
		msg[i] = temp;

	sciNumber(msg);*/

	// SPI
    spiSend(CpuTimer0.InterruptCount);

	// Acknowledge this interrupt to receive more interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	CpuTimer0Regs.TCR.bit.TIF=1;
	CpuTimer0Regs.TCR.bit.TRB=1;
}

int sign(double param)
{
	if (param >= 0)
		return 1;
	else
		return 0;
}
