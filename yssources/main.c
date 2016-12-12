/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "main.h"

/******************************************************************************
| local variable definitions
|----------------------------------------------------------------------------*/
Uint16 invprd1 = period / 2, invprd2 = period / 2;
Uint16 Tinv1[3] = {period / 4, period / 4, period / 4};
Uint16 Tinv2[3] = {period / 4, period / 4, period / 4};
double mSample[4];
double Uab, Ubc, Uca;
double drec;  // ����ռ�ձ�
Uint16 dutycycle;  // ����ռ�ձȣ�ʱ������
Uint16 rec_sector;

double Angle = 0;

/******************************************************************************
@brief  Main
******************************************************************************/
void main()
{
   InitSysCtrl();

   DINT;

   InitPieCtrl();

   IER = 0x0000;
   IFR = 0x0000;

   InitPieVectTable();

   EALLOW;
   PieVectTable.EPWM1_INT = &epwm1_timer_isr;  // ePWM1�ж����
   PieVectTable.TINT0 = &ISRTimer0;;
   EDIS;

   InitPORT();
   InitPWM();
   InitADC();
   InitSCI();
   InitSPI();
   InitCpuTimers();  // ����ת�ٺ�ת�ٸ���ֵ

   ConfigCpuTimer(&CpuTimer0, 150, 1000);  // 100ms
   CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
	
   IER |= M_INT3;  // enable ePWM CPU_interrupt
   IER |= M_INT1;  // CpuTimer
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // enable ePWM1 pie_interrupt
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

   EINT;   // ���ж� INTM ʹ��
   ERTM;   // Enable Global realtime interrupt DBGM

   int i;
   for(; ;)
   {
	   asm("          NOP");
	   for(i=1;i<=10;i++)
	   {}
   }

}

interrupt void epwm1_timer_isr(void)
{
	int Sa, Sb, Sc;

	// ----------------ǿ�������ʼֵ---------------------
	EPwm4Regs.AQSFRC.bit.OTSFA = 1;
	EPwm5Regs.AQSFRC.bit.OTSFA = 1;
	EPwm6Regs.AQSFRC.bit.OTSFA = 1;

	EPwm1Regs.AQSFRC.bit.OTSFA = 1;
	EPwm1Regs.AQSFRC.bit.OTSFB = 1;
	EPwm2Regs.AQSFRC.bit.OTSFA = 1;
	EPwm2Regs.AQSFRC.bit.OTSFB = 1;
	EPwm3Regs.AQSFRC.bit.OTSFA = 1;
	EPwm3Regs.AQSFRC.bit.OTSFB = 1;

	/* ��伶�ڶ��׶μĴ������ã� ʱ�� <invprd1 */
	EPwm4Regs.TBPRD = invprd2;
	EPwm5Regs.TBPRD = invprd2;
	EPwm6Regs.TBPRD = invprd2;
	EPwm4Regs.CMPA.half.CMPA = Tinv2[0];
	EPwm5Regs.CMPA.half.CMPA = Tinv2[1];
	EPwm6Regs.CMPA.half.CMPA = Tinv2[2];

	while(EPwm4Regs.ETFLG.bit.INT == 1)
		EPwm4Regs.ETCLR.bit.INT = 1;

	/* ��������*/

	//DELAY_US(0.5);  // ���������л�����

	// Clear INT flag for this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

	// ----------------��ѹ��������---------------------
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

	/* ====�����ж�====*/
	Sa = sign(uabc.a);
	Sb = sign(uabc.b);
	Sc = sign(uabc.c);

	if (Sa == 1 && Sb == 0 && Sc == 0)
		rec_sector = 1;
	else if (Sa == 1 && Sb == 1 && Sc == 0)
		rec_sector = 2;
	else if (Sa == 0 && Sb == 1 && Sc == 0)
		rec_sector = 3;
	else if (Sa == 0 && Sb == 1 && Sc == 1)
		rec_sector = 4;
	else if (Sa == 0 && Sb == 0 && Sc == 1)
		rec_sector = 5;
	else if (Sa == 1 && Sb == 0 && Sc == 1)
		rec_sector = 6;
	else
		rec_sector = 0;

   	switch (rec_sector)
   	{
       case 1:
       {
    	   // ��ֹ����ʱ�����
    	   drec = -uabc.b / uabc.a;
    	   dutycycle = (int)(drec * period);
    	   if (dutycycle  <= limitclk)
    	   {
    		   dutycycle = limitclk;
    		   drec = 0.025;  // limitclk / period = 0.05
    	   }
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    	   {
    		   dutycycle = period - limitclk;
    		   drec = 0.975;
    	   }

    	   Ud = drec * Uab - (1 - drec) * Uca;

    	   EPwm1Regs.AQSFRC.bit.ACTSFA = AQ_SET;  // AU, BL
    	   EPwm1Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
    	   EPwm2Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
    	   EPwm2Regs.AQSFRC.bit.ACTSFB = AQ_SET;
    	   EPwm3Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
    	   EPwm3Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;

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
           drec = -uabc.b / uabc.c;
     	   dutycycle = (int)(drec * period);
    	   if (dutycycle  <= limitclk)
    	   {
    		   dutycycle = limitclk;
    		   drec = 0.025;  // limitclk / period = 0.05
    	   }
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    	   {
    		   dutycycle = period - limitclk;
    		   drec = 0.975;
    	   }

    	   Ud = drec * Ubc - (1 - drec) * Uca;

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
           drec = -uabc.c / uabc.b;
     	   dutycycle = (int)(drec * period);
    	   if (dutycycle  <= limitclk)
    	   {
    		   dutycycle = limitclk;
    		   drec = 0.025;  // limitclk / period = 0.05
    	   }
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    	   {
    		   dutycycle = period - limitclk;
    		   drec = 0.975;
    	   }

    	   Ud = drec * Ubc - (1 - drec) * Uab;

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
           drec = -uabc.c / uabc.a;
     	   dutycycle = (int)(drec * period);
    	   if (dutycycle  <= limitclk)
    	   {
    		   dutycycle = limitclk;
    		   drec = 0.025;  // limitclk / period = 0.05
    	   }
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    	   {
    		   dutycycle = period - limitclk;
    		   drec = 0.975;
    	   }

    	   Ud = drec * Uca - (1 - drec) * Uab;

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
           drec = -uabc.a / uabc.c;
     	   dutycycle = (int)(drec * period);
    	   if (dutycycle  <= limitclk)
    	   {
    		   dutycycle = limitclk;
    		   drec = 0.025;  // limitclk / period = 0.05
    	   }
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    	   {
    		   dutycycle = period - limitclk;
    		   drec = 0.975;
    	   }

    	   Ud = drec * Uca - (1 - drec) * Ubc;

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
           drec = -uabc.a / uabc.b;
     	   dutycycle = (int)(drec * period);
    	   if (dutycycle  <= limitclk)
    	   {
    		   dutycycle = limitclk;
    		   drec = 0.025;  // limitclk / period = 0.05
    	   }
    	   else if (period - dutycycle <= limitclk || period <= dutycycle)
    	   {
    		   dutycycle = period - limitclk;
    		   drec = 0.975;
    	   }

    	   Ud = drec * Uab - (1 - drec) * Ubc;

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
           Ud = 0;

      	   EPwm1Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;  // AU, BL
      	   EPwm1Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
      	   EPwm2Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
      	   EPwm2Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
      	   EPwm3Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
      	   EPwm3Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;

      	   EPwm1Regs.CMPA.half.CMPA = period + 1;  // AU, CU
      	   EPwm1Regs.CMPB = period + 1;
      	   EPwm2Regs.CMPA.half.CMPA = period + 1;
      	   EPwm2Regs.CMPB = period + 1;
      	   EPwm3Regs.CMPA.half.CMPA =  period + 1;
      	   EPwm3Regs.CMPB = period + 1;
        }
   	}

   	/* ��䲿�� */
    double cosIn = cos(theta);
    double sinIn = sin(theta);

   	invprd1 = dutycycle;
   	invprd2 = period - invprd1;

    /* SVM�������� */
    u_cmd = RAMP(VSpdramp, 0, spd_cmd, Ud, Voltlimit_L);
   	//u_cmd = 20;
    theta += 0.0000418879 * spd_cmd; // theta += 2 * pi * (spd_cmd / 30.0) * 0.0002;
    if (theta > 6.2831852)  // 2 * pi = 6.2831852
      theta -= 6.2831852;
    ualbe_cmd.al = u_cmd * cosIn;
    ualbe_cmd.be = u_cmd * sinIn;

   	ualbeSVM(ualbe_cmd.al, ualbe_cmd.be, Ud, invprd1, invprd2, Tinv1, Tinv2);

   	while(EPwm4Regs.ETFLG.bit.INT != 1){}  // ��һ�׶ν���

	EPwm4Regs.TBPRD = invprd1;
	EPwm5Regs.TBPRD = invprd1;
	EPwm6Regs.TBPRD = invprd1;
   	EPwm4Regs.CMPA.half.CMPA = Tinv1[0];  // AU, CU
   	EPwm5Regs.CMPA.half.CMPA = Tinv1[1];
   	EPwm6Regs.CMPA.half.CMPA = Tinv1[2];

/*   	msg[0] = (int)(Uab * 1e5);
   	msg[2] = (int)(Uca * 1e5);
   	sciNumber(msg);*/

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
	int temp;
	temp = floor(sin(Angle) * 100 + 100);
	Angle += 2 * pi * 50 * 0.001;
	if (Angle > 2 * pi)      Angle -= 2*pi;

	int msg[SCINumSend]; // {n1, n2, torque, ia, ib, id, iq, lambdard, lambdarq};
	int i;
	for (i = 0; i < SCINumSend; i++) //msg[i] = (int)(-20.51 * 10);
		msg[i] = 85;

	sciNumber(msg);

	// SPI
    //spiSend(CpuTimer0.InterruptCount);

	  if (spd_cmd < spd_req)
	  {
	    spd_cmd = RAMP(spdramp, spd_cmd, 0.1, spdlimit_H, spdlimit_L);  // ת�ٸ���ֵ����
	  }

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
