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
double drec;  // 整流占空比
Uint16 dutycycle;  // 整流占空比（时钟数）
Uint16 rec_sector;

double Angle = 0;
int indexDA = 7;
double temp = 0;
int count = 0;
double ualsum = 0;
double ubesum = 0;
double ialsum = 0;
double ibesum = 0;
double lamdasalsum = 0;
double lamdasbesum = 0;
double av_al = 0;
double av_be = 0;

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
   PieVectTable.EPWM1_INT = &epwm1_timer_isr;  // ePWM1中断入口
   PieVectTable.TINT0 = &ISRTimer0;;
   EDIS;

   InitPORT();
   InitPWM();
   InitADC();
   InitSCI();
   InitSPI();
   InitCpuTimers();  // 计算转速和转速给定值

   ConfigCpuTimer(&CpuTimer0, 150, 100000);  // 100ms
   CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
	
   IER |= M_INT3;  // enable ePWM CPU_interrupt
   IER |= M_INT1;  // CpuTimer
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // enable ePWM1 pie_interrupt
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

   EINT;   // 总中断 INTM 使能
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

	// ----------------强制输出初始值---------------------
	EPwm4Regs.AQSFRC.bit.OTSFA = 1;
	EPwm5Regs.AQSFRC.bit.OTSFA = 1;
	EPwm6Regs.AQSFRC.bit.OTSFA = 1;

	EPwm1Regs.AQSFRC.bit.OTSFA = 1;
	EPwm1Regs.AQSFRC.bit.OTSFB = 1;
	EPwm2Regs.AQSFRC.bit.OTSFA = 1;
	EPwm2Regs.AQSFRC.bit.OTSFB = 1;
	EPwm3Regs.AQSFRC.bit.OTSFA = 1;
	EPwm3Regs.AQSFRC.bit.OTSFB = 1;

	/* 逆变级第二阶段寄存器设置， 时间 <invprd1 */
	EPwm4Regs.TBPRD = invprd2;
	EPwm5Regs.TBPRD = invprd2;
	EPwm6Regs.TBPRD = invprd2;
	EPwm4Regs.CMPA.half.CMPA = Tinv2[0];
	EPwm5Regs.CMPA.half.CMPA = Tinv2[1];
	EPwm6Regs.CMPA.half.CMPA = Tinv2[2];

	while(EPwm4Regs.ETFLG.bit.INT == 1)
		EPwm4Regs.ETCLR.bit.INT = 1;

	/* 整流部分*/

	//DELAY_US(0.5);  // 整流开关切换死区

	// Clear INT flag for this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

	// ----------------电压电流采样---------------------
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

	/* ====扇区判断====*/
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
    	   // 防止两段时间过短
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
    	   EPwm3Regs.CMPB = dutycycle + DT;  // 防止两相短路
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

   	/* 逆变部分 */
    double cosIn = cos(theta);
    double sinIn = sin(theta);

   	invprd1 = dutycycle;
   	invprd2 = period - invprd1;

    /* SVM开环计算 */
    u_cmd = RAMP(VSpdramp, 0, spd_cmd, Ud, Voltlimit_L);
   	//u_cmd = 20;
    theta += 0.0000418879 * spd_cmd; // theta += 2 * pi * (spd_cmd / 30.0) * 0.0002;
    if (theta > 6.2831852)  // 2 * pi = 6.2831852
      theta -= 6.2831852;
    ualbe_cmd.al = u_cmd * cosIn;
    ualbe_cmd.be = u_cmd * sinIn;

   	ualbeSVM(ualbe_cmd.al, ualbe_cmd.be, Ud, invprd1, invprd2, Tinv1, Tinv2);

   	while(EPwm4Regs.ETFLG.bit.INT != 1){}  // 第一阶段结束

	EPwm4Regs.TBPRD = invprd1;
	EPwm5Regs.TBPRD = invprd1;
	EPwm6Regs.TBPRD = invprd1;
   	EPwm4Regs.CMPA.half.CMPA = Tinv1[0];  // AU, CU
   	EPwm5Regs.CMPA.half.CMPA = Tinv1[1];
   	EPwm6Regs.CMPA.half.CMPA = Tinv1[2];

	/* 3s/2r coordinate transform */
	S3toS2(uabc, &ualbe);
	S3toS2(iabc, &ialbe);

	/* stator flux calculation */
	lamdasalbeCal(ualbe_cmd, ialbe, &lamdasalbe);
	Te = 1.5 * np * (lamdasalbe.al * ialbe.be - lamdasalbe.be * ialbe.al);

	switch(indexDA)
	{
		case 0:
		{
			DACout(0, uabc.a * 0.1);
			DACout(1, uabc.b * 0.1);
			DACout(2, uabc.c * 0.1);
			break;
		}
		case 1:
		{
			ualsum += ualbe_cmd.al;
			ubesum += ualbe_cmd.be;
			count++;
			if(count == 5000)
			{
				av_al = ualsum * 0.0002;
			    av_be = ubesum * 0.0002;
			    count = 0;
			    ualsum = 0;
			    ubesum = 0;
			}
			temp = ualbe_cmd.al * ualbe_cmd.al + ualbe_cmd.be * ualbe_cmd.be;
			DACout(0, ualbe_cmd.al * 0.1);
			DACout(1, ualbe_cmd.be * 0.1);
			DACout(2, temp);
			break;
		}
		case 2:
		{
			DACout(0, iabc.a * 5);
			DACout(1, iabc.b * 5);
			DACout(2, iabc.c * 5);
			break;
		}
		case 3:
		{
			ialsum += ialbe.al;
			ibesum += ialbe.be;
			count++;
			if(count == 5000)
			{
				av_al = ialsum * 0.0002;
			    av_be = ibesum * 0.0002;
			    count = 0;
			    ialsum = 0;
			    ibesum = 0;
			}
			DACout(0, ialbe.al * 5);
			DACout(1, ialbe.be * 5);
			DACout(2, ialbe.al * ialbe.al + ialbe.be * ialbe.be);
			break;
		}
		case 4:
		{
			DACout(0, lamdasalbe.al * 8);
			DACout(1, ialbe.al * 5);
			DACout(2, lamdasalbe.al * lamdasalbe.al + lamdasalbe.be * lamdasalbe.be);
			break;
		}
		case 5:
		{
			DACout(0, lamdasalbe.be * 8);
			DACout(1, ialbe.be * 5);
			DACout(2, lamdasalbe.al * lamdasalbe.al + lamdasalbe.be * lamdasalbe.be);
			break;
		}
		case 6:
		{
			lamdasalsum += lamdasalbe.al;
			lamdasbesum += lamdasalbe.be;
			count++;
			if(count == 5000)
			{
				av_al = lamdasalsum * 0.0002;
			    av_be = lamdasbesum * 0.0002;
			    count = 0;
			    lamdasalsum = 0;
			    lamdasbesum = 0;
			}
			temp = lamdasalbe.al * lamdasalbe.al + lamdasalbe.be * lamdasalbe.be;
			DACout(0, lamdasalbe.al * 8);
			DACout(1, lamdasalbe.be * 8);
			DACout(2, temp);
			break;
		}
		case 7:
		{
			DACout(0, Te);
			DACout(1, lamdasalbe.be * 8);
			DACout(2, temp);
			break;
		}
		case 8:
		{
			DACout(0, 0);
			DACout(1, 0);
			DACout(2, 0);
			count = 0;
			ualsum = 0;
			ubesum = 0;
			ialsum = 0;
			ibesum = 0;
			lamdasalsum = 0;
			lamdasbesum = 0;
			av_al = 0;
			av_be = 0;
			break;
		}
	}

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

	// SPI
    //spiSend(CpuTimer0.InterruptCount);

	if (spd_cmd < spd_req)
	{
		spd_cmd = RAMP(spdramp, spd_cmd, 0.1, spdlimit_H, spdlimit_L);  // 转速给定值计算
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
