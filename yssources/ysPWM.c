/******************************************************************************
| includes
|-------------------------------------------------------------------------------------------*/
#include "ysPWM.h"

/******************************************************************************
| variables
|--------------------------------------------------------------------------------------------*/

/******************************************************************************
| functions
|-------------------------------------------------------------------------------------------*/
void InitPWM()
{
/* Grid Side */

	// ----------------EPwm1---------------------
	EPwm1Regs.TBPHS.half.TBPHS = 0;  // ʱ�����ڼĴ���
	EPwm1Regs.TBCTR = 0;  // ʱ�������Ĵ�������
	EPwm1Regs.TBCTL.bit.PHSDIR = TB_UP;
	EPwm1Regs.TBCTL.bit.CLKDIV = prediv;  // ʱ��Ԥ��Ƶ
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;

	EPwm1Regs.CMPA.half.CMPA = period / 2; // duty_cycle = 0.5
	EPwm1Regs.CMPB = period / 2;
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // TBCTR = 0ʱװ��
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	EPwm1Regs.AQCTLA.bit.CAU = AQ_TOGGLE;
	EPwm1Regs.AQCTLB.bit.CBU = AQ_TOGGLE;
	EPwm1Regs.AQCTLA.bit.PRD = AQ_CLEAR;
	EPwm1Regs.AQCTLB.bit.PRD = AQ_CLEAR;

	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;  // ���²������������ر�

	EPwm1Regs.ETSEL.bit.INTEN = 1;  // ʹ��EPwm1�ж�
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;  // TBCTR = period�����ж�
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;  // ÿ���ж϶���Ӧ

	   // ----------------EPwm2---------------------
	EPwm2Regs.TBPHS.half.TBPHS = 0;  // ʱ�����ڼĴ���
	EPwm2Regs.TBCTR = 0;  // ʱ�������Ĵ�������
	EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP;
	EPwm2Regs.TBCTL.bit.CLKDIV = prediv;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;

	EPwm2Regs.CMPA.half.CMPA = period / 2; // duty_cycle = 0.5
	EPwm2Regs.CMPB = period / 2;
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	EPwm2Regs.AQCTLA.bit.CAU = AQ_TOGGLE;
	EPwm2Regs.AQCTLB.bit.CBU = AQ_TOGGLE;
	EPwm2Regs.AQCTLA.bit.PRD = AQ_CLEAR;
	EPwm2Regs.AQCTLB.bit.PRD = AQ_CLEAR;

	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

	// ----------------EPwm3---------------------
	EPwm3Regs.TBPHS.half.TBPHS = 0;  // ʱ�����ڼĴ���
	EPwm3Regs.TBCTR = 0;  // ʱ�������Ĵ�������
	EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;
	EPwm3Regs.TBCTL.bit.CLKDIV = prediv;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;

	EPwm3Regs.CMPA.half.CMPA = period / 2; // duty_cycle = 0.5
	EPwm3Regs.CMPB = period / 2;
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	EPwm3Regs.AQCTLA.bit.CAU = AQ_TOGGLE;
	EPwm3Regs.AQCTLB.bit.CBU = AQ_TOGGLE;
	EPwm3Regs.AQCTLA.bit.PRD = AQ_CLEAR;
	EPwm3Regs.AQCTLB.bit.PRD = AQ_CLEAR;

	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

/* Load Side */

	// ----------------EPwm4---------------------
	EPwm4Regs.TBPHS.half.TBPHS = 0;  // ʱ�����ڼĴ���
	EPwm4Regs.TBCTR = 0;  // ʱ�������Ĵ�������
	EPwm4Regs.TBCTL.bit.PHSDIR = TB_UP;
	EPwm4Regs.TBCTL.bit.CLKDIV = prediv;  // ʱ��Ԥ��Ƶ
	EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;

	EPwm4Regs.CMPA.half.CMPA = period / 4; // duty_cycle = 0.5
	EPwm4Regs.CMPB = period / 4;
	EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // TBCTR = 0ʱװ��
	EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	EPwm4Regs.AQCTLA.bit.CAU = AQ_TOGGLE;

	EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
	EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // A����ת��B��ת
	EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm4Regs.DBRED = DT; // Deadzone
	EPwm4Regs.DBFED = DT;

	EPwm4Regs.ETSEL.bit.INTEN = 1;  // ʹ��EPwm1�ж�
	EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;  // TBCTR = 0�����ж�
	EPwm4Regs.ETPS.bit.INTPRD = ET_1ST;  // ÿ���ж϶���Ӧ

	// ----------------EPwm5---------------------
	EPwm5Regs.TBPHS.half.TBPHS = 0;  // ʱ�����ڼĴ���
	EPwm5Regs.TBCTR = 0;  // ʱ�������Ĵ�������
	EPwm5Regs.TBCTL.bit.PHSDIR = TB_UP;
	EPwm5Regs.TBCTL.bit.CLKDIV = prediv;  // ʱ��Ԥ��Ƶ
	EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;

	EPwm5Regs.CMPA.half.CMPA = period / 4; // duty_cycle = 0.5
	EPwm5Regs.CMPB = period / 4;
	EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // TBCTR = 0ʱװ��
	EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;  // TBCTR = 0ʱװ��

	EPwm5Regs.AQCTLA.bit.CAU = AQ_TOGGLE;

	//EPwm5Regs.DBCTL.bit.IN_MODE = DBB_RED_DBA_FED;
	EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
	EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // A����ת��B��ת
	EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm5Regs.DBRED = DT; // Deadzone
	EPwm5Regs.DBFED = DT;

	// ----------------EPwm6---------------------
	EPwm6Regs.TBPHS.half.TBPHS = 0;  // ʱ�����ڼĴ���
	EPwm6Regs.TBCTR = 0;  // ʱ�������Ĵ�������
	EPwm6Regs.TBCTL.bit.PHSDIR = TB_UP;
	EPwm6Regs.TBCTL.bit.CLKDIV = prediv;  // ʱ��Ԥ��Ƶ
	EPwm6Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;

	EPwm6Regs.CMPA.half.CMPA = period / 4; // duty_cycle = 0.5
	EPwm6Regs.CMPB = period / 4;
	EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // TBCTR = 0ʱװ��
	EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;  // TBCTR = 0ʱװ��

	EPwm6Regs.AQCTLA.bit.CAU = AQ_TOGGLE;

	//EPwm6Regs.DBCTL.bit.IN_MODE = DBB_RED_DBA_FED;
	EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
	EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // A����ת��B��ת
	EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm6Regs.DBRED = DT; // Deadzone
	EPwm6Regs.DBFED = DT;

	// ----------------��ʼ��ʱ---------------------
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;  // ���ϼ���
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;

	EPwm1Regs.TBPRD = period;  // ��������
	EPwm2Regs.TBPRD = period;
	EPwm3Regs.TBPRD = period;
	EPwm4Regs.TBPRD = period / 2;
	EPwm5Regs.TBPRD = period / 2;
	EPwm6Regs.TBPRD = period / 2;

	EPwm4Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
	EPwm5Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
	EPwm6Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
}
