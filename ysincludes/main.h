/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "ysPWM.h"
#include "ysPORT.h"
#include "ysADC.h"
#include "ysDAC.h"
#include "ysSCI.h"
#include "ysSPI.h"
#include "imcontrol.h"

/******************************************************************************
| defines
|----------------------------------------------------------------------------*/
#define HallRatioV1  135.5  // 电压霍尔传感器变比1
#define HallRatioV2  135.0  // 电压霍尔传感器变比2
#define HallRatioC1   0.74  // 电流霍尔传感器变比1
#define HallRatioC2   0.74  // 电流霍尔传感器变比1
#define pi 3.1415926

/******************************************************************************
| variables
|----------------------------------------------------------------------------*/

/******************************************************************************
| local functions prototypes
|----------------------------------------------------------------------------*/
interrupt void epwm1_timer_isr(void);
interrupt void ISRTimer0(void);
int sign(double);  // sign函数
