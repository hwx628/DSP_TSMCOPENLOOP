/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "math.h"
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "ysPWM.h"

/******************************************************************************
| defines
|----------------------------------------------------------------------------*/
/* IM parameters */
#define Rs 0.4
#define Ls 0.289368
#define Rr 3.3278
#define Lr 0.289368
#define Lm 0.27325
#define Tr 0.0869557
#define np 2

/* control period */
#define Ts 1e-4

/* lamdar最小值限制 */
#define lamdarlimit_L 0.01

/* PI parameters */
  // id闭环
#define ud_Kp 6
#define ud_Ki 0
#define udlimit_H 60
#define udlimit_L -60

  // 速度闭环
#define spdthd 400  // speed threshold
#define iq_Kp1 2
#define iq_Ki1 0
#define iq_Kp2 1
#define iq_Ki2 20
#define iqlimit_H 0
#define iqlimit_L 0

  // iq闭环
#define uq_Kp 3
#define uq_Ki 10
#define uqlimit_H 60
#define uqlimit_L -60

/* speed ramp */
#define spdramp 50  // 斜率
#define spdlimit_H 500  // 转速上限
#define spdlimit_L 0  // 转速下限

/* V/spd curve */
//#define VSpdramp 0.38386  // 斜率
#define VSpdramp 0.15  // 斜率
#define Voltlimit_H 100  // 电压上限
#define Voltlimit_L 10  // 电压下限

/* auxiliary */
#define pi 3.1415926
#define Z 1024  // 光电码盘线数
#define digit 1e6  // roundn参数

/******************************************************************************
| types
|----------------------------------------------------------------------------*/
typedef struct
{
  double a, b, c;
} PHASE_ABC;

typedef struct
{
  double al, be;
} PHASE_ALBE;

typedef struct
{
  double d,q;
} PHASE_DQ;

/******************************************************************************
| global variables
|----------------------------------------------------------------------------*/
/* 观测值 */
  // 电压
extern double Ud;
extern PHASE_ABC uabc;
extern PHASE_ALBE ualbe;
extern PHASE_DQ udq;
  // 电流
extern PHASE_ABC iabc;
extern PHASE_ALBE ialbe;
extern PHASE_DQ idq;
  // 磁链
extern double lamdar;
extern PHASE_ALBE lamdaralbe;
extern double theta;
  // 转速
extern double speed;

/* 给定值 */
  // 电压
extern double u_cmd;
extern PHASE_ALBE ualbe_cmd;
extern PHASE_DQ udq_cmd;
  // 电流
extern PHASE_DQ idq_cmd;
  // 转速
extern double spd_cmd;  // 转速给定
extern double spd_req;  // 转速设定

/* PI 变量 */
extern double idlasterr;
extern double iqlasterr;
extern double spdlasterr;

/******************************************************************************
| local functions prototypes
|----------------------------------------------------------------------------*/
double roundn(double input, int _digit);

/******************************************************************************
| exported functions
|----------------------------------------------------------------------------*/
/* Forward conversion */  
extern void S3toR2(PHASE_ABC abc, PHASE_DQ *dq, double theta);
extern void S3toS2(PHASE_ABC abc, PHASE_ALBE *albe);
extern void S2toR2(PHASE_ALBE albe, PHASE_DQ *dq, double cosIn, double sinIn);

/* Backward conversion */  
extern void R2toS3(PHASE_DQ dq, PHASE_ABC *abc, double theta);
extern void S2toS3(PHASE_ALBE albe, PHASE_ABC *abc);
extern void R2toS2(PHASE_DQ dq, PHASE_ALBE *albe, double cosIn, double sinIn);

/* calculate lamdar */  
extern double lamdarCal(double lamdar, double ism);

/* calculate position and speed */  
extern double wrCal_M();
extern double positonCal(double wr, double lamdar, double ist, double theta);

/* PI module */  
extern double PImodule(double Kp, double Ki, double uk, double err, double *lasterr, double Uplim, double Downlim);
extern double Integrator(double paramin, double sum, double ts);

/* SVM */  
extern void positionSVM();
extern void ualbeSVM(double Ual, double Ube, double Ud, Uint16 invprd1, Uint16 invprd2, Uint16 *Tinv1, Uint16 *Tinv2);

/* Auxiliary Function */
extern double RAMP(double ramp, double initial, double increment, double Hlimit, double Llimit);
extern double roundn(double input, int _digit);
