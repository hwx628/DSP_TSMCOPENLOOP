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
// IM parameters
#define Rs 0.4
#define Ls 0.289368
#define Rr 3.3278
#define Lr 0.289368
#define Lm 0.27325
#define Tr 0.0869557

// period
#define Ts 1e-4

// command values
#define nset 400
#define idset 1.5

// PI parameters
#define ud_Kp 6
#define ud_Ki 0
#define ud_Uplimit 60
#define ud_Downlimit -60

#define iqset_Kp1 2
#define iqset_Ki1 0
#define iqset_Kp2 1
#define iqset_Ki2 20
#define iqset_Uplimit 0
#define iqset_Downlimit 0

#define uq_Kp 3
#define uq_Ki 10
#define uq_Uplimit 60
#define uq_Downlimit -60

// auxiliary
#define pi 3.1415926
#define M 0.95  // modulation factor
#define Z 1024  // 光电码盘线数

/******************************************************************************
| variables
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

extern PHASE_ABC uabc;
extern PHASE_ALBE ualbe;
extern PHASE_ABC iabc;
extern PHASE_ALBE ialbe;
extern PHASE_DQ idq;
extern PHASE_ALBE ualbe_cmd;
extern PHASE_DQ udq_cmd;
extern double Ud;

extern double theta;
extern double lamdar;
extern double n;
extern double wr;
extern double iqset;

extern double ud_Isum;
extern double uq_Isum;
extern double iqset_Isum;
extern int period_count;

extern PHASE_ALBE lamdaralbe;
extern double anglek;
extern double ualsum;
extern double ubesum;
extern double ialsum;
extern double ibesum;

extern unsigned int cntFTM1;

/******************************************************************************
| local functions prototypes
|----------------------------------------------------------------------------*/
double roundn(double input, int digit);

/******************************************************************************
| exported functions
|----------------------------------------------------------------------------*/
/* Forward conversion */  
extern void S3toR2(PHASE_ABC *abc, PHASE_DQ *dq, double theta);
extern void S3toS2(PHASE_ABC *abc, PHASE_ALBE *albe);
extern void S2toR2(PHASE_ALBE *albe, PHASE_DQ *dq, double theta);

/* Backward conversion */  
extern void R2toS3(PHASE_DQ *dq, PHASE_ABC *abc, double theta);
extern void S2toS3(PHASE_ALBE *albe, PHASE_ABC *abc);
extern void R2toS2(PHASE_DQ *dq, PHASE_ALBE *albe, double theta);

/* calculate lamdar */  
extern double lamdarCal(double lamdar, double ism);
extern void lamdardqCal();
extern void lamdaralbeCal(PHASE_ALBE ualbe, PHASE_ALBE ialbe, double *ualsum, \
        double *ubesum, double *ialsum, double *ibesum, PHASE_ALBE *lamdaralbe);

/* calculate position and speed */  
extern double wrCal_M();
extern double wrCal_T();
extern double wrCal_MT();
extern double wrCal_lamdar(PHASE_ALBE *lamdaralbe, double *anglek, PHASE_ALBE ualbe, PHASE_ALBE ialbe, double ts);
extern double positonCal(double wr, double lamdar, double ist, double theta);

/* PI module */  
extern double PImodule(double Kp, double Ki, double uk, double err, double *lasterr, double Uplim, double Downlim);
extern double Integrator(double paramin, double sum, double ts);

/* SVM */  
extern void positionSVM();
extern void ualbeSVM(double Ual, double Ube, double Ud, Uint16 invprd1, Uint16 invprd2, Uint16 *Tinv1, Uint16 *Tinv2);
extern void udqSVM();
