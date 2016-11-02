/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "imcontrol.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/
double angle = 0;  // 向量与扇区间夹角
int period_count = 0;  // PWM中断次数
Uint16 sector = 0;  // SVM扇区

/******************************************************************************
| global variable definitions                          
|----------------------------------------------------------------------------*/
/* 观测值 */
  // 电压
double Ud = 40;
PHASE_ABC uabc = {0, 0, 0};
PHASE_ALBE ualbe = {0, 0};
PHASE_DQ udq = {0, 0};
  // 电流
PHASE_ABC iabc = {0, 0, 0};
PHASE_ALBE ialbe = {0, 0};
PHASE_DQ idq = {0, 0};
  // 磁链
double lamdar = 0;
PHASE_ALBE lamdaralbe = {0, 0};
double theta = 0;
  // 转速
double speed = 0;

/* 给定值 */
  // 电压
double u_cmd = 0;  // 线电压幅值
PHASE_ALBE ualbe_cmd = {0, 0};
PHASE_DQ udq_cmd = {0, 0};
  // 电流
PHASE_DQ idq_cmd = {0, 0};
  // 转速
double spd_cmd = 0;  // 转速给定
double spd_req = 450;  // 转速设定

/* PI 变量 */
double idlasterr = 0;
double iqlasterr = 0;
double spdlasterr = 0;

/******************************************************************************
@brief   Coordinate Transform
******************************************************************************/
void S3toR2(PHASE_ABC *abc, PHASE_DQ *dq, double theta)
{
  //dq->d = sqrt(2.0/3.0) * (cos(theta) * abc->a + cos(theta - 2.0/3.0*pi) * abc->b + cos(theta + 2.0/3.0*pi) * abc->c);
  //dq->q = -sqrt(2.0/3.0) * (sin(theta) * abc->a + sin(theta - 2.0/3.0*pi) * abc->b + sin(theta + 2.0/3.0*pi) * abc->c);
  dq->d = sqrt(2.0) * (cos(theta - 1.0/6.0*pi) * abc->a + sin(theta) * abc->b);
  dq->q = -sqrt(2.0) * (sin(theta - 1.0/6.0*pi) * abc->a - cos(theta) * abc->b);
}

void S3toS2(PHASE_ABC *abc, PHASE_ALBE *albe)
{
  //albe->al = sqrt(2.0/3.0) * (abc->a - 0.5 * abc->b - 0.5 * abc->c);
  //albe->be = sqrt(2.0/3.0) * (sqrt(3)/2.0 * abc->b - sqrt(3)/2.0 * abc->c);
  albe->al = sqrt(3.0/2.0) * abc->a;
  albe->be = 1.0/sqrt(2) * abc->a + sqrt(2) * abc->b;
}

void S2toR2(PHASE_ALBE *albe, PHASE_DQ *dq, double theta)
{
  dq->d = cos(theta) * albe->al + sin(theta) * albe->be;
  dq->q = -sin(theta) * albe->al + cos(theta) * albe->be;
}

void R2toS3(PHASE_DQ *dq, PHASE_ABC *abc, double theta)
{
  abc->a = sqrt(2.0/3.0) * (cos(theta) * dq->d - sin(theta) *dq->q);
  abc->b = sqrt(2.0/3.0) * (cos(theta - 2.0/3.0*pi) * dq->d - sin(theta - 2.0/3.0*pi) *dq->q);
  abc->c = sqrt(2.0/3.0) * (cos(theta + 2.0/3.0*pi) * dq->d - sin(theta + 2.0/3.0*pi) *dq->q);
}

void S2toS3(PHASE_ALBE *albe, PHASE_ABC *abc)
{
  abc->a = sqrt(2.0/3.0) * albe->al;
  abc->b = sqrt(2.0/3.0) * (-0.5 * albe->al + sqrt(3)/2.0 * albe->be);
  abc->c = sqrt(2.0/3.0) * (-0.5 * albe->al - sqrt(3)/2.0 * albe->be);
}

void R2toS2(PHASE_DQ *dq, PHASE_ALBE *albe, double theta)
{
  albe->al = cos(theta) * dq->d - sin(theta) * dq->q;
  albe->be = sin(theta) * dq->d + cos(theta) * dq->q;
}

/******************************************************************************
@brief   Rotor Flux Calculation
******************************************************************************/
/* calculate lamdar */  
double lamdarCal(double lamdar, double ism)
{
  return (1.0 - Ts/Tr) * lamdar + Lm*Ts/Tr * ism;
}

void lamdaralbeCal(PHASE_ALBE ualbe, PHASE_ALBE ialbe, double *ualsum, double *ubesum, double *ialsum, double *ibesum, PHASE_ALBE *lamdaralbe)
{
  double tempal, tempbe;
  tempal = Integrator(ualbe.al, *ualsum, Ts) - Rs * Integrator(ialbe.al, *ialsum, Ts) - (Ls*Lr/Lm - Lm) * ialbe.al;
  tempbe = Integrator(ualbe.be, *ubesum, Ts) - Rs * Integrator(ialbe.be, *ibesum, Ts) - (Ls*Lr/Lm - Lm) * ialbe.be;
  lamdaralbe->al = tempal * Lr/Lm;
  lamdaralbe->be = tempbe * Lr/Lm;
}

void lamdardqCal()
{
}

/******************************************************************************
@brief   Calculate Position and Speed 
******************************************************************************/
double wrCal_M()
{
  //unsigned int temp;
  //return 60.0 * (cntFTM1 - temp) / (Z * 0.001);
}

double wrCal_lamdar(PHASE_ALBE *lamdaralbe, double *anglek, PHASE_ALBE ualbe, PHASE_ALBE ialbe, double ts) // 未测试
{
  double angle = 0;
  double we =0, wsl = 0;
  
  lamdaralbeCal(ualbe, ialbe, &ualsum, &ubesum, &ialsum, &ibesum, lamdaralbe);
  if (lamdaralbe->al != 0)
  {    
    angle = atan(lamdaralbe->be / lamdaralbe->al);
    if (fabs(angle - *anglek) < 0.5 * pi)
      we = (angle - *anglek) / ts;
    else if (angle <= 0)
      we = (angle - *anglek + pi) / ts;
    else
      we = (angle - *anglek - pi);
    
    *anglek = angle;
    
    wsl = Lm/Tr * (lamdaralbe->be - lamdaralbe->al) / (pow(lamdaralbe->al, 2) + pow(lamdaralbe->be, 2));
  }
  else
  {
    we = 0;
    wsl = 0;
  }
  
  if (fabs(we - wsl) < 90)
    return we - wsl;
  else
    return 0;
}

double positonCal(double wr, double lamdar, double ist, double theta)
{
  double we = 0;  
  
  if (lamdar > 0.01)
    we = Lm/Tr * ist / lamdar + wr;
  else
    we = 0;
  
  return Integrator(we, theta, Ts);
} 

/******************************************************************************
@brief   PI Module 
******************************************************************************/
double PImodule(double Kp, double Ki, double inputk, double err, double *lasterr, double Uplim, double Downlim)
{
	inputk += Kp * (err - *lasterr) + Ki * Ts * err;
	*lasterr = err;

	if (inputk >= Downlim && inputk <= Uplim)
		return inputk;
	else if (inputk > Uplim)
		return Uplim;
	else
		return Downlim;
}

double Integrator(double paramin, double sum, double ts)
{
  return paramin * ts + sum;
}

/******************************************************************************
@brief   SVM 
******************************************************************************/ 
void positionSVM(unsigned int *Tinv)
{
    double Dm = 0, Dn = 0, D0 = 0;

    /* V/spd曲线计算电压给定值 */
    u_cmd = RAMP(VSpdramp, 0, spd_cmd, Voltlimit_H, Voltlimit_L);
  
    /* 扇区及夹角计算 */
    //theta += 2 * pi * (spd_cmd / 30.0) * 0.0001;  ==========================================================
    theta += 0.0000418879 * spd_cmd; // theta += 2 * pi * (spd_cmd / 30.0) * 0.0002;
    if (theta > 6.2831852)  // 2 * pi = 6.2831852
      theta -= 6.2831852;

    angle = fmod(theta, 1.047197551);  // 1/3.0 * pi = 1.047197551
    sector = (int)floor(theta * 0.9549296586) + 1;  // 1 / (1/3.0 * pi) = 0.9549296586

    /* 占空比计算 */
    Dm = u_cmd / Ud * sin(1.047197551 - angle);  // 1/3.0 * pi = 1.047197551
    Dn = u_cmd / Ud * sin(angle);
    D0 = (1 - Dm - Dn) * 0.5;
    Dm = roundn(Dm, digit);
    Dn = roundn(Dn, digit);
    D0 = roundn(D0, digit);
    if (D0 < 0) D0 = 0;
  
    /* 三相PWM比较值计算 */
    switch (sector)
    {
    case 1:
      Tinv[0] = (int)(period * (Dm + Dn + D0));
      Tinv[1] = (int)(period * (D0 + Dn));
      Tinv[2] = (int)(period * (D0));
      break;
    case 2:
      Tinv[0] = (int)(period * (Dm + D0));
      Tinv[1] = (int)(period * (Dm + Dn + D0));
      Tinv[2] = (int)(period * (D0));
      break;
    case 3:
      Tinv[0] = (int)(period * (D0));
      Tinv[1] = (int)(period * (Dm + Dn + D0));
      Tinv[2] = (int)(period * (Dn + D0));
      break;
    case 4:
      Tinv[0] = (int)(period * (D0));
      Tinv[1] = (int)(period * (Dm + D0));
      Tinv[2] = (int)(period * (Dm + Dn + D0));
      break;
    case 5:
      Tinv[0] = (int)(period * (Dn + D0));
      Tinv[1] = (int)(period * (D0));
      Tinv[2] = (int)(period * (Dm + Dn + D0));
      break;
    case 6:
      Tinv[0] = (int)(period * (Dm + Dn + D0));
      Tinv[1] = (int)(period * (D0));
      Tinv[2] = (int)(period * (Dm + D0));
    }
}

void ualbeSVM(double Ual, double Ube, double Ud, Uint16 invprd1, Uint16 invprd2, Uint16 *Tinv1, Uint16 *Tinv2)
{
	double dm, dn, d0;
	double k = Ube / Ual;
	double reciUd = 1.0 / Ud;

	/* 扇区判断及占空比计算 */
	// sqrt(3) = 1.7320508, sqrt(3) / 2.0 = 0.8660254044
	// 1 / sqrt(3) = 0.57735027
	if (Ual > 0 && Ube >= 0 && k >= 0 && k < 1.7320508)
	{
	sector = 1;
	dm = 0.8660254044 * (Ual - Ube * 0.57735027) * reciUd;
	dn = Ube * reciUd;
	}
	else if (Ube > 0 && (k >= 1.7320508 || k < -1.7320508))
	{
	sector = 2;
	dm = 0.8660254044 * (Ual + Ube * 0.57735027) * reciUd;
	dn = 0.8660254044 * (-Ual + Ube * 0.57735027) * reciUd;
	}
	else if (Ual < 0 && Ube > 0 && k >= -1.7320508 && k < 0)
	{
	sector = 3;
	dm = Ube * reciUd;
	dn = 0.8660254044 * (-Ual - Ube * 0.57735027) * reciUd;
	}
	else if (Ual < 0 && Ube <= 0 && k >= 0 && k < 1.7320508)
	{
	sector = 4;
	dm = 0.8660254044 * (-Ual + Ube * 0.57735027) * reciUd;
	dn = -Ube * reciUd;
	}
	else if (Ube < 0 && (k >= 1.7320508 || k < -1.7320508))
	{
	sector = 5;
	dm = 0.8660254044 * (-Ual - Ube * 0.57735027) * reciUd;
	dn = 0.8660254044 * (Ual - Ube * 0.57735027) * reciUd;
	}
	else if (Ual > 0 && Ube < 0 && k >= -1.7320508 && k < 0)
	{
	sector = 6;
	dm = -Ube * reciUd;
	dn = 0.8660254044 * (Ual + Ube * 0.57735027) * reciUd;
	}
	else
	{
	sector = 1;
	dm = 0;
	dn = 0;
	}

	if (dm + dn >= 1)
	{
	double temp = dm / (dm + dn);
	dn = dn / (dm + dn);
	dm = temp;
	d0 = 0;
	}
	else
	d0 = 0.5 * (1 - dm - dn);

	/* 三相PWM比较值计算 */
	switch (sector)
	{
	case 1:
	{
	  Tinv[0] = (int)(period * (dm + dn + d0));
	  Tinv[1] = (int)(period * (dn + d0));
	  Tinv[2] = (int)(period * d0);
	  break;
	}
	case 2:
	{
	  Tinv[0] = (int)(period * (dm + d0));
	  Tinv[1] = (int)(period * (dm + dn + d0));
	  Tinv[2] = (int)(period * d0);
	  break;
	}
	case 3:
	{
	  Tinv[0] = (int)(period * (d0));
	  Tinv[1] = (int)(period * (dm + dn + d0));
	  Tinv[2] = (int)(period * (dn + d0));
	  break;
	}
	case 4:
	{
	  Tinv[0] = (int)(period * (d0));
	  Tinv[1] = (int)(period * (dm + d0));
	  Tinv[2] = (int)(period * (dm + dn + d0));
	  break;
	}
	case 5:
	{
	  Tinv[0] = (int)(period * (dn + d0));
	  Tinv[1] = (int)(period * (d0));
	  Tinv[2] = (int)(period * (dm + dn + d0));
	  break;
	}
	case 6:
	{
	  Tinv[0] = (int)(period * (dm + dn + d0));
	  Tinv[1] = (int)(period * (d0));
	  Tinv[2] = (int)(period * (dm + d0));
	  break;
	}
	default:
	{
	  Tinv[0] = period + 1;
	  Tinv[1] = period + 1;
	  Tinv[2] = period + 1;
	}
	}
}

void udqSVM()
{
}

/******************************************************************************
@brief   Auxiliary Function
******************************************************************************/
double roundn(double input, int digit)
{
  double temp;
  temp = input * pow(10, digit);
  temp = floor(temp);
  temp = temp / (pow(10, digit) * 1.0);
  return temp;
}
