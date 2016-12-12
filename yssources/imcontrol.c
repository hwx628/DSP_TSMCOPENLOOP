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
double Ud;
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
double spd_req = 300;  // 转速设定

/* PI 变量 */
double idlasterr = 0;
double iqlasterr = 0;
double spdlasterr = 0;

/*==============================================================================
=========================== Coordinate Transform ===============================
==============================================================================*/

/******************************************************************************
@brief   3s/2r 坐标变换

@param   abc -- 三相向量
         dq -- 两相旋转向量（*指针传递*）
         theta -- 合成向量角度

@return  N/A
******************************************************************************/
void S3toR2(PHASE_ABC abc, PHASE_DQ *dq, double theta)
{
  // sqrt(2) = 1.414213562, 1.0/6.0*pi = 0.52359878
  dq->d = 1.414213562 * (cos(theta - 0.52359878) * abc.a + sin(theta) * abc.b);
  dq->q = -1.414213562 * (sin(theta - 0.52359878) * abc.a - cos(theta) * abc.b);
}

/******************************************************************************
@brief   3s/2s 坐标变换

@param   abc -- 三相向量
         albe -- 两相静止向量（*指针传递*）

@return  N/A
******************************************************************************/
void S3toS2(PHASE_ABC abc, PHASE_ALBE *albe)
{
  // sqrt(3.0/2.0) = 1.22474487, 1.0/sqrt(2) = 0.7071067812, sqrt(2) = 1.414213562
  albe->al = 1.22474487 * abc.a;
  albe->be = 0.7071067812 * abc.a + 1.414213562 * abc.b;
}

/******************************************************************************
@brief   2s/2r 坐标变换

@param   albe -- 两相静止向量
         dq -- 两相旋转向量（*指针传递*）
         theta -- 合成向量角度

@return  N/A
******************************************************************************/
void S2toR2(PHASE_ALBE albe, PHASE_DQ *dq, double cosIn, double sinIn)
{
  dq->d = cosIn * albe.al + sinIn * albe.be;
  dq->q = -sinIn * albe.al + cosIn * albe.be;
}

/******************************************************************************
@brief   2r/3s 坐标变换

@param   dq -- 两相旋转向量
         abc -- 三相向量（*指针传递*）
         theta -- 合成向量角度

@return  N/A
******************************************************************************/
void R2toS3(PHASE_DQ dq, PHASE_ABC *abc, double theta)
{
  // sqrt(2.0/3.0) = 0.81649658, 2.0/3.0*pi = 2.094395102
  abc->a = 0.81649658 * (cos(theta) * dq.d - sin(theta) *dq.q);
  abc->b = 0.81649658 * (cos(theta - 2.094395102) * dq.d - sin(theta - 2.094395102) *dq.q);
  abc->c = 0.81649658 * (cos(theta + 2.094395102) * dq.d - sin(theta + 2.094395102) *dq.q);
}

/******************************************************************************
@brief   2s/3s 坐标变换

@param   albe -- 两相静止向量
         abc -- 三相向量（*指针传递*）

@return  N/A
******************************************************************************/
void S2toS3(PHASE_ALBE albe, PHASE_ABC *abc)
{
  // sqrt(2.0/3.0) = 0.81649658, sqrt(3)/2.0 = 0.8660254
  abc->a = 0.81649658 * albe.al;
  abc->b = -0.40824829 * albe.al + 0.70710678 * albe.be; // abc->b = sqrt(2.0/3.0) * (-0.5 * albe.al + sqrt(3)/2.0 * albe.be)
  abc->c = -0.40824829 * albe.al - 0.70710678 * albe.be; // sqrt(2.0/3.0) * (-0.5 * albe.al - sqrt(3)/2.0 * albe.be);
}

/******************************************************************************
@brief   2r/2s 坐标变换

@param   dq -- 两相旋转向量
         albe -- 两相静止向量（*指针传递*）
         theta -- 合成向量角度

@return  N/A
******************************************************************************/
void R2toS2(PHASE_DQ dq, PHASE_ALBE *albe, double cosIn, double sinIn)
{
  albe->al = cosIn * dq.d - sinIn * dq.q;
  albe->be = sinIn * dq.d + cosIn * dq.q;
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
  
  switch (sector)
  {
  case 1:
    {      
      if (d0 < 0 || (invprd1 * d0 < zerolimit))
      {
    	  double dmt = dm / (dm + dn);
    	  Tinv1[0] = zerolimit;
    	  Tinv1[1] = zerolimit + (int)((invprd1 - 2*zerolimit) * dmt);
    	  Tinv1[2] = invprd1 - zerolimit;
      }
      else
      {
          Tinv1[0] = (int)(invprd1 * (d0));
          Tinv1[1] = (int)(invprd1 * (dm + d0));
          Tinv1[2] = (int)(invprd1 * (dm + dn + d0));
      }

      if (d0 < 0 || (invprd2 * d0 < zerolimit))
      {
    	  double dnt = dn / (dm + dn);
    	  Tinv2[0] = invprd2 - zerolimit;
    	  Tinv2[1] = zerolimit + (int)((invprd2 - 2*zerolimit) * dnt);
    	  Tinv2[0] = zerolimit;
      }
      else
      {
          Tinv2[0] = (int)(invprd2 * (dm + dn + d0));
          Tinv2[1] = (int)(invprd2 * (dn + d0));
          Tinv2[2] = (int)(invprd2 * (d0));
      }

      break;
    }
  case 2:
    {
      if (d0 < 0 || (invprd1 * d0 < zerolimit))
      {
    	  double dnt = dn / (dm + dn);
    	  Tinv1[0] = zerolimit + (int)((invprd1 - 2*zerolimit) * dnt);
      	  Tinv1[1] = zerolimit;
      	  Tinv1[2] = invprd1 - zerolimit;
      }
      else
      {
          Tinv1[0] = (int)(invprd1 * (dn + d0));
          Tinv1[1] = (int)(invprd1 * (d0));
          Tinv1[2] = (int)(invprd1 * (dm + dn + d0));
      }

      if (d0 < 0 || (invprd2 * d0 < zerolimit))
      {
    	  double dmt = dm / (dm + dn);
    	  Tinv2[0] = zerolimit + (int)((invprd2 - 2*zerolimit) * dmt);
    	  Tinv2[1] = invprd2 - zerolimit;
    	  Tinv2[2] = zerolimit;
      }
      else
      {
          Tinv2[0] = (int)(invprd2 * (dm + d0));
          Tinv2[1] = (int)(invprd2 * (dm + dn + d0));
          Tinv2[2] = (int)(invprd2 * (d0));
      }

      break;
    }
  case 3:
    {
      if (d0 < 0 || (invprd1 * d0 < zerolimit))
      {
    	  double dmt = dm / (dm + dn);
      	  Tinv1[0] = invprd1 - zerolimit;
      	  Tinv1[1] = zerolimit;
      	  Tinv1[2] = zerolimit + (int)((invprd1 - 2*zerolimit) * dmt);
      }
      else
      {
          Tinv1[0] = (int)(invprd1 * (dm + dn + d0));
          Tinv1[1] = (int)(invprd1 * (d0));
          Tinv1[2] = (int)(invprd1 * (dm + d0));
      }

      if (d0 < 0 || (invprd2 * d0 < zerolimit))
      {
      	  double dnt = dn / (dm + dn);
      	  Tinv2[0] = zerolimit;
      	  Tinv2[1] = invprd2 - zerolimit;
      	  Tinv2[2] = zerolimit + (int)((invprd2 - 2*zerolimit) * dnt);
      }
      else
      {
          Tinv2[0] = (int)(invprd2 * (d0));
          Tinv2[1] = (int)(invprd2 * (dm + dn + d0));
          Tinv2[2] = (int)(invprd2 * (dn + d0));
      }

      break;
    }
  case 4:
    {
      if (d0 < 0 || (invprd1 * d0 < zerolimit))
      {
      	  double dnt = dn / (dm + dn);
      	  Tinv1[0] = invprd1 - zerolimit;
      	  Tinv1[1] = zerolimit + (int)((invprd1 - 2*zerolimit) * dnt);
      	  Tinv1[2] = zerolimit;
      }
      else
      {
          Tinv1[0] = (int)(invprd1 * (dm + dn + d0));
          Tinv1[1] = (int)(invprd1 * (dn + d0));
          Tinv1[2] = (int)(invprd1 * (d0));
      }

      if (d0 < 0 || (invprd2 * d0 < zerolimit))
      {
    	  double dmt = dm / (dm + dn);
      	  Tinv2[0] = zerolimit;
      	  Tinv2[1] = zerolimit + (int)((invprd2 - 2*zerolimit) * dmt);
      	  Tinv2[2] = invprd2 - zerolimit;
      }
      else
      {
          Tinv2[0] = (int)(invprd2 * (d0));
          Tinv2[1] = (int)(invprd2 * (dm + d0));
          Tinv2[2] = (int)(invprd2 * (dm + dn + d0));
      }

      break;
    }
  case 5:
    {
      if (d0 < 0 || (invprd1 * d0 < zerolimit))
      {
    	  double dmt = dm / (dm + dn);
    	  Tinv1[0] = zerolimit + (int)((invprd1 - 2*zerolimit) * dmt);
    	  Tinv1[1] = invprd1 - zerolimit;
    	  Tinv1[2] = zerolimit;
      }
      else
      {
          Tinv1[0] = (int)(invprd1 * (dm + d0));
          Tinv1[1] = (int)(invprd1 * (dm + dn + d0));
          Tinv1[2] = (int)(invprd1 * (d0));
      }

      if (d0 < 0 || (invprd2 * d0 < zerolimit))
      {
    	  double dnt = dn / (dm + dn);
    	  Tinv2[0] = zerolimit + (int)((invprd2 - 2*zerolimit) * dnt);
    	  Tinv2[1] = zerolimit;
    	  Tinv2[2] = invprd2 - zerolimit;
      }
      else
      {
          Tinv2[0] = (int)(invprd2 * (dn + d0));
          Tinv2[1] = (int)(invprd2 * (d0));
          Tinv2[2] = (int)(invprd2 * (dm + dn + d0));
      }

      break;
    }
  case 6:
    {
      if (d0 < 0 || (invprd1 * d0 < zerolimit))
      {
    	  double dnt = dn / (dm + dn);
    	  Tinv1[0] = zerolimit;
    	  Tinv1[1] = invprd1 - zerolimit;
    	  Tinv1[2] = zerolimit + (int)((invprd1 - 2*zerolimit) * dnt);
      }
      else
      {
          Tinv1[0] = (int)(invprd1 * (d0));
          Tinv1[1] = (int)(invprd1 * (dm + dn + d0));
          Tinv1[2] = (int)(invprd1 * (dn + d0));
      }

      if (d0 < 0 || (invprd2 * d0 < zerolimit))
      {
    	  double dmt = dm / (dm + dn);
    	  Tinv2[0] = invprd2 - zerolimit;
    	  Tinv2[1] = zerolimit;
    	  Tinv2[2] = zerolimit + (int)((invprd2 - 2*zerolimit) * dmt);
      }
      else
      {
          Tinv2[0] = (int)(invprd2 * (dm + dn + d0));
          Tinv2[1] = (int)(invprd2 * (d0));
          Tinv2[2] = (int)(invprd2 * (dm + d0));
      }

      break;
    }
  default:
    {
      Tinv1[0] = invprd1 + 1;
      Tinv1[1] = invprd1 + 1;
      Tinv1[2] = invprd1 + 1;
      Tinv2[0] = invprd2 + 1;
      Tinv2[1] = invprd2 + 1;
      Tinv2[2] = invprd2 + 1;
    }
  }
}

/******************************************************************************
@brief   RAMP -- 增量式斜坡函数

@param   ramp -- 斜率
         initial -- 应变量起始值
         increment -- 自变量增量
         Hlimit -- 上限
         Llimit -- 下限

@return  应变量终值
******************************************************************************/
double RAMP(double ramp, double initial, double increment, double Hlimit, double Llimit)
{
  double temp = ramp * increment + initial;
  if (temp > Hlimit)
    return Hlimit;
  else if (temp < Llimit)
    return Llimit;
  else
    return temp;
}

/******************************************************************************
@brief   roundn -- 有理数取指定位数

@param   input -- 输入
         digit -- 保留小数点后位数

@return  舍弃指定位数后的值
******************************************************************************/
double roundn(double input, int _digit)
{
  double temp;
  temp = input * _digit;
  temp = floor(temp);
  temp = temp / _digit;
  return temp;
}
