#include "unusual.h"
uint8 count_loopR=0,count_loopL=0,count_loopr=0,count_loopl=0;
int16 vall,valr,RV,LV,RH,LH,ftmave,ideal_velocity;
float Biasd;
float divertion,velocity,velocity_angle,dif_velocity_L,dif_velocity_R;
int16 reset1,reset2,reset3,reset4;
int32 freql,freqr;
uint8 div_mode,chuku;
/*--------------出入环---------------*/
static float Integral_speed,Integral_anglespeed;
void Loop_trial_R()//右大环
{
    if(count_loopR==1||count_loopR==2)
    {
      Integral_anglespeed+=(mpu_gyro_z+11.5);
    }
    if(LV<70&&RV<70&&count_loopR==1)
    {
      Integral_speed+=ftmave;
      //DifferencialFTM_PID(200*ftmave/40);
      Biasd-=740*ftmave/40;
    }
    if(Integral_speed>8*100&&count_loopR==1)
    {
      gpio_set(D15, 0);
      count_loopR++;
      Integral_speed=0;
    }
    if(count_loopR==2&&Integral_anglespeed<-(207*100))
    {
      count_loopR++;   
      Integral_anglespeed=0;
    }
    if(count_loopR==3)
    {
      Integral_speed+=ftmave;
      //DifferencialFTM_PID(Set1*ftmave/40);
      Biasd+=960*ftmave/40;
    }
    if(count_loopR==3&&Integral_speed>20*100)
    {
      count_loopR++;
      Integral_speed=0;
    }
    if(count_loopR==4)
    {
      Integral_speed+=ftmave;
      Biasd=0;
    }
    if(count_loopR==4&&Integral_speed>0*100)
    {
      count_loopR=-1; 
      Integral_speed=0;
    }
}
void Loop_trial_l()//左小环
{
  if(count_loopl==1||count_loopl==2)
    {
      Integral_anglespeed+=(mpu_gyro_z+11.5);
    }
  if(count_loopl==1)
    {
      Integral_speed+=ftmave;
      Biasd+=1700*ftmave/40;
    }
  if(Integral_speed>7*100&&count_loopl==1)
    {
      gpio_set(D15, 0);
      count_loopl++;
      Integral_speed=0;
    }
  if(count_loopl==2&&Integral_anglespeed>(340*100))
    {
      count_loopl++;   
      Integral_anglespeed=0;
    }
  if(count_loopl==3)
    {
      Integral_speed+=ftmave;
      Biasd+=1000*ftmave/40;
    }
  if(count_loopl==3&&Integral_speed>5*100)
    {
      count_loopl++;
      Integral_speed=0;
    }
  if(count_loopl==4)
    {
      Integral_speed+=ftmave;
      Biasd-=500*ftmave/40;
    }
  if(count_loopl==4&&Integral_speed>10*100)
  {
      count_loopl=-1; 
      Integral_speed=0;
  }
}
void Loop_trial_L()//左大环
{
    if(count_loopL==1||count_loopL==2||count_loopL==3)
    {
      Integral_anglespeed+=(mpu_gyro_z+11.5);
    }
    if(count_loopL==1)
    {
      Integral_speed+=ftmave;
      Biasd+=1100*ftmave/40;
    }
    if(Integral_speed>45*10&&count_loopL==1)
    {
      count_loopL++;
      Integral_speed=0;
    }
    if(count_loopL==2)
    {
      Integral_speed+=ftmave;
      Biasd+=1500*ftmave/40;
    }
    if(Integral_speed>8*100&&count_loopL==2)
    {
      gpio_set(D15, 1);
      count_loopL++;
      Integral_speed=0;
    }
    if(count_loopL==3&&Integral_anglespeed>(180*100))
    {
      gpio_set(D15, 0);
      count_loopL++;   
      Integral_anglespeed=0;
    }
    if(count_loopL==4)
    {
      Integral_speed+=ftmave;
      Biasd+=600*ftmave/40;
    }
    if(count_loopL==4&&Integral_speed>5*100)
    {
      count_loopL++;
      Integral_speed=0;
    }
    if(count_loopL==5)
    {
      Integral_speed+=ftmave;
      Biasd-=500*ftmave/40;
    }
    if(count_loopL==5&&Integral_speed>10*100)
    {
      count_loopL=-1; 
      Integral_speed=0;
    }
}
void Loop_trial_r()//右小环
{
  if(count_loopr==1||count_loopr==2)
    {
      Integral_anglespeed+=(mpu_gyro_z+11.5);
    }
  if(count_loopr==1)
    {
      Integral_speed+=ftmave;
      Biasd-=1300*ftmave/40;
    }
  if(Integral_speed>7*100&&count_loopr==1)
    {
      count_loopr++;
      Integral_speed=0;
    }
  if(count_loopr==2&&Integral_anglespeed<(-180*100))
    {
      gpio_set(D15, 0);
      count_loopr++;   
      Integral_anglespeed=0;
    }
  if(count_loopr==3)
    {
      Integral_speed+=ftmave;
      Biasd-=600*ftmave/40;
    }
  if(count_loopr==3&&Integral_speed>3*100)
    {
      count_loopr++;
      Integral_speed=0;
    }
  if(count_loopr==4)
    {
      Integral_speed+=ftmave;
      Biasd+=1000*ftmave/40;
    }
  if(count_loopr==4&&Integral_speed>10*100)
  {
      count_loopr=-1; 
      Integral_speed=0;
  }
}
/*---------------出库---------------*/
static int chuku_angle,chuku_distance;
void CHUKU(void)
{
      gpio_set(D15, 1);
      chuku_distance+=ftmave;
      chuku_angle+=(mpu_gyro_z+4);
      freql=1*velocity;
      freqr=1*velocity;
      ideal_velocity=150;
      freql+=dif_velocity_L;
      freqr-=dif_velocity_R;
      div_mode=0;
      if(chuku==2&&chuku_distance<reset1)
      {
        Biasd=0;
      }
      if(chuku==2&&chuku_distance>=reset1)
        chuku--;
      if(chuku==1&&chuku_angle<reset2*100)
      {       
        Biasd=8;
      }
      if(chuku==1&&chuku_angle>=reset2*100)
      {
        gpio_set(D15, 0);
        chuku--;
      }
}