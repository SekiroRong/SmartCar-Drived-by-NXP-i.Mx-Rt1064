/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		IAR 8.3 or MDK 5.28
 * @Target core		NXP RT1064DVL6A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 ********************************************************************************************************************/


//�����Ƽ�IO�鿴Projecct�ļ����µ�TXT�ı�


//���µĹ��̻��߹����ƶ���λ�����ִ�����²���
//��һ�� �ر��������д򿪵��ļ�
//�ڶ��� project  clean  �ȴ��·�����������

//���ش���ǰ������Լ�ʹ�õ��������ڹ���������������Ϊ�Լ���ʹ�õ�

#include "headfile.h"


#define targetangle 0
#define LV_MAX 4000
#define LV_MIN 0
#define RV_MAX 4000
#define RV_MIN 0
   
//����flash���һ������������������ͻ
//�洢������ô����һ��������ʼʹ��
#define EXAMPLE_FLASH_SECTOR        (FLASH_SECTOR_NUM-1)//�������ݴ�ȡ����
#define PARAMETER_FLASH_SECTOR        (FLASH_SECTOR_NUM-2)//������ȡ����

//��������������ҳ���
#define EXAMPLE_FLASH_SECTOR_PAGE_1   (0)
#define EXAMPLE_FLASH_SECTOR_PAGE_2   (1)
#define EXAMPLE_FLASH_SECTOR_PAGE_3   (2)
#define EXAMPLE_FLASH_SECTOR_PAGE_4   (3)
#define FLASH_SAVE_NUM  64//�궨����Ҫ�洢�����ĸ���    һ��ҳ���Ϊ256�ֽڣ�һ��������Ҫռ4���ֽڣ�����������Ϊ256/4=64
   
#define FLASH_TEMP_DISTANCE 25*60
#define FLASH_TEMP_DISTANCE2 2.0*512*3
uint8 status;

static uint8 chuhuan_mark;//������־λ
static uint16 chuhuan_distance;//��������
extern uint16 Zhi_velocity,Wan_velocity,Acceleration_Criteria,Chuhuan_distance;//��׼�ٶȺͻ�׼���ٶ�,����ƫб�����0.001��
extern uint16 Wan_P,Wan_D,Zhi_P,Zhi_D;//����PD
extern uint16 Cross_address;//ʮ����flash���λ��
extern uint16 Ruhuan_deviate_S,Ruhuan_deviate_M,Ruhuan_deviate_L,Chuhuan_deviate_S,Chuhuan_deviate_M,Chuhuan_deviate_L;//�뻷����ƫб��1000��
extern uint16 Ruhuan_record_S,Ruhuan_record_M,Ruhuan_record_L,Chuhuan_record_S,Chuhuan_record_M,Chuhuan_record_L;//��¼ģʽ�뻷������ǵ�1000��
extern uint16 Ruhuan_record_distance,Chuhuan_record_distance;//��¼ģʽ�뻷�������ƫ�Ƶľ���
extern uint16 Chuanqiu_distance;//����ǰ�ļ��پ���
static uint8 temp_count=0;
static float Integral_speed,Integral_anglespeed;
extern uint8 F1_mode;//F1ģʽ ���ã�����
static uint8 S_road_mark=0;//S���־λ
static float Apex_point=0;
extern int8 count_loopR,count_loopL,count_loopr,count_loopl,count_loopLM,count_loopRM;
int8 count_loopR=0,count_loopL=0,count_loopr=0,count_loopl=0,count_loopLM=0,count_loopRM=0;
extern int16 music_mark,record_mark,record_mark2;
static uint8 Direction=0,Direction_record=0;//0��ת1��ת
static uint16 record_address=0;
static uint32 record_write_buf1[FLASH_SAVE_NUM],record_write_buf2[FLASH_SAVE_NUM],record_write_buf3[FLASH_SAVE_NUM],record_time[FLASH_SAVE_NUM],record_write_buf4[FLASH_SAVE_NUM];
static float record_float_buf3[FLASH_SAVE_NUM],record_write_temp3=0;
static uint32 record_write_temp1=0,record_write_temp2=0,record_time_temp=0;
extern uint32 record_read_buf1[FLASH_SAVE_NUM],record_read_buf2[FLASH_SAVE_NUM],record_read_buf3[FLASH_SAVE_NUM],record_read_buf4[FLASH_SAVE_NUM];
static int32 location=0;
//buf1���˱��������־��룬buf2��1���0ֱ��3�µ�4�Ҵ�5���6��С��7��С��8���л�9���л���buf3�������ʣ�buf4Apex��
extern uint32 record_parameter_save1[FLASH_SAVE_NUM];
extern uint8 emergency_stop;
extern uint8 example_rx_buffer;
extern lpuart_transfer_t   example_receivexfer;
extern lpuart_handle_t     example_g_lpuartHandle;
extern uint16 distance;	//���������ն˲�õľ���ֵ
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
extern int16 ftmave,ideal_velocity;
extern int16 FTM_ave;
extern int16 vall,valr;
extern float RV,LV,RH,LH,M1,M2;
extern float LV_normal,RV_normal;
extern int32 freql,freqr;
extern float Integral_angle,angle;
extern float divertion,velocity,velocity_angle,dif_velocity_L,dif_velocity_R;
float dif_velocity_L,dif_velocity_R;
extern float Biasd;
extern uint8 txt[30];
extern int16 reset1,reset2,reset3,reset4;
extern uint8 MODE;
static uint8 Loop_turn=0;
float addr[8];//ʾ����
extern int8 chuku_direction,chuku;//���ⷽ��0��1��-1�޳���
static uint8 div_mode=0,diversion_mode=0,emergency_stop=0,offroad=0;//���ٻ�ģʽ������ģʽ������ͣ����־�������־
extern uint16 count;
static float road_check;  //�ж�ֱ�����
static int32 brake_check; //ֱ��һ�ξ���ɲ��
static uint8 brake_activated=0;
static uint8 shake_judge_mark=0;//�������ж�
static int8 shake_count=0;//��������
static uint32 shake_time=0,shake_distance=0;
static void Loop_trial_R();
static void Loop_trial_l();
static void Loop_trial_r();
static void Loop_trial_L();
static void Loop_trial_LM();
static void Loop_trial_RM();
static void shakejudge_inductance(void);
static void shakejudge_gyro(void);
static void CHUKU(int8);


void trinity()//����һ��Ϊ��
{
    int i;
    for(i=0;i<record_address;i++)
    {
    if(record_write_buf3[i]>-75&&record_write_buf3[i]<0)
      record_write_buf3[i]=-60;
    else if(record_write_buf3[i]<75&&record_write_buf3[i]>0)
      record_write_buf3[i]=60;
    else if(record_write_buf3[i]>-95&&record_write_buf3[i]<-75)
      record_write_buf3[i]=-80;
    else if(record_write_buf3[i]<95&&record_write_buf3[i]>75)
      record_write_buf3[i]=80;
    else if(record_write_buf3[i]<-95)
      record_write_buf3[i]=-100;
    else if(record_write_buf3[i]>95)
      record_write_buf3[i]=100;
    }
}
void Record_road(void)
{
  if(record_mark==1)
  {
    if(flash_check(EXAMPLE_FLASH_SECTOR,EXAMPLE_FLASH_SECTOR_PAGE_1))//У�鵱ǰ ��������ҳ�Ƿ������ݣ�����������������������
    {
        status = flash_erase_sector(EXAMPLE_FLASH_SECTOR);//������������������Ѿ�������������������֮������ٴ�д���µ�����
        if(status)  while(1);//����ʧ��
    }
    record_mark=-1;
  }
  if(record_mark==-2)
  {
    status = flash_page_program(EXAMPLE_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_1, record_write_buf1, FLASH_SAVE_NUM);
    if(status)  while(1);//д��ʧ��
    status = flash_page_program(EXAMPLE_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_2, record_write_buf2, FLASH_SAVE_NUM);
    if(status)  while(1);//д��ʧ��
    status = flash_page_program(EXAMPLE_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_3, record_write_buf3, FLASH_SAVE_NUM);
    if(status)  while(1);//д��ʧ��
    status = flash_page_program(EXAMPLE_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_4, record_write_buf4, FLASH_SAVE_NUM);
    if(status)  while(1);//д��ʧ��
    record_mark=-3;
  }
}
void Record_parameter(void)
{
  if(record_mark2==1)
  {
    if(flash_check(PARAMETER_FLASH_SECTOR,EXAMPLE_FLASH_SECTOR_PAGE_1))//У�鵱ǰ ��������ҳ�Ƿ������ݣ�����������������������
    {
        status = flash_erase_sector(PARAMETER_FLASH_SECTOR);//������������������Ѿ�������������������֮������ٴ�д���µ�����
        if(status)  while(1);//����ʧ��
    }
    record_mark2=-1;
  }
  if(record_mark2==-2)
  {
    status = flash_page_program(PARAMETER_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_1, record_parameter_save1, FLASH_SAVE_NUM);
    if(status)  while(1);//д��ʧ��
    record_mark2=-3;
  }
}
/*��ȡģ��--------------------------------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------------------------------------*/
/*float newtonSqrt(float c){  //��ɫ������ ���ܿ�0�ĸ���
  int i;  
  float x;  
  x=c;  
  for(i=1;i<=10;i++)  
    x=(x+c/x)/2;  
  return x;  
}*/
float Fsqrt(float x)   //������������

{
    if (x>=0){

		float xhalf = 0.5f * x;

		int I = *(int*)&x;         // evil floating point bit level hacking

		//I = 0x5f3759df - (I >> 1);  // what the fuck?

		I  = 0X5F3504F3 - ( I >> 1 ); //���ȸ���

		x = *(float*)&I;

		x = x*(1.5f-(xhalf*x*x));

		return 1/x;
    }

    else{

		x = -x;

		float xhalf = 0.5f * x;

		int I = *(int*)&x;         // evil floating point bit level hacking

		//I = 0x5f3759df - (I >> 1);  // what the fuck?

		I  = 0X5F3504F3 - ( I >> 1 ); //���ȸ���

		x = *(float*)&I;

		x = x*(1.5f-(xhalf*x*x));

		return -1/x;

    }

} //end of function Fsqrt
void Dutycontrol(void)       //�������
{
  if(freqr>9999)
    {
      pwm_duty(PWM1_MODULE3_CHB_D1,0);
      pwm_duty(PWM1_MODULE3_CHA_D0,9999);
    }
    else if(freqr>0){
      pwm_duty(PWM1_MODULE3_CHB_D1,0);
      pwm_duty(PWM1_MODULE3_CHA_D0,freqr);
    }
    else if(freqr<-9999)
    {
      pwm_duty(PWM1_MODULE3_CHA_D0,0);
      pwm_duty(PWM1_MODULE3_CHB_D1,9999);
    }
    else
    {
      pwm_duty(PWM1_MODULE3_CHA_D0,0);
      pwm_duty(PWM1_MODULE3_CHB_D1,-freqr);
    }
    if(freql>9999)
    {
      pwm_duty(PWM1_MODULE0_CHA_D12,0);
      pwm_duty(PWM1_MODULE0_CHB_D13,9999);
    }
    else if(freql>0)
    {
      pwm_duty(PWM1_MODULE0_CHA_D12,0);
      pwm_duty(PWM1_MODULE0_CHB_D13,freql);
    }
    else if(freql<-9999)
    {
      pwm_duty(PWM1_MODULE0_CHB_D13,0);
      pwm_duty(PWM1_MODULE0_CHA_D12,9999);
    }
    else
    {
      pwm_duty(PWM1_MODULE0_CHB_D13,0);
      pwm_duty(PWM1_MODULE0_CHA_D12,-freql);
    }
    /*
    if(freql>5000&&freqr>5000)
    {
      ftm_pwm_duty(FTM0, FTM_CH2,0);
      ftm_pwm_duty(FTM0, FTM_CH3,0);
      ftm_pwm_duty(FTM0, FTM_CH4,0);
      ftm_pwm_duty(FTM0, FTM_CH7,0);
    }*/
    if((LH<10&&RH<10)&&chuku==0)
    {
      pwm_duty(PWM1_MODULE3_CHA_D0,0);
      pwm_duty(PWM1_MODULE3_CHB_D1,0);
      pwm_duty(PWM1_MODULE0_CHA_D12,0);
      pwm_duty(PWM1_MODULE0_CHB_D13,0);
      /*if(offroad==0)
      {
        music_mark=1;
        pwm_init(PWM1_MODULE1_CHB_D15, 500, 0);//������
        count=0;
        offroad++;
      }*/
      if(record_mark==-1)
      {
        record_mark=-2;
        trinity();
      }
    }
    if(emergency_stop==2)
    {
      pwm_duty(PWM1_MODULE3_CHA_D0,0);
      pwm_duty(PWM1_MODULE3_CHB_D1,0);
      pwm_duty(PWM1_MODULE0_CHA_D12,0);
      pwm_duty(PWM1_MODULE0_CHB_D13,0);
    }
}
static int16 ftm_filter[10];
void FTMread()
{
    uint8 i;
    vall = qtimer_quad_get(QTIMER_1,QTIMER1_TIMER0_C0);
    valr = qtimer_quad_get(QTIMER_1,QTIMER1_TIMER2_C2);             //��ȡFTM �������� ��������(������ʾ������)
    ftmave=(valr-vall)/2;
    if(ftmave<0)
      ftmave=0;
    if(chuku==0)
    {
      FTM_ave=0;
      for(i=0;i<9;i++)
      {
        ftm_filter[i]=ftm_filter[i+1];
        FTM_ave+=ftm_filter[i];
      }
      ftm_filter[9]=ftmave;
      FTM_ave+=ftm_filter[9];
      FTM_ave/=10;
    }
    qtimer_quad_clear(QTIMER_1,QTIMER1_TIMER0_C0);
    qtimer_quad_clear(QTIMER_1,QTIMER1_TIMER2_C2);
}
void Normalization()  //��һ������
{
  LV_normal=(LV-LV_MIN)*4000/(LV_MAX-LV_MIN);
  RV_normal=(RV-RV_MIN)*4000/(RV_MAX-RV_MIN);
}
void Capacityread()
{  
    LH = adc_mean_filter(ADC_1,ADC1_CH3_B14,5);
    LV = adc_mean_filter(ADC_1,ADC1_CH4_B15,5);
    M1 = adc_mean_filter(ADC_1,ADC1_CH5_B16,5);
    M2 = adc_mean_filter(ADC_1,ADC1_CH6_B17,5);
    RV = adc_mean_filter(ADC_1,ADC1_CH7_B18,5);
    RH = adc_mean_filter(ADC_1,ADC1_CH8_B19,5);
    //Biasd=13.3*(1.0*(newtonSqrt(LH)-newtonSqrt(RH))/(LH+RH));
    //Biasd=(LH-RH)/(LH+RH);
    Biasd=13.3*(1.0*(Fsqrt(LH)-Fsqrt(RH))/(LH+RH));
    if(Biasd>=(5*0.001*13.3)||Biasd<=-(5*0.001*13.3))
    {
      //shake_judge_mark=1;
      road_check=50;
      //road_check++;
    }
    else if(Biasd>=(2*0.001*13.3)||Biasd<=-(2*0.001*13.3))
    {
        //shake_judge_mark=1;
        road_check+=0.25;
    }
    else
    {
      //shake_judge_mark=1;
      road_check-=1;
      if(road_check<=0)
        road_check=0;
    }
    Normalization();
}
static float Gyro_z[50];
static float Gyro_z_ave;
static float angular_velocity;
void Read6050()
{
    uint8 i;
    get_accdata();
    get_gyro();
    angular_velocity=2000.0*(mpu_gyro_x+13)/32768;
    if(chuku==0)
    {
      Gyro_z_ave=0;
      for(i=0;i<49;i++)
      {
        Gyro_z[i]=Gyro_z[i+1];
        Gyro_z_ave+=Gyro_z[i];
      }
      Gyro_z[49]=mpu_gyro_z;
      Gyro_z_ave+=Gyro_z[49];
      Gyro_z_ave/=50;
      if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13))
        shake_judge_mark=0;
      else
        shake_judge_mark=1;
    }
      
}
static float angle_bias,angle_bias_last;
void GetAngle()
{
    Read6050();
    angle=atan2(mpu_acc_z,mpu_acc_x)*57.3;
}
void GetI_Angle()             //�Ƕȼ���                                    //Ī�м��˲�
{  
    GetAngle();
    Integral_angle+=(mpu_gyro_y+6.5+(angle-Integral_angle)/0.23)*0.0016;    //�˲���ĽǶ�
    angle_bias=(-17.1-Integral_angle)*0.8+angle_bias_last*0.2;       //ֱ�����ǶȲ�
    angle_bias_last=angle_bias;
}
void Balance_mzj()           //ֱ��PID
{  
   GetI_Angle() ;
   velocity_angle=-230*(angle_bias+targetangle)+6*(mpu_gyro_y+6.5);
}
/*PIDģ��----------------------------------------------------------------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static float Bias,Last_Bias,Integral_Bias,Last_Biasd;
int Diversion_PD()//����
{
  float Pwmd;
  if(Biasd<(0.1*0.001*13.3) && Biasd>-(0.1*0.001*13.3))
    Biasd=0;
  switch(diversion_mode)
  {
    case 0:Pwmd=-0.1*Zhi_P*10*(10)*Biasd-0*0.015*(mpu_gyro_z+13)-Zhi_D*(Biasd-Last_Biasd);break; //165 2000 110 2200
    case 1:Pwmd=-0.1*Wan_P*10*(9)*Biasd-0*0.015*(mpu_gyro_z+13)-Wan_D*(Biasd-Last_Biasd);break;
    case 2:Pwmd=-0.1*200*10*(Fsqrt(20))*Biasd-0*0.015*(mpu_gyro_z+13)-2500*(Biasd-Last_Biasd);break;//����ר��
    case 3:Pwmd=-0.1*150*10*(10)*Biasd-0*0.015*(mpu_gyro_z+13)-2000*(Biasd-Last_Biasd);break;

    case 4:Pwmd=-0.1*75*10*(10)*Biasd-0*0.015*(mpu_gyro_z+13)-2000*(Biasd-Last_Biasd);break;//��¼ģʽר��

    default:Pwmd=-0.1*230*10*(Fsqrt(FTM_ave))*Biasd-0*0.015*(mpu_gyro_z+11.5)-4500*(Biasd-Last_Biasd);break;
  }
  //Pwmd=-0.1*220*ftmave*Biasd-0*0.015*(mpu_gyro_z+11.5)-4500*(Biasd-Last_Biasd);
  //Pwmd=-1*22*30*Biasd+15*0.015*(mpu_gyro_z+11.5);
  //Biasd=1.0*(LH-RH)/(LH+RH);
  //Pwmd=reset1*10*Biasd-reset2*16*0.015*(mpu_gyro_z+11.5);;
  //Oscilloscope();
  return -1*20*Pwmd;
  /*if(ftmave<=20 && ftmave>=-20 || abs(Biasd)<1)
  {
    return -1*20*Pwmd;
  }
  else
  {
  return -1*20*Pwmd;
  }*/
}
int FTM_PD(int FTMinput,int Target)//����PD�ٶȻ�
{
  float Pwm;
  Bias=FTMinput-Target;
  switch(div_mode)
  {
  case 0:Pwm=-10*(Bias-Last_Bias)-30*Bias;break;
  case 1:Pwm=-10*(Bias-Last_Bias)-30*Bias;break;
  default:Pwm=-10*(Bias-Last_Bias)-30*Bias;break;
  }
  Last_Bias=Bias;
  return Pwm;
}
static float Last_Pwm;
int FTM_PI(int FTMinput,int Target) //����PI�ٶȻ�
{
  float Pwm;
  Bias=FTMinput-Target;
  /*if(abs(Bias)>20)
    Pwm+=-15*Bias;
  else*/
  if(ftmave<=20)
  {
    Pwm=-2*Acceleration_Criteria*Bias;
  }
  else
  {
    switch(diversion_mode)
  {
  case 0:Pwm=-2.0*Acceleration_Criteria*Bias+300*0.01*Integral_Bias;break;
  case 1:Pwm=-1.2*Acceleration_Criteria*Bias+300*0.01*Integral_Bias;break;
  
  case 4:Pwm=-1*150*Bias+100*0.01*Integral_Bias;break;//��¼ģʽר��
  
  default:Pwm=-1*Acceleration_Criteria*Bias+0*0.01*Integral_Bias;break;
  }
    Integral_Bias-=Bias;
  }
  /*if(Pwm>8000)//�ٶȻ��޷�
    Pwm=8000;
  else if(Pwm<-8000)
    Pwm=-8000;*/
  /*if((Pwm-Last_Pwm)>1000)
    Pwm=Last_Pwm+1000;
  else if((Pwm-Last_Pwm)<-1000)
    Pwm=Last_Pwm-1000;
  Last_Pwm=Pwm;*/
  return Pwm;
}
void DifferencialFTM_PID()//���ٻ�
{
  //static float Last_Biasd;
  float Pwmf;
  if(Biasd<(0.1*0.001*13.3) && Biasd>-(0.1*0.001*13.3))
    Biasd=0;
  switch(div_mode)
  {
  case 0:Pwmf=-55*50*1*Biasd-0*0.001*(mpu_gyro_z+13)-1000*(Biasd-Last_Biasd);break;//���
  case 1:Pwmf=-30*FTM_ave*1*Biasd-0*0.001*(mpu_gyro_z+13)-4500*(Biasd-Last_Biasd);break;//ֱ��&����&ɲ��
  case 2:Pwmf=-130*FTM_ave*1*Biasd-0*0.001*(mpu_gyro_z+13)-0*(Biasd-Last_Biasd);break;//���ٹ���
  case 3:Pwmf=-55*70*1*Biasd-0*0.001*(mpu_gyro_z+13)-1000*(Biasd-Last_Biasd);break;//���
  default:Pwmf=-30*FTM_ave*1*Biasd-0*0.001*(mpu_gyro_z+13)-4500*(Biasd-Last_Biasd);break;
  }
  Last_Biasd=Biasd;
  dif_velocity_L=FTM_PD((vall+ftmave),Pwmf);
  dif_velocity_R=FTM_PD((valr-ftmave),Pwmf);    
}
/*void DifferencialFTM_PID_new()
{
  float Pwm_new_l,Pwm_new_r,Biasl,Biasr;
  if(record_read_buf3[address]>50&&record_read_buf3[address]<100)
    record_read_buf3[address]=80;
  if(record_read_buf3[address]<-50&&record_read_buf3[address]>-100)
    record_read_buf3[address]=-80;
  Biasl=(48/record_read_buf3[address]+0.18)*record_read_buf3[address]/48*FTMave;
  Biasr=(48/record_read_buf3[address]-0.18)*record_read_buf3[address]/48*FTMave;
  
}*/
static int32 Brake_length,Brake_mark=0;
void Generalcontrol(void)//�ܿ� �ؼ�������
{
    if((vall-valr)/2<=8)
    {
      freql=0*velocity_angle+1*velocity;
      freqr=0*velocity_angle+1*velocity;
    }
    else
    {
      freql=0*velocity_angle+1*velocity;
      freqr=0*velocity_angle+1*velocity;
    }
    //if(div_mode!=2)
    //{
      //if(road_check>=80)
    /*----------------������ʻ---------------*/
    if(chuku==0&&record_mark==0)
    {
      location+=ftmave;
      if(record_read_buf1[record_address]==0&&record_read_buf1[record_address+1]==0&&record_read_buf1[record_address+2]==0)//������������0�����¼ģʽ��֤��ȫ
        record_mark=-3;
      if(record_read_buf2[record_address]!=0&&Brake_mark!=0)//��ֱ�����ٱ�־λ����
        Brake_mark=0;
      if(record_read_buf2[record_address]==4||record_read_buf2[record_address]==5||record_read_buf2[record_address]==6||record_read_buf2[record_address]==7||record_read_buf2[record_address]==8||record_read_buf2[record_address]==9)
      {
        Integral_anglespeed+=2000.0*(mpu_gyro_z+13)/32768;
      }
      if(chuhuan_mark==1)
      {
        chuhuan_distance+=ftmave;
      }
      if(chuhuan_distance>Chuhuan_distance*1000)
      {
        chuhuan_mark=0;
        chuhuan_distance=0;
      }
      if(record_read_buf1[record_address]==0)//�����쳣0����·�ε�ַ+1
      {
        location=0;
        record_address++;
      }
      /*if(location>1.0*(record_read_buf1[record_address])&&(record_read_buf2[record_address+1]==4||record_read_buf2[record_address+1]==5||record_read_buf2[record_address+1]==6||record_read_buf2[record_address+1]==7))
      {
        location=0;
        record_address++;
      }*/
      else if(record_read_buf2[record_address+1]==4||record_read_buf2[record_address+1]==5)//���ǰ����ƫб
      {
        ideal_velocity=0.7*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        if(M2>2600)
        {
          gpio_set(D15,1);
          location=0;
          record_address++;
        }
        if(record_read_buf2[record_address+1]==4)//�Ҵ�
            Biasd-=1*0.001*13.3;
        else                                     //���
          Biasd+=1*0.001*13.3;
      }
      else if(record_read_buf2[record_address+1]==6||record_read_buf2[record_address+1]==7)//��С��ǰ����ƫб
      {
        ideal_velocity=0.5*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        if(M2>2600)
        {
          gpio_set(D15,1);
          location=0;
          record_address++;
        }
        if(record_read_buf2[record_address+1]==7)//��С��
            Biasd+=2.5*0.001*13.3;
        else                                     //��С��
            Biasd-=2.5*0.001*13.3;
      }
      else if(record_read_buf2[record_address+1]==8||record_read_buf2[record_address+1]==9)//���л�ǰ����ƫб
      {
        ideal_velocity=0.50*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        if(M2>2200)
        {
          gpio_set(D15,1);
          location=0;
          record_address++;
        }
        if(record_read_buf2[record_address+1]==9)//���л�
            Biasd+=2.0*0.001*13.3;
        else                                     //���л�
            Biasd-=2.0*0.001*13.3;
      }
      else if(location<0.20*(record_read_buf1[record_address])&&record_read_buf2[record_address]==8)//���л�ǰ20%Сƫб
      {
        ideal_velocity=0.60*85;//��������������������������ͬ
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        Biasd-=(Ruhuan_deviate_M*0.001)*0.001*13.3;//4
      }
      /*else if(location>0.80*(record_read_buf1[record_address])&&record_read_buf2[record_address]==8)//���л�������ǰ����Сƫб
      {
        ideal_velocity=0.80*Velocity_Criteria;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        if(M2>2400)
           temp_count++;
        if(temp_count!=0)
            Biasd+=1.0*0.001*13.3;
        if(Integral_anglespeed>7800)
        {
          Integral_anglespeed=0;
          chuhuan_mark=1;
          gpio_set(D15,0);
          temp_count=0;
          location=0;
          record_address++;
        }
      }*/
      else if(location<0.20*(record_read_buf1[record_address])&&record_read_buf2[record_address]==9)//���л�ǰ20%Сƫб
      {
        ideal_velocity=0.60*85;//��������������������������ͬ
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        Biasd+=(Ruhuan_deviate_M*0.001)*0.001*13.3;//5.5
      }
      /*else if(location>0.80*(record_read_buf1[record_address])&&record_read_buf2[record_address]==9)//���л�������ǰ����Сƫб
      {
        ideal_velocity=0.80*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;*/
        /*if(M2>2100)
           temp_count++;
        else
          Biasd+=2.0*0.001*13.3;
        if(temp_count!=0)
            Biasd-=4.0*0.001*13.3;*/
        /*if(Integral_anglespeed<-8000)
        {
          Integral_anglespeed=0;
          chuhuan_mark=1;
          gpio_set(D15,0);
          temp_count=0;
          location=0;
          record_address++;
        }
      }*/
      else if(record_read_buf2[record_address]==8||record_read_buf2[record_address]==9)//�л��м������
      {
        ideal_velocity=0.55*85;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        if(record_read_buf2[record_address]==8)//���л�
        {
            Biasd-=2*0.001*13.3;
            if(Integral_anglespeed>15500)
        {
          Integral_anglespeed=0;
          chuhuan_mark=1;
          gpio_set(D15,0);
          temp_count=0;
          location=0;
          record_address++;
        }
        }
        else                                     //���л�
        {
          Biasd+=2*0.001*13.3;
        if(Integral_anglespeed<-15500)
        {
          Integral_anglespeed=0;
          chuhuan_mark=1;
          gpio_set(D15,0);
          temp_count=0;
          location=0;
          record_address++;
        }
        }
        div_mode=1;
        diversion_mode=0;
      }
      else if(location<0.20*(record_read_buf1[record_address])&&record_read_buf2[record_address]==5)//���ǰ20%Сƫб
      {
        ideal_velocity=0.70*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        Biasd+=(Ruhuan_deviate_L*0.001)*0.001*13.3;//3
      }
      else if(location>0.70*(record_read_buf1[record_address])&&record_read_buf2[record_address]==5)//��󻷳�����ǰ����Сƫб
      {
        ideal_velocity=0.70*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        if(M2>2400)
           temp_count++;
        if(temp_count!=0)
          Biasd-=2.2*0.001*13.3;
        if(location>1*(record_read_buf1[record_address]))
        {
          Integral_anglespeed=0;
          chuhuan_mark=1;
          gpio_set(D15,0);
          temp_count=0;
          location=0;
          record_address++;
        }
      }
      else if(location<0.30*(record_read_buf1[record_address])&&record_read_buf2[record_address]==6)//��С��ǰ30%Сƫб
      {
        ideal_velocity=0.60*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        Biasd-=(Ruhuan_deviate_S*0.001)*0.001*13.3;//5
      }
      else if(location>0.80*(record_read_buf1[record_address])&&record_read_buf2[record_address]==6)//��С��������ǰ����Сƫб
      {
        ideal_velocity=0.60*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        if(M2>2400)
           temp_count++;
        if(temp_count!=0)
            Biasd+=4.0*0.001*13.3;
        if(location>1*(record_read_buf1[record_address]))
        {
          Integral_anglespeed=0;
          chuhuan_mark=1;
          gpio_set(D15,0);
          temp_count=0;
          location=0;
          record_address++;
        }
      }
      else if(location<0.30*(record_read_buf1[record_address])&&record_read_buf2[record_address]==7)//��С��ǰ30%Сƫб
      {
        ideal_velocity=0.60*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        Biasd+=(Ruhuan_deviate_S*0.001)*0.001*13.3;//5
      }
      else if(location>0.80*(record_read_buf1[record_address])&&record_read_buf2[record_address]==7)//��С��������ǰ����Сƫб
      {
        ideal_velocity=0.60*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        if(M2>2400)
           temp_count++;
        if(temp_count!=0)
            Biasd-=3.0*0.001*13.3;
        if(location>1*(record_read_buf1[record_address]))
        {
          Integral_anglespeed=0;
          chuhuan_mark=1;
          gpio_set(D15,0);
          temp_count=0;
          location=0;
          record_address++;
        }
      }
      else if(record_read_buf2[record_address]==6||record_read_buf2[record_address]==7)//С���м������
      {
        ideal_velocity=0.60*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
      }
      else if(location<0.20*(record_read_buf1[record_address])&&record_read_buf2[record_address]==4)//�Ҵ�ǰ20%Сƫб
      {
        ideal_velocity=0.70*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        Biasd-=(Ruhuan_deviate_L*0.001)*0.001*13.3;//3
      }
      else if(location>0.70*(record_read_buf1[record_address])&&record_read_buf2[record_address]==4)//�Ҵ󻷳�����ǰ����Сƫб
      {
        ideal_velocity=0.70*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
        if(M2>2400)
           temp_count++;
        if(temp_count!=0)
          Biasd+=2.0*0.001*13.3;
        if(location>1*(record_read_buf1[record_address]))
        {
          Integral_anglespeed=0;
          chuhuan_mark=1;
          gpio_set(D15,0);
          temp_count=0;
          location=0;
          record_address++;
        }
      }
      else if(record_read_buf2[record_address]==4||record_read_buf2[record_address]==5)//���м������
      {
        ideal_velocity=0.70*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
      }
      else if(location<0.80*(record_read_buf1[record_address])&&record_read_buf2[record_address]==3)//ǰ80%�µ������÷��򻷲�����������
      {
        ideal_velocity=0.50*Wan_velocity;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
      }
      else if(location>=0.80*(record_read_buf1[record_address])&&record_read_buf2[record_address]==3)
      {
        if(Integral_angle>0)
        {
          ideal_velocity=0.50*Zhi_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          div_mode=1;
          diversion_mode=0;
        }
        else
        {
          location=0;
          record_address++;
        }
      }
      /*--------����F1��������----���ã�����----*/
      else if(F1_mode==1)
      {
      if(record_read_buf2[record_address]==0)//ֱ�߼���
      {
        S_road_mark=0;
        Direction_record=0;
        gpio_set(D15,0);
        if(record_read_buf3[record_address+1]==0)
          record_read_buf3[record_address+1]=-record_read_buf3[record_address+2];
        Brake_length=0.24*512*3*(FTM_ave-(50*8.9/Fsqrt(abs(record_read_buf3[record_address+1]))*0.01*Wan_velocity));//ɲ���������
        if(Brake_length<0)
          Brake_length=0;
        //if(((record_read_buf1[record_address])-location)<=0.1*512*3*(ftmave-(125-0.60*abs(record_read_buf3[record_address+1]))))
        if(record_address==Cross_address&&location>0.4*record_read_buf1[record_address]&&location>0.6*record_read_buf1[record_address]&&abs(Biasd)<0.5*0.001*13.3)//����ʮ�ֶ���
        {
          Biasd*=0.5;
        }
        if(location>1.0*record_read_buf1[record_address]&&chuhuan_distance!=0)
        {
          //gpio_set(D15,1);
          location=0;
          record_address++;
        }
        else if(location>2.0*record_read_buf1[record_address]&&record_read_buf1[record_address]>3000)//λ�ô���
        {
          //gpio_set(D15,1);
          location=0;
          record_address=60;
        }
        else if(location>2.0*record_read_buf1[record_address])
        {
          //gpio_set(D15,1);
          location=0;
          record_address++;
        }
        if(record_read_buf2[record_address+1]==1&&record_read_buf3[record_address+1]==0)//�쳣���
        {        
          ideal_velocity=0.50*Wan_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          div_mode=1;
          diversion_mode=0;
        }
        else if(record_read_buf1[record_address]<8000&&(location<record_read_buf1[record_address]-FLASH_TEMP_DISTANCE-Brake_length))//����ֱ������һ����ٶ���
        {
          ideal_velocity=(50*8.9/Fsqrt(abs(record_read_buf3[record_address+1]))*0.01*Wan_velocity);
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          div_mode=1;
          diversion_mode=0;
        }
        else if(record_read_buf1[record_address]<22000&&(location<record_read_buf1[record_address]-FLASH_TEMP_DISTANCE-Brake_length))//��ֱ������
        {
          ideal_velocity=0.6*Zhi_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          div_mode=1;
          diversion_mode=0;
        }
        else if((location>=record_read_buf1[record_address]-FLASH_TEMP_DISTANCE-Brake_length)||Brake_mark==1)//����ɲ��
        {
          ideal_velocity=50*8.9/Fsqrt(abs(record_read_buf3[record_address+1]))*0.01*Wan_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          div_mode=1;
          diversion_mode=1;
          Brake_mark=1;
        }
        else if(location<0.5*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&Brake_mark==0)//ǰ�����������
        {
          ideal_velocity=0.65*Zhi_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          div_mode=1;
          diversion_mode=0;
        }
        else if(location<2*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&Brake_mark==0)//����ɲ��ǰԤ����
        {
          ideal_velocity=0.55*Zhi_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          div_mode=1;
          diversion_mode=0;
        }
        if(location>=0.90*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&record_read_buf2[record_address+1]!=4&&record_read_buf2[record_address+1]!=5&&record_read_buf2[record_address+1]!=6&&record_read_buf2[record_address+1]!=7)
      {
        if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13)||road_check>=20)//�������
      {
        //gpio_set(D15,1);
        Brake_mark=0;
        location=0;
        record_address++;
      }
      }
      }
      if(record_read_buf2[record_address]==1)//��������� Apex��ǰ����������� Apex���������ٳ���
      {
        if(Gyro_z_ave>(50-13)&&Direction_record==0)
        {
          Direction_record++;
          Direction=0;
        }
        if(Gyro_z_ave<(-50-13)&&Direction_record==0)
        {
          Direction_record++;
          Direction=1;
        }
        Brake_length=0;
        //gpio_set(D15,1);
        if(record_read_buf3[record_address]==0)  //�쳣���
        {
          ideal_velocity=0.50*Wan_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          //freql+=1*dif_velocity_L;
          //freqr-=1*dif_velocity_R;
          div_mode=0;
          diversion_mode=1;
        }
        else if(record_read_buf1[record_address]>60000)//�����
        {
          ideal_velocity=55*8.9/Fsqrt(abs(record_read_buf3[record_address]))*0.01*Wan_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          //freql+=1*dif_velocity_L;
          //freqr-=1*dif_velocity_R;
          div_mode=0;
          diversion_mode=1;
        }
        else if(location>1.0*record_read_buf1[record_address]&&chuhuan_distance!=0)
        {
          gpio_set(D15,0);
          location=0;
          record_address++;
        }
        else if(record_read_buf2[record_address+1]==0&&record_read_buf1[record_address+1]<8000&&record_read_buf2[record_address+2]==1)//��s����ж�ֱ��
        {
          ideal_velocity=50*8.9/Fsqrt(abs(record_read_buf3[record_address]))*0.01*Wan_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          div_mode=0;
          diversion_mode=1;
          if(location>1*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE))
          {
            gpio_set(D15,0);
            location=0;
            record_address++;
          }
        }
        else if(location<0.5*record_read_buf1[record_address])   //Apex��ǰ
        {
          ideal_velocity=50*8.9/Fsqrt(abs(record_read_buf3[record_address]))*0.01*Wan_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          //freql+=1*dif_velocity_L;
          //freqr-=1*dif_velocity_R;
          div_mode=0;
          diversion_mode=1;
        }
        else if(record_read_buf2[record_address+1]==1)//��s����޶�ֱ��
        {
          ideal_velocity=50*8.9/Fsqrt(abs(record_read_buf3[record_address]))*0.01*Wan_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          //freql+=1*dif_velocity_L;
          //freqr-=1*dif_velocity_R;
          div_mode=0;
          diversion_mode=1;
          if(location>1*(record_read_buf1[record_address]))
          {
            gpio_set(D15,0);
            location=0;
            record_address++;
          }
        }
        /*else if(location>1.00*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE))
        {
          ideal_velocity=(140-0.1*abs(record_read_buf3[record_address]))*0.01*Wan_velocity;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          //freql+=1*dif_velocity_L;
          //freqr-=1*dif_velocity_R;
          div_mode=1;
          diversion_mode=1;
        }*/
        else if(location<2.0*(record_read_buf1[record_address]))//Apex���
        {
          ideal_velocity=55*8.9/Fsqrt(abs(record_read_buf3[record_address]))*0.01*Wan_velocity;//60
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          //freql+=1*dif_velocity_L;
          //freqr-=1*dif_velocity_R;
          div_mode=0;
          diversion_mode=1;
        }
        else if((record_read_buf1[record_address])>3000)//λ�ô���
        {
          gpio_set(D15,0);
          location=0;
          record_address=60;
        }
        else
        {
          gpio_set(D15,0);
          location=0;
          record_address++;
        }
        if(location>0.60*(record_read_buf1[record_address])&&((Gyro_z_ave<(-0-13)&&Direction==0)||(Gyro_z_ave>(0-13)&&Direction==1))&&record_write_buf2[record_address]==1&&record_write_buf2[record_address+1]==1)
          {
            S_road_mark=1;//����s��
            if(Direction==0)
              Direction++;
            else
              Direction=0;
            location=0;
            record_address++;
          }
        if(location>=1.0*(record_read_buf1[record_address]))
      {
        //if(Gyro_z_ave>(40-13)||Gyro_z_ave<(-40-13)||road_check>=20);
        //else
        if((Gyro_z_ave<(50-13)&&Gyro_z_ave>(-50-13))||road_check<20);
          {
        location=0;
        record_address++;
           }
      }
      }
      if(chuhuan_distance<=Chuhuan_distance*1000&&(record_read_buf2[record_address-1]==4||record_read_buf2[record_address-2]==4||record_read_buf2[record_address-3]==4||record_read_buf2[record_address-1]==5||record_read_buf2[record_address-2]==5||record_read_buf2[record_address-3]==5))//���󻷺�ƫб
      {
        if(record_read_buf2[record_address-1]==4||record_read_buf2[record_address-2]==4||record_read_buf2[record_address-3]==4)//�Ҵ�
            Biasd+=(Chuhuan_deviate_L*0.001)*0.001*13.3;//2.0
        else                                     //���
            Biasd-=(Chuhuan_deviate_L*0.001)*0.001*13.3;//2.5
      }
      else if(chuhuan_distance<=Chuhuan_distance*1000&&(record_read_buf2[record_address-1]==6||record_read_buf2[record_address-2]==6||record_read_buf2[record_address-3]==6||record_read_buf2[record_address-1]==7||record_read_buf2[record_address-2]==7||record_read_buf2[record_address-3]==7))//��С����ƫб
      {
        if(record_read_buf2[record_address-1]==7||record_read_buf2[record_address-2]==7||record_read_buf2[record_address-3]==7)//��С��
            Biasd-=(Chuhuan_deviate_L*0.001)*0.001*13.3;//2.5
        else                                     //��С��
            Biasd+=(Chuhuan_deviate_L*0.001)*0.001*13.3;//2.5
      }
      else if(chuhuan_distance<=Chuhuan_distance*1000&&(record_read_buf2[record_address-1]==8||record_read_buf2[record_address-2]==8||record_read_buf2[record_address-3]==8||record_read_buf2[record_address-1]==9||record_read_buf2[record_address-2]==9||record_read_buf2[record_address-3]==9))//���л���ƫб
      {
        if(record_read_buf2[record_address-1]==9||record_read_buf2[record_address-2]==9||record_read_buf2[record_address-3]==9)//���л�
            Biasd-=(Chuhuan_deviate_M*0.001)*0.001*13.3;//4.7
        else                                     //���л�
            Biasd+=(Chuhuan_deviate_M*0.001)*0.001*13.3;//2.5
      }
      }
      
      /*---------��ͨģʽ------*/
      else if(F1_mode==0)
      {
      if(location<0.80*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&record_read_buf2[record_address]==0&&record_read_buf1[record_address]<22000)//��ֱ������
      {
        ideal_velocity=100;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
      }
      else if(location<0.20*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&record_read_buf2[record_address]==0)//ǰ20%��ֱ���÷��򻷼���������
      {
        ideal_velocity=120;
        shakejudge_gyro();
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
      }
      else if(location<0.50*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&record_read_buf2[record_address]==0)//ǰ50%��ֱ���÷��򻷲�����������
      {
        ideal_velocity=140;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
      }
      else if(location<0.75*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&record_read_buf2[record_address]==0)//ǰ75%ֱ���÷��򻷲�����������
      {
        ideal_velocity=110;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        div_mode=1;
        diversion_mode=0;
      }
      if(location>=0.75*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&record_read_buf2[record_address]==0)
      {
         /*if(abs(record_read_buf3[record_address+1])<=40)
        {
          ideal_velocity=80;
        }
        else if(abs(record_read_buf3[record_address+1])>90)
        {
          ideal_velocity=70;
        }
        else
        {
          ideal_velocity=80;
        }*/
        ideal_velocity=(125-0.60*abs(record_read_buf3[record_address+1]));
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        diversion_mode=0;
      }
      if(location>=0.90*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&record_read_buf2[record_address]==0)
      {
        if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13)||road_check>=20)
      {
        location=0;
        record_address++;
      }
      }
      if(location<1*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&record_read_buf2[record_address]==1)//ǰ100%����ò��ٻ�������������
      {
        /*if(abs(record_read_buf3[record_address])<=40)
        {
          ideal_velocity=80;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          diversion_mode=1;
        }
        else if(abs(record_read_buf3[record_address])>90)
        {
          ideal_velocity=70;
          //freql+=1.2*dif_velocity_L;
          //freqr-=1.2*dif_velocity_R;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          div_mode=3;
          diversion_mode=1;
        }
        else
        {
          ideal_velocity=80;
          //freql+=1*dif_velocity_L;
          //freqr-=1*dif_velocity_R;
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          div_mode=0;
          diversion_mode=1;
        }*/
        ideal_velocity=(125-0.60*abs(record_read_buf3[record_address]));
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        diversion_mode=1;
      }
      if(location>=1*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&record_read_buf2[record_address]==1)
      {
        ideal_velocity=(125-0.40*abs(record_read_buf3[record_address]));
        if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13)||road_check>=20)
      {
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          diversion_mode=0;
        /*if(shake_judge_mark!=0)
        {
          freql-=2.5*divertion;
          freqr+=2.5*divertion;
          shakejudge_gyro();
        }
        else 
        {
          freql+=1*dif_velocity_L;
          freqr-=1*dif_velocity_R;
          div_mode=0;
        }*/
      }
        else
      {
        location=0;
        record_address++;
      }
      }
      if(location>=1.4*(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE)&&record_read_buf2[record_address]==1)
      {
        location=0;
        record_address++;
      }     
    }
    }
    /*----------------��¼ģʽ---------------*/
    if(chuku==0&&record_mark==-1)
    {
      if(count_loopRM == 2&&(record_write_buf2[record_address]==1||record_write_buf2[record_address]==0))//���л�
      {
        gpio_set(D15,1);
        record_write_buf1[record_address]+=record_write_temp1;
        record_float_buf3[record_address]+=record_write_temp3;
        if(record_write_temp2!=2)
          record_write_buf2[record_address]=record_write_temp2;
        record_time[record_address]+=record_time_temp;
        record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
        record_address++;
        record_write_temp1=0;
        record_write_temp3=0;
        record_time_temp=0;
        record_write_temp2=8;//��̬���л�
        record_write_buf2[record_address]=8;
      }
      else if(count_loopRM != 4&&record_write_buf2[record_address]==8)
      {
        record_write_buf1[record_address]+=ftmave;
      }
      else if(record_write_buf2[record_address]==8)
      {
          gpio_set(D15,0);
          record_address++;
          if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13)||road_check>20)
            record_write_buf2[record_address]=1;
          else
            record_write_buf2[record_address]=0;
          record_write_temp2=2;//��ֱ̬��
      }
      if(count_loopLM == 2&&(record_write_buf2[record_address]==1||record_write_buf2[record_address]==0))//���л�
      {
        gpio_set(D15,1);
        record_write_buf1[record_address]+=record_write_temp1;
        record_float_buf3[record_address]+=record_write_temp3;
        if(record_write_temp2!=2)
          record_write_buf2[record_address]=record_write_temp2;
        record_time[record_address]+=record_time_temp;
        record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
        record_address++;
        record_write_temp1=0;
        record_write_temp3=0;
        record_time_temp=0;
        record_write_temp2=9;//��̬���л�
        record_write_buf2[record_address]=9;
      }
      else if(count_loopLM != 4&&record_write_buf2[record_address]==9)
      {
        record_write_buf1[record_address]+=ftmave;
      }
      else if(record_write_buf2[record_address]==9)
      {
          gpio_set(D15,0);
          record_address++;
          if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13)||road_check>20)
            record_write_buf2[record_address]=1;
          else
            record_write_buf2[record_address]=0;
          record_write_temp2=2;//��ֱ̬��
      }
      else if(count_loopL == 2&&(record_write_buf2[record_address]==1||record_write_buf2[record_address]==0))//���
      {
        gpio_set(D15,1);
        record_write_buf1[record_address]+=record_write_temp1;
        record_float_buf3[record_address]+=record_write_temp3;
        if(record_write_temp2!=2)
          record_write_buf2[record_address]=record_write_temp2;
        record_time[record_address]+=record_time_temp;
        record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
        record_address++;
        record_write_temp1=0;
        record_write_temp3=0;
        record_time_temp=0;
        record_write_temp2=5;//��̬���
        record_write_buf2[record_address]=5;
      }
      else if(count_loopL != 4&&record_write_buf2[record_address]==5)
      {
        record_write_buf1[record_address]+=ftmave;
      }
      else if(record_write_buf2[record_address]==5)
      {
          gpio_set(D15,0);
          record_address++;
          if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13)||road_check>20)
            record_write_buf2[record_address]=1;
          else
            record_write_buf2[record_address]=0;
          record_write_temp2=2;//��ֱ̬��
      }
      else if(count_loopr == 2&&(record_write_buf2[record_address]==1||record_write_buf2[record_address]==0))//��С��
      {
        gpio_set(D15,1);
        record_write_buf1[record_address]+=record_write_temp1;
        record_float_buf3[record_address]+=record_write_temp3;
        if(record_write_temp2!=2)
          record_write_buf2[record_address]=record_write_temp2;
        record_time[record_address]+=record_time_temp;
        record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
        record_address++;
        record_write_temp1=0;
        record_write_temp3=0;
        record_time_temp=0;
        record_write_temp2=6;//��̬��С��
        record_write_buf2[record_address]=6;
      }
      else if(count_loopr != 4&&record_write_buf2[record_address]==6)
      {
        record_write_buf1[record_address]+=ftmave;
      }
      else if(record_write_buf2[record_address]==6)
      {
          gpio_set(D15,0);
          record_address++;
          if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13)||road_check>20)
            record_write_buf2[record_address]=1;
          else
            record_write_buf2[record_address]=0;
          record_write_temp2=2;//��ֱ̬��
      }
      else if(count_loopl == 2&&(record_write_buf2[record_address]==1||record_write_buf2[record_address]==0))//��С��
      {
        gpio_set(D15,1);
        record_write_buf1[record_address]+=record_write_temp1;
        record_float_buf3[record_address]+=record_write_temp3;
        if(record_write_temp2!=2)
          record_write_buf2[record_address]=record_write_temp2;
        record_time[record_address]+=record_time_temp;
        record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
        record_address++;
        record_write_temp1=0;
        record_write_temp3=0;
        record_time_temp=0;
        record_write_temp2=7;//��̬��С��
        record_write_buf2[record_address]=7;
      }
      else if(count_loopl != 4&&record_write_buf2[record_address]==7)
      {
        record_write_buf1[record_address]+=ftmave;
      }
      else if(record_write_buf2[record_address]==7)
      {
          gpio_set(D15,0);
          record_address++;
          if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13)||road_check>20)
            record_write_buf2[record_address]=1;
          else
            record_write_buf2[record_address]=0;
          record_write_temp2=2;//��ֱ̬��
      }
      else if(count_loopR == 2&&(record_write_buf2[record_address]==1||record_write_buf2[record_address]==0))//�Ҵ�
      {
        gpio_set(D15,1);
        record_write_buf1[record_address]+=record_write_temp1;
        record_float_buf3[record_address]+=record_write_temp3;
        if(record_write_temp2!=2)
          record_write_buf2[record_address]=record_write_temp2;
        record_time[record_address]+=record_time_temp;
        record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
        record_address++;
        record_write_temp1=0;
        record_write_temp3=0;
        record_time_temp=0;
        record_write_temp2=4;//��̬�Ҵ�
        record_write_buf2[record_address]=4;
      }
      else if(count_loopR != 4&&record_write_buf2[record_address]==4)
      {
        record_write_buf1[record_address]+=ftmave;
      }
      else if(record_write_buf2[record_address]==4)
      {
          gpio_set(D15,0);
          record_address++;
          if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13)||road_check>20)
            record_write_buf2[record_address]=1;
          else
            record_write_buf2[record_address]=0;
          record_write_temp2=2;//��ֱ̬��
      }
      else if(Integral_angle<=-35&&(record_write_buf2[record_address]==1||record_write_buf2[record_address]==0))//����
      { 
            record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
            record_address++;
            if(record_write_temp1!=0)
            {
              record_write_buf1[record_address]+=record_write_temp1;
              record_float_buf3[record_address]+=record_write_temp3;
              if(record_write_temp2!=2)
                record_write_buf2[record_address]=record_write_temp2;
              record_time[record_address]+=record_time_temp;
              record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
              record_address++;
            }
            record_write_temp1=0;
            record_write_temp3=0;
            record_time_temp=0;
            record_write_temp2=3;//��̬�µ�
            record_write_buf2[record_address]=3;
      }
      else if(record_write_buf1[record_address]<11*512*3&&record_write_buf2[record_address]==3)
      {
        record_write_buf1[record_address]+=ftmave;
      }
      else if(record_write_buf1[record_address]>=11*512*3&&record_write_buf2[record_address]==3)
      {
        /*if(Integral_angle>0)
          record_write_buf1[record_address]+=ftmave;
        else
        {*/
          record_address++;
          if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13)||road_check>20)
            record_write_buf2[record_address]=1;
          else
            record_write_buf2[record_address]=0;
          record_write_temp2=2;//��ֱ̬��
        //}
      }
      else if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13))//���
      {            
        if(Gyro_z_ave>(50-13)&&Direction_record==0)
        {
          Direction_record++;
          Direction=0;
        }
        if(Gyro_z_ave<(-50-13)&&Direction_record==0)
        {
          Direction_record++;
          Direction=1;
        }
          if(record_write_temp2==0)
          {
            record_write_buf1[record_address]+=record_write_temp1;
            record_float_buf3[record_address]+=record_write_temp3;
            record_time[record_address]+=record_time_temp;
            record_write_temp1=0;
            record_write_temp3=0;
            record_write_temp2=2;
            record_time_temp=0;
          }
          if(record_write_buf2[record_address]==0)
          {
            record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
            record_write_temp1+=ftmave;
            if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13))
            {
              record_write_temp3+=(Gyro_z_ave+13);
              record_time_temp++;
            }
            record_write_temp2=1;//��̬���
          }
          if(record_write_temp1>=FLASH_TEMP_DISTANCE2&&S_road_mark==0)
          { 
            record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
            record_address++;
            record_write_buf1[record_address]+=record_write_temp1;
            record_float_buf3[record_address]+=record_write_temp3;
            record_write_buf2[record_address]=1;
            record_time[record_address]+=record_time_temp;
            record_write_temp1=0;
            record_write_temp3=0;
            record_time_temp=0;
            record_write_temp2=2;//��Ч��̬
            Apex_point=0;
          }
          if(record_write_buf1[record_address]>FLASH_TEMP_DISTANCE2&&((Gyro_z_ave<(-50-13)&&Direction==0)||(Gyro_z_ave>(50-13)&&Direction==1))&&record_write_buf2[record_address]==1)
          {
            S_road_mark=1;//����s��
            if(Direction==0)
              Direction++;
            else
              Direction=0;
            record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
            record_address++;
            record_write_buf2[record_address]=1;
            record_write_temp1=0;
            record_write_temp3=0;
            record_time_temp=0;
            record_write_temp2=2;//��Ч��̬
            Apex_point=0;
          }
          if(record_write_buf2[record_address]==1)
          {
            gpio_set(D15,1);
            record_write_buf1[record_address]+=ftmave;
            if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13))
            {
              record_float_buf3[record_address]+=(Gyro_z_ave+13);
              record_time[record_address]++;
            }
            if((Direction==0&&Apex_point<Gyro_z_ave)||((Direction==1&&Apex_point>Gyro_z_ave)))
            {
              Apex_point=Gyro_z_ave;
              record_write_buf4[record_address]=record_write_buf1[record_address];
            }
          }
        }
      else//ֱ��
        {
          if(record_write_temp2==1)
          {
            record_write_buf1[record_address]+=record_write_temp1;
            record_float_buf3[record_address]+=record_write_temp3;
            record_time[record_address]+=record_time_temp;
            record_write_temp1=0;
            record_write_temp3=0;
            record_write_temp2=2;
            record_time_temp=0;
          }
          if(record_write_buf2[record_address]==1)
          {
            record_write_buf3[record_address]=(uint32)((record_float_buf3[record_address]+record_write_temp3)/(record_time[record_address]+record_time_temp));
            record_write_temp1+=ftmave;
            if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13))
            {
              record_write_temp3+=(Gyro_z_ave+13);
              record_time_temp++;
            }
            record_write_temp2=0;//��ֱ̬��
          }
          if(record_write_temp1>=FLASH_TEMP_DISTANCE2)
          {
            record_write_buf3[record_address]=(int32)(record_float_buf3[record_address]/record_time[record_address]);
            record_address++;
            record_write_buf1[record_address]+=record_write_temp1;
            record_float_buf3[record_address]+=record_write_temp3;
            record_write_buf2[record_address]=0;
            record_time[record_address]+=record_time_temp;
            record_write_temp1=0;
            record_write_temp3=0;
            record_time_temp=0;
            record_write_temp2=2;//��Ч��̬
            S_road_mark=0;
          }
          if(record_write_buf2[record_address]==0)
          {
            Direction_record=0;
            gpio_set(D15,0);
            record_write_buf1[record_address]+=ftmave;
            if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13))
            {
            record_float_buf3[record_address]+=(Gyro_z_ave+13);
            record_time[record_address]++;
            }
          }
        }
    }
    if(chuku==0&&(record_mark!=0||(record_read_buf1[record_address]==0&&record_read_buf1[record_address+1])))
    {
    if(Gyro_z_ave>(50-13)||Gyro_z_ave<(-50-13)||road_check>=20)
      {
        
        brake_check-=10*ftmave;
        ideal_velocity=0;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        diversion_mode=4;
        //if(brake_activated==0)
          div_mode=4;
        //else
          //div_mode=2;
      }
      else
      {
        brake_check+=ftmave;
        ideal_velocity=0;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        diversion_mode=4;
        div_mode=4;
      }
   // }
    if(shake_judge_mark!=0)
    {//shakejudge_inductance();
      shakejudge_gyro();
    }
    //if(brake_check>(9*512*3))//��ֱ��
    if(brake_check>(2*512*3))//��ֱ��
    {
        brake_activated=1;
        ideal_velocity=0;
        freql-=2.5*divertion;
        freqr+=2.5*divertion;
        diversion_mode=4;
        //freql+=1*dif_velocity_L;
        //freqr-=1*dif_velocity_R;
        div_mode=4;
    }
    else
      brake_activated=0;
    if(brake_check<0)
        brake_check=0;
    /*Loop_trial_R();
    Loop_trial_L();
    Loop_trial_r();
    Loop_trial_l();*/
    switch(MODE)
    {
    case 0:
      if(Loop_turn==0)
      {
        Loop_trial_R();
        if(count_loopR==6)
          Loop_turn++;
      }
      else
        Loop_trial_l();break;//Rl
    case 1:
      if(Loop_turn==0)
      {
        Loop_trial_r();
        if(count_loopr==6)
          Loop_turn++;
      }
      else
        Loop_trial_L();break;//rL
    case 2:
      if(Loop_turn==0)
      {
        Loop_trial_R();
        if(count_loopR==6)
          Loop_turn++;
      }
      else
        Loop_trial_r();break;//Rr
    case 3:
      if(Loop_turn==0)
      {
        Loop_trial_L();
        if(count_loopL==6)
          Loop_turn++;
      }
      else
        Loop_trial_l();break;//Ll
    case 4:
      if(Loop_turn==0)
      {
        Loop_trial_R();
        if(count_loopL==6)
          Loop_turn++;
      }
      else
        Loop_trial_R();break;//RR
    case 5:
      if(Loop_turn==0)
      {
        Loop_trial_R();
        if(count_loopL==6)
          Loop_turn++;
      }
      else
        Loop_trial_L();break;//RL
    case 6:
      if(Loop_turn==0)
      {
        Loop_trial_l();
        if(count_loopl==6)
          Loop_turn++;
      }
      else
        Loop_trial_L();break;//lL
    case 7:Loop_trial_LM();break;//LM
    case 8:Loop_trial_RM();break;//RM
    case 9:break;
    case 10:break;
    case 11:break;
    case 12:break;
    case 13:break;
    case 14:break;
    case 15:break;
    default:break;
    }
    }
    if(emergency_stop==1)
      ideal_velocity=20;
    if(chuku!=0)
      CHUKU(chuku_direction);
    Dutycontrol();
}
void Oscilloscope(void)
{
    /*addr[0]=count_loopRM*100;
    addr[1]=Integral_anglespeed;
    addr[2]=ideal_velocity;
    addr[3]=RV;
    addr[4]=LV;
    addr[5]=M2;
    addr[6]=record_address*100;
    addr[7]=ftmave;*/
    addr[0]=Gyro_z_ave+13;                       //ʾ����
    addr[1]=freql;
    addr[2]=Biasd*1000/13.3;
    addr[3]=FTM_ave;
    addr[4]=record_address*100;
    addr[5]=location/100;
    addr[6]=freqr;
    addr[7]=(record_read_buf1[record_address]-FLASH_TEMP_DISTANCE-Brake_length)/100;
    //addr[7]=M2;
    vcan_sendware(addr,sizeof(addr));
}
/*------------����·��----------*/
static void shakejudge_inductance(void)//�����ж�-�������
{
    if(shake_count!=0)
      shake_distance+=ftmave;
    if(shake_count==0&&Biasd>(100*0.001/13.3))
    {
      shake_count=1;
    }
    if(shake_count==1&&Biasd<-(100*0.001/13.3))
    {
      shake_count=2;
    }
    if(shake_count==2&&Biasd>(100*0.001/13.3))
    {
      shake_count=-1;
      shake_distance=0;
    }
    if(shake_count==0&&Biasd<-(100*0.001/13.3))
    {
      shake_count=3;
    }
    if(shake_count==3&&Biasd>(100*0.001/13.3))
    {
      shake_count=4;
    }
    if(shake_count==4&&Biasd<-(100*0.001/13.3))
    {
      shake_count=-1;
      shake_distance=0;
    }
    if(shake_count==-1)
    {
      ideal_velocity=50;
      div_mode=2;
      gpio_set(D15,1);
    }
    if(shake_count>0&&shake_distance>(150*100))
    {
      shake_count=0;
      shake_distance=0;
    }
    if(shake_count==-1&&shake_distance>(70*100))
    {
      shake_count=0;
      shake_distance=0;
    }
}
static void shakejudge_gyro(void)//�����ж�-����������
{
    if(shake_count!=0)
      shake_distance+=ftmave;
    if(shake_count==0&&mpu_gyro_z>(250+Gyro_z_ave))
    {
      shake_count=1;
    }
    if(shake_count==1&&mpu_gyro_z<(-250+Gyro_z_ave))
    {
      shake_count=2;
    }
    if(shake_count==2&&mpu_gyro_z>(250+Gyro_z_ave))
    {
      shake_count=3;
    }
    if(shake_count==3&&mpu_gyro_z<(-250+Gyro_z_ave))
    {
      shake_count=4;
    }
    if(shake_count==4&&mpu_gyro_z>(250+Gyro_z_ave))
    {
      shake_count=-1;
      shake_distance=0;
    }
    if(shake_count==0&&mpu_gyro_z<(-250+Gyro_z_ave))
    {
      shake_count=5;
    }
    if(shake_count==5&&mpu_gyro_z>(250+Gyro_z_ave))
    {
      shake_count=6;
    }
    if(shake_count==6&&mpu_gyro_z<(-250+Gyro_z_ave))
    {
      shake_count=7;
    }
    if(shake_count==7&&mpu_gyro_z>(250+Gyro_z_ave))
    {
      shake_count=8;
    }
    if(shake_count==8&&mpu_gyro_z<-250+(Gyro_z_ave))
    {
      shake_count=-1;
      shake_distance=0;
    }
    if(shake_count==-1)
    {
      if(record_mark==0&&record_read_buf1[record_address]!=0)
        ideal_velocity=80;
      else
        ideal_velocity=60;
      div_mode=1;
    }
    if(shake_count>0&&shake_distance>(150*100))
    {
      shake_count=0;
      shake_distance=0;
    }
    if(shake_count==-1&&(mpu_gyro_z>(300+Gyro_z_ave)||mpu_gyro_z>(-300+Gyro_z_ave)))
      shake_distance=0;
    if(shake_count==-1&&shake_distance>(0.5*512*3))
    {
      shake_count=0;
      shake_distance=0;
    }
}
void Loop_trial_R()//�Ҵ�
{    
  if(count_loopR == 0 && RH>2400 && LH>2400 && RV>300)
  {//�����뻷�е㣬Ŀǰ�ݶ����뻷���ֵ����
    count_loopR = 1;
    
  }
  if(count_loopR == 1 && M2>2400 && RV<10 && LV<10)//
  {
    count_loopR = 2;
    
  }
  if(count_loopR == 2 && Integral_speed < 25000 /*&& Integral_anglespeed < 4000*/)
  {//�ݶ�ftm\ƫ���ǶȻ�������,3600=3Ȧ
    Biasd -= Ruhuan_record_L*0.001*0.001*13.3;//���Ҵ󻷴�� 4
    Integral_speed += ftmave;//ftm����
    
  } 
  if(count_loopR == 2 && Integral_speed >= 25000 /*&& Integral_anglespeed >= 4000*/)
  {//������������󣬻ָ�ѭ������������
    count_loopR = 3;//��־λ��3
    Integral_speed = 0;//��������
    
  }
  if(count_loopR == 3 && Integral_anglespeed < 6000)//6000
  {//
    Integral_anglespeed += 2000.0*(mpu_gyro_z+13)/32768;//�����ǻ���
    
  }
  if(count_loopR == 3 && Integral_anglespeed >= 6000)
  {//������
    count_loopR = 4;
    Integral_anglespeed = 0;//��������
  }
  if(count_loopR == 4 && M2>2600 && RV<10 && LV<10)//ԭ��RV<50 && LV<50
  {//��������е�
    count_loopR = 5;
    
  }
  if(count_loopR == 5 && Integral_speed < 40000)
  {//������ǣ�ftm���־���̵��
    Biasd += Chuhuan_record_L*0.001*0.001*13.3;//���Ҵ󻷴��   0.8
    Integral_speed += ftmave;//ftm����
    
  }
  if(count_loopR == 5 && Integral_speed >= 40000)
  {//���ֽ�����ftm�������㣬��־λ���㣬�ָ�ѭ��
    count_loopR = 6;//��־λ��600  
    Integral_speed = 0;//��������
    
  }
}
void Loop_trial_l()//��С��
{
  if(count_loopl == 0 && RH>2500 && LH>2500 && LV>300)
  {//������С��ֱ������,���ý������뻷�е�
    count_loopl = 1;//������С��ֱ������־λ��1

  }
  else if(count_loopl == 1 && M2>2400 && RV<10 && LV<10)
  {//�����뻷�е㣬Ŀǰ�ݶ����뻷���ֵ����
    count_loopl = 2;//��־λ��2
  }
  else if(count_loopl == 2 && Integral_speed < 1500)
  {
    //ѭ��һС�ξ���
    Integral_speed += ftmave;//ftm����
  }
  else if(count_loopl == 2 && 1500 <= Integral_speed && Integral_speed < 14000 )
  {//�ݶ�ftm\ƫ���ǶȻ�������,3600=3Ȧ
    Biasd = Ruhuan_record_S*0.001*0.001*13.3;//����С��ֱ�Ӵ�� 1.6
    Integral_speed += ftmave;//ftm����
  } 
  else if(count_loopl == 2 && Integral_speed >= 14000 )
  {//������������󣬻ָ�ѭ������������
    count_loopl = 3;//��־λ��3
    Integral_speed = 0;//��������
    Integral_anglespeed = 0;////��������
  }
  else if(count_loopl == 3 && Integral_anglespeed <= -5000)
  {//�������������ý���������е�
    count_loopl = 4;//��־λ��4
    Integral_anglespeed = 0;
  }
  else if(count_loopl == 3)
  {
    Integral_anglespeed += 2000.0*(mpu_gyro_z+13)/32768;//�����ǻ���
  }
  else if(count_loopl == 4 && M2>2400 && LV<200 && RV<200)//��������
  {//��������е�
    count_loopl = 5;     
  }
  else if(count_loopl == 5 && Integral_speed < 20000)
  {
    Biasd -= Chuhuan_record_S*0.001*0.001*13.3;//����С����� 1.5
    Integral_speed += ftmave;//ftm����
  }
  else if(count_loopl == 5 && Integral_speed >= 20000)
  {//���ֽ�����ftm�������㣬��־λ���㣬�ָ�ѭ��
    count_loopl = 6;//��־λ��600  
    Integral_speed = 0;//��������
  }
}
void Loop_trial_L()//���
{
   if(count_loopL == 0 && RH>2400 && LH>2400 && LV>300)
  {//�����뻷�е㣬Ŀǰ�ݶ����뻷���ֵ����
    count_loopL = 1;
    
  }
  if(count_loopL == 1 && M2>2400 && RV<10 && LV<10)//
  {
    count_loopL = 2;
    
  }
  if(count_loopL == 2 && Integral_speed < 25000 /*&& Integral_anglespeed < 4000*/)
  {//�ݶ�ftm\ƫ���ǶȻ�������,3600=3Ȧ
    Biasd += Ruhuan_record_L*0.001*0.001*13.3;//���Ҵ󻷴�� 3
    Integral_speed += ftmave;//ftm����
    
  } 
  if(count_loopL == 2 && Integral_speed >= 25000 /*&& Integral_anglespeed >= 4000*/)
  {//������������󣬻ָ�ѭ������������
    count_loopL = 3;//��־λ��3
    Integral_speed = 0;//��������
    
  }
  if(count_loopL == 3 && Integral_anglespeed > -6000)//6000
  {//
    Integral_anglespeed += 2000.0*(mpu_gyro_z+13)/32768;//�����ǻ���
    
  }
  if(count_loopL == 3 && Integral_anglespeed <= -6000)
  {//������
    count_loopL = 4;
    Integral_anglespeed = 0;//��������
  }
  if(count_loopL == 4 && M2>2600 && RV<10 && LV<10)//ԭ��RV<50 && LV<50
  {//��������е�
    count_loopL = 5;
    
  }
  if(count_loopL == 5 && Integral_speed < 40000)
  {//������ǣ�ftm���־���̵��
    Biasd -= Chuhuan_record_L*0.001*0.001*13.3;//���Ҵ󻷴��    0.8
    Integral_speed += ftmave;//ftm����
    
  }
  if(count_loopL == 5 && Integral_speed >= 40000)
  {//���ֽ�����ftm�������㣬��־λ���㣬�ָ�ѭ��
    count_loopL = 6;//��־λ��600
    Integral_speed = 0;//��������
    
  }
}
void Loop_trial_r()//��С��
{
  if(count_loopr == 0 && RH>2500 && LH>2500 && RV>300)
  {//������С��ֱ������,���ý������뻷�е�
    count_loopr = 1;//������С��ֱ������־λ��1

  }
  else if(count_loopr == 1 && M2>2400 && RV<10 && LV<10)
  {//�����뻷�е㣬Ŀǰ�ݶ����뻷���ֵ����
    count_loopr = 2;//��־λ��2
    Integral_speed = 0;
  }
  else if(count_loopr == 2 && Integral_speed < 1500)
  {
    //ѭ��һС�ξ���
    Integral_speed += ftmave;//ftm����
  }
  else if(count_loopr == 2 && 1500 <= Integral_speed &&Integral_speed < 14000 )
  {//�ݶ�ftm\ƫ���ǶȻ�������,3600=3Ȧ
    Biasd = -Ruhuan_record_S*0.001*0.001*13.3;//����С��ֱ�Ӵ�� 1.6
    Integral_speed += ftmave;//ftm����
  } 
  else if(count_loopr == 2 && Integral_speed >= 14000 )
  {//������������󣬻ָ�ѭ������������
    count_loopr = 3;//��־λ��3
    Integral_speed = 0;//��������
    Integral_anglespeed = 0;////��������
  }
  else if(count_loopr == 3 && Integral_anglespeed >= 5000)
  {//�������������ý���������е�
    count_loopr = 4;//��־λ��4
    Integral_anglespeed = 0;
  }
  else if(count_loopr == 3)
  {
    Integral_anglespeed += 2000.0*(mpu_gyro_z+13)/32768;//�����ǻ���
  }
  else if(count_loopr == 4 && M2>2400 && LV<200 && RV<200)//��������
  {//��������е�
    count_loopr = 5;     
  }
  else if(count_loopr == 5 && Integral_speed < 20000)
  {
    Biasd += Chuhuan_record_S*0.001*0.001*13.3;//����С����� 2
    Integral_speed += ftmave;//ftm����
  }
  else if(count_loopr == 5 && Integral_speed >= 20000)
  {//���ֽ�����ftm�������㣬��־λ���㣬�ָ�ѭ��
    count_loopr = 6;//��־λ��600  
    Integral_speed = 0;//��������
  }
}
void Loop_trial_LM()//���л�
{
       if(count_loopLM == 0 && RH>2500 && LH>2500 && LV>2000)
  {//�������л�ֱ������,���ý������뻷�е�
    count_loopLM = 1;//�������л�ֱ������־λ��1

  }
  else if(count_loopLM == 1 && M2>2600 && RV<10)
  {//�����뻷�е㣬Ŀǰ�ݶ����뻷���ֵ����
    count_loopLM = 2;//��־λ��2
    gpio_set(D15,1);//��������
  }
  else if(count_loopLM == 2 && Integral_speed < Ruhuan_record_distance)
  {
    //ѭ��һС�ξ���
    Integral_speed += ftmave;//ftm����
  }
  else if(count_loopLM == 2 && Ruhuan_record_distance <= Integral_speed && Integral_speed < 16000 )
  {//�ݶ�ftm\ƫ���ǶȻ�������,3600=3Ȧ
    Biasd = Ruhuan_record_M*0.001*0.001*13.3;//�����л�ֱ�Ӵ�� 2.5
    Integral_speed += ftmave;//ftm����
  } 
  else if(count_loopLM == 2 && Integral_speed >= 16000 )
  {//������������󣬻ָ�ѭ������������
    count_loopLM = 3;//��־λ��3
    Integral_speed = 0;//��������
    Integral_anglespeed = 0;////��������
  }
  else if(count_loopLM == 3 && Integral_anglespeed <= -15000)
  {//�������������ý���������е�
    count_loopLM = 4;//��־λ��4
    Integral_anglespeed = 0;
  }
  else if(count_loopLM == 3)
  {
    Integral_anglespeed += 2000.0*(mpu_gyro_z+13)/32768;//�����ǻ���
    //gpio_set(D15,1);//��������
  }
  /*else if(count_loopLM == 4 && M2>2600 && RV<10)//��������
  {//��������е�
    count_loopLM = 5;     
  }*/
  else if(count_loopLM == 4 && Integral_speed < Chuhuan_record_distance*10)
  {
    Biasd -= Chuhuan_record_M*0.001*0.001*13.3;//�����л���� 2
    Integral_speed += ftmave;//ftm����
  }
  else if(count_loopLM == 4 && Integral_speed >= Chuhuan_record_distance*10)
  {//���ֽ�����ftm�������㣬��־λ���㣬�ָ�ѭ��
    count_loopLM = 5;//��־λ��500  
    Integral_speed = 0;//��������
  }
}
void Loop_trial_RM()//���л�
{
    if(count_loopRM == 0 && RH>2500 && LH>2500 && RV>2000)
  {//�������л�ֱ������,���ý������뻷�е�
    count_loopRM = 1;//�������л�ֱ������־λ��1

  }
  else if(count_loopRM == 1 && M2>2400 && LV<10)
  {//�����뻷�е㣬Ŀǰ�ݶ����뻷���ֵ����
    count_loopRM = 2;//��־λ��2
    gpio_set(D15,1);//��������
    Integral_speed = 0;
  }
  else if(count_loopRM == 2 && Integral_speed < Ruhuan_record_distance)
  {
    //ѭ��һС�ξ���
    Integral_speed += ftmave;//ftm����
  }
  else if(count_loopRM == 2 && Ruhuan_record_distance <= Integral_speed && Integral_speed < 14000 )
  {//�ݶ�ftm\ƫ���ǶȻ�������,3600=3Ȧ
    Biasd = -Ruhuan_record_M*0.001*0.001*13.3;//�����л�ֱ�Ӵ�� 2.5
    Integral_speed += ftmave;//ftm����
  } 
  else if(count_loopRM == 2 && Integral_speed >= 14000 )
  {//������������󣬻ָ�ѭ������������
    count_loopRM = 3;//��־λ��3
    Integral_speed = 0;//��������
    Integral_anglespeed = 0;////��������
  }
  else if(count_loopRM == 3 && Integral_anglespeed >= 14000)
  {//�������������ý���������е�
    count_loopRM = 4;//��־λ��4
    gpio_set(D15,0);//����������
    Integral_anglespeed = 0;
  }
  else if(count_loopRM == 3)
  {
    Integral_anglespeed += 2000.0*(mpu_gyro_z+13)/32768;//�����ǻ���
    //gpio_set(D15,1);//��������
  }
  else if(count_loopRM == 4 && Integral_speed < Chuhuan_record_distance*10)
  {
    Biasd += Chuhuan_record_M*0.001*0.001*13.3;//�����л���� 2.5
    Integral_speed += ftmave;//ftm����
  }
  else if(count_loopRM == 4 && Integral_speed >= Chuhuan_record_distance*10)
  {//���ֽ�����ftm�������㣬��־λ���㣬�ָ�ѭ��
    count_loopRM = 5;//��־λ��500  
    Integral_speed = 0;//��������
  }
}
/*---------------����---------------*/
static int chuku_angle,chuku_distance;
void CHUKU(int8 direction)//����
{
     ideal_velocity=10;
    div_mode = 0;
    diversion_mode = 2;
    
    if(chuku==2&&RV<300&&LV<300)//��������ֱ��ж�С��300ʱֱ��
    {
      gpio_set(D15, 1);
      Biasd=0;
    }
    if(chuku==2&&RV>=300&&LV>=300)//��������ֱ��ж�����300ʱ����־λchuku-1
    {
      chuku--;
      gpio_set(D15, 0);
    }
    if(chuku==1)//
    {       
      div_mode=0;
      diversion_mode = 2;
      
      if(direction==0)//�������
         Biasd=3.5*13.3*0.001;//4
      else                  //���ҳ���
         Biasd=-3.5*13.3*0.001;//4
      freql-=3*divertion;
      freqr+=3*divertion;
      //freql+=1.2*dif_velocity_L;
      //freqr-=1.2*dif_velocity_R;
      chuku_distance += ftmave;
    }
    if(chuku==1&&((LH+RH)>3500&&abs(Biasd)<0.2*0.001*13.3)||chuku_distance>=30000)//ֹͣ�����ǵ�����
    {
      gpio_set(D15, 0);
      chuku--;
      chuku_distance = 0;
    } 
}
int main(void)
{
    DisableGlobalIRQ();
    board_init();//��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���
    flash_init();   //��ʼ��flash
	//�˴���д�û�����(���磺�����ʼ�������)
    
    gpio_init(C31, GPO, 1, GPIO_PIN_CONFIG);
    gpio_set(C31,1);//C3 ����ߵ�ƽ
    gpio_init(B9, GPO, 1, GPIO_PIN_CONFIG);//С����
    gpio_init(D15, GPO, 1, GPIO_PIN_CONFIG);//������
    systick_delay_ms(80);
    gpio_set(D15,0);
    systick_delay_ms(80);
    gpio_set(D15,1);
    systick_delay_ms(80);
    gpio_set(D15,0);
    key_init(KEY_MAX);
    simiic_init();
    mpu6050_init();
    
    oled_init();
    
    adc_init(ADC_1,ADC1_CH3_B14,ADC_12BIT);
    adc_init(ADC_1,ADC1_CH4_B15,ADC_12BIT);
    adc_init(ADC_1,ADC1_CH5_B16,ADC_12BIT);
    adc_init(ADC_1,ADC1_CH6_B17,ADC_12BIT);
    adc_init(ADC_1,ADC1_CH7_B18,ADC_12BIT);
    adc_init(ADC_1,ADC1_CH8_B19,ADC_12BIT);
    
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER0_C0,QTIMER1_TIMER1_C1);
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER2_C2,QTIMER1_TIMER3_C24);
    //�ڰ׻���
    pwm_init(PWM1_MODULE3_CHA_D0, 13*1000, 0);//��ǰ
    pwm_init(PWM1_MODULE3_CHB_D1, 13*1000, 0);//�Һ�
    pwm_init(PWM1_MODULE0_CHA_D12, 13*1000,0);//���
    pwm_init(PWM1_MODULE0_CHB_D13, 13*1000,  0);//��ǰ
    //pwm_init(PWM1_MODULE1_CHB_D15, 500, 0);//������
    //pwm_init(PWM4_MODULE3_CHA_C31, 500, 0);//������
    uart_init (USART_1, 115200,UART1_TX_B12,UART1_RX_B13); //�ϰ׳Ⱥ�
    //uart_putstr(USART_1,"i lvoe you");
    //��ʼ������   ������Ϊ115200 TXΪD16 RXΪD17
    uart_init (USART_3, 115200,UART3_TX_B22,UART3_RX_B23);
    //uart_putstr(USART_1,"i lvoe you");
    NVIC_SetPriority(LPUART3_IRQn,15);         //���ô����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ��
    uart_rx_irq(USART_3,1);
    
    //���ô��ڽ��յĻ�����������������
    example_receivexfer.dataSize = 1;
    example_receivexfer.data = &example_rx_buffer;
    
    //�����жϺ����������
    uart_set_handle(USART_3, &example_g_lpuartHandle, example_uart_callback, NULL, 0, example_receivexfer.data, 1);
    
    pit_init();                     //��ʼ��pit����
    pit_interrupt_ms(PIT_CH0,1);  //��ʼ��pitͨ��0 ����
    NVIC_SetPriority(PIT_IRQn,0);  //�����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ�� ��·PIT����һ��PIT�жϺ���
      
    distance=6800;
    
    flash_read_page(PARAMETER_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_1, record_parameter_save1, FLASH_SAVE_NUM);
    firstlist();                      //�˵�
    location=0;
    Read_flash();
    EnableGlobalIRQ(0);
    //if(music_mark!=0)
        //pwm_init(PWM1_MODULE1_CHB_D15, 500, 0);//������
    while (1)
    {
      //if(music_mark!=0)
          //Play_Song2(0); //���� 
      Oscilloscope();
      Record_road();
      if(emergency_stop==0)   //����
          oled_printf_float(0,1,distance,4,0);
      if(distance<Chuanqiu_distance&&distance>=80)
      {
        oled_printf_float(0,1,distance,4,0);
        emergency_stop=1;
      }
      if(distance<80)
      {
        emergency_stop=2;
        gpio_set(C31,0);//C3 ����͵�ƽ
        oled_printf_float(0,1,0001,4,0);
        uart_putchar(USART_1,0xA5); 
        if(record_mark==-1)
      {
        record_mark=-2;
        trinity();
      }
      }
        //�˴���д��Ҫѭ��ִ�еĴ���

    }
}





