#include "menu.h"
#include "SEEKFREE_OLED.h"
#include "SEEKFREE_MPU6050.h"
#include "string.h"
#include "key.h"
#include "SEEKFREE_FONT.h"

//定义flash最后一个扇区，避免与程序冲突
//存储参数最好从最后一个扇区开始使用
#define EXAMPLE_FLASH_SECTOR        (FLASH_SECTOR_NUM-1)
#define PARAMETER_FLASH_SECTOR        (FLASH_SECTOR_NUM-2)//参数存取扇区

//定义所在扇区的页编号
#define EXAMPLE_FLASH_SECTOR_PAGE_1   (0)
#define EXAMPLE_FLASH_SECTOR_PAGE_2   (1)
#define EXAMPLE_FLASH_SECTOR_PAGE_3   (2)
#define EXAMPLE_FLASH_SECTOR_PAGE_4   (3)
#define FLASH_SAVE_NUM  64//宏定义需要存储参数的个数    一个页最多为256字节，一个参数需要占4个字节，因此最多这里为256/4=64

uint8 status2;

int8 chuku_direction=0,chuku=0;//出库方向，0左1右2无出库
uint16 Zhi_velocity,Wan_velocity,Acceleration_Criteria,Chuhuan_distance;
uint16 Wan_P,Wan_D,Zhi_P,Zhi_D;//方向环PD
uint16 Cross_address;//十字在flash里的位置
uint16 Ruhuan_deviate_S,Ruhuan_deviate_M,Ruhuan_deviate_L,Chuhuan_deviate_S,Chuhuan_deviate_M,Chuhuan_deviate_L;//入环出环偏斜
uint16 Ruhuan_record_S,Ruhuan_record_M,Ruhuan_record_L,Chuhuan_record_S,Chuhuan_record_M,Chuhuan_record_L;//记录模式入环出环打角的1000倍
uint16 Ruhuan_record_distance,Chuhuan_record_distance;//记录模式入环出环打角偏移的距离
uint16 Chuanqiu_distance;//传球前的减速距离
int32 freql,freqr;
int16 vall,valr;
float RV,LV,RH,LH,M1,M2;
float LV_normal,RV_normal;
float Integral_angle,angle;
float Biasd;
uint8 txt[30];
int16 reset1,reset2,reset3,reset4;
uint8 MODE;
extern uint8 tick_mark;
int16 music_mark,record_mark=0,record_mark2=0;
uint8 F1_mode=0;
uint32 record_read_buf1[FLASH_SAVE_NUM],record_read_buf2[FLASH_SAVE_NUM],record_read_buf3[FLASH_SAVE_NUM],record_read_buf4[FLASH_SAVE_NUM];
uint32 record_parameter_save1[FLASH_SAVE_NUM];

void Read_flash(void)
{
  flash_read_page(EXAMPLE_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_1, record_read_buf1, FLASH_SAVE_NUM);
  flash_read_page(EXAMPLE_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_2, record_read_buf2, FLASH_SAVE_NUM);
  flash_read_page(EXAMPLE_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_3, record_read_buf3, FLASH_SAVE_NUM);
  flash_read_page(EXAMPLE_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_4, record_read_buf4, FLASH_SAVE_NUM);
}
void Load(void)//载入参数
{
  Zhi_velocity=record_parameter_save1[0];//速度
  Acceleration_Criteria=record_parameter_save1[1];//加速度
  chuku=record_parameter_save1[2];//是否出库
  chuku_direction=record_parameter_save1[3];//出库方向
  Chuhuan_distance=record_parameter_save1[4];//出环距离
  Wan_P=record_parameter_save1[5];
  Wan_D=record_parameter_save1[6];
  Zhi_P=record_parameter_save1[7];
  Zhi_D=record_parameter_save1[8];
  Cross_address=record_parameter_save1[9];
  Ruhuan_deviate_S=record_parameter_save1[10];
  Ruhuan_deviate_M=record_parameter_save1[11];
  Ruhuan_deviate_L=record_parameter_save1[12];
  Chuhuan_deviate_S=record_parameter_save1[13];
  Chuhuan_deviate_M=record_parameter_save1[14];
  Chuhuan_deviate_L=record_parameter_save1[15];
  Ruhuan_record_S=record_parameter_save1[16];
  Ruhuan_record_M=record_parameter_save1[17];
  Ruhuan_record_L=record_parameter_save1[18];
  Chuhuan_record_S=record_parameter_save1[19];
  Chuhuan_record_M=record_parameter_save1[20];
  Chuhuan_record_L=record_parameter_save1[21];
  Ruhuan_record_distance=record_parameter_save1[22];
  Chuhuan_record_distance=record_parameter_save1[23];
  Chuanqiu_distance=record_parameter_save1[24];
  Wan_velocity=record_parameter_save1[25];
}
/*拨码开关检测,4位2进制编码，16种模式*/
void Boma_check(void)
{
  uint8 boma1,boma2,boma3,boma4;
  if(key_check(KEY_1)==KEY_DOWN)
    {
      boma1=1;
    }
  else
    boma1=0;
  if(key_check(KEY_2)==KEY_DOWN)
    {
      boma2=1;
    }
  else
    boma2=0;
  if(key_check(KEY_3)==KEY_DOWN)
    {
      boma3=1;
    }
  else
    boma3=0;
  if(key_check(KEY_4)==KEY_DOWN)
    {
      boma4=1;
    }
  else
    boma4=0;
  if(boma1==0&&boma2==0&&boma3==0&&boma4==0)
    MODE=0;
  else if(boma1==0&&boma2==0&&boma3==0&&boma4==1)
    MODE=1;
  else if(boma1==0&&boma2==0&&boma3==1&&boma4==0)
    MODE=2;
  else if(boma1==0&&boma2==0&&boma3==1&&boma4==1)
    MODE=3;
  else if(boma1==0&&boma2==1&&boma3==0&&boma4==0)
    MODE=4;
  else if(boma1==0&&boma2==1&&boma3==0&&boma4==1)
    MODE=5;
  else if(boma1==0&&boma2==1&&boma3==1&&boma4==0)
    MODE=6;
  else if(boma1==0&&boma2==1&&boma3==1&&boma4==1)
    MODE=7;
  else if(boma1==1&&boma2==0&&boma3==0&&boma4==0)
    MODE=8;
  else if(boma1==1&&boma2==0&&boma3==0&&boma4==1)
    MODE=9;
  else if(boma1==1&&boma2==0&&boma3==1&&boma4==0)
    MODE=10;
  else if(boma1==1&&boma2==0&&boma3==1&&boma4==1)
    MODE=11;
  else if(boma1==1&&boma2==1&&boma3==0&&boma4==0)
    MODE=12;
  else if(boma1==1&&boma2==1&&boma3==0&&boma4==1)
    MODE=13;
  else if(boma1==1&&boma2==1&&boma3==1&&boma4==0)
    MODE=14;
  else if(boma1==1&&boma2==1&&boma3==1&&boma4==1)
    MODE=15;
}
/*矩阵转换为整型变量*/
uint16 matrixswitchint(uint16 *a)
{
  uint16 i,num=0;
  for(i=0;i<4;i++)
  {
    num+=a[i]*pow(10,3-i);
  }
  return num;
}
void OLED_capacity_new()
{
  sprintf((char*)txt,"LH:%06d",(int)LH);
  oled_p6x8str(0,0,txt);
  sprintf((char*)txt,"RH:%06d",(int)RH);
  oled_p6x8str(0,1,txt);
  sprintf((char*)txt,"LV:%06d",(int)LV_normal);
  oled_p6x8str(0,2,txt);
  sprintf((char*)txt,"RV:%06d",(int)RV_normal);
  oled_p6x8str(0,3,txt);
  sprintf((char*)txt,"M2:%06d",(int)M2);
  oled_p6x8str(0,4,txt);
}

void OLED_FTM_new()
{
  sprintf((char*)txt,"VL；:%06d",vall);
  oled_p6x8str(0,5,txt);
  sprintf((char*)txt,"VR:%06d",valr);
  oled_p6x8str(0,6,txt);
  sprintf((char*)txt,"aveFTM:%06d",(valr-vall)/2);
  oled_p6x8str(0,7,txt);
}
void OLED_6050_new()
{
  sprintf((char*)txt,"GyroX:%06d",mpu_gyro_x);
  oled_p6x8str(0,0,txt);
  sprintf((char*)txt,"GyroY:%06d",mpu_gyro_y);
  oled_p6x8str(0,1,txt);
  sprintf((char*)txt,"GyroZ:%06d",mpu_gyro_z);
  oled_p6x8str(0,2,txt);
  sprintf((char*)txt,"AccX:%06d",mpu_acc_x);
  oled_p6x8str(0,3,txt);
  sprintf((char*)txt,"AccY:%06d",mpu_acc_y);
  oled_p6x8str(0,4,txt);
  sprintf((char*)txt,"AccZ:%06d",mpu_acc_z);
  oled_p6x8str(0,5,txt);
  sprintf((char*)txt,"Iangle:%06d",(int)Integral_angle);
  oled_p6x8str(0,6,txt);
  sprintf((char*)txt,"Angle:%06d",(int)angle);
  oled_p6x8str(0,7,txt);
}
/*在一级或部分菜单中去除右边五向导航的位置标志*/
void Clean_Key_PositionY()
{
  uint8 y=0,i=0;
  for(y=0;y<=7;y++)
  {
    oled_set_pos(120,y);
    for(i=0;i<6;i++) 
    {
      oled_wrdat(oled_6x8[0][i]);
    }
  }
}
/*在一级或部分菜单下去除下边五向导航的位置标志*/
void Clean_Key_PositionX()
{
  uint8 x=0,i=0;
  for(x=0;x<=120;x+=6)
  {
    oled_set_pos(x,2);
    for(i=0;i<6;i++) 
    {
      oled_wrdat(oled_6x8[0][i]);
    }
  }
}

uint8 zero[4]="0";
uint8 one[4]="1";
uint8 two[4]="2";
uint8 three[4]="3";
uint8 four[4]="4";
uint8 five[4]="5";
uint8 six[4]="6";
uint8 seven[4]="7";
uint8 eight[4]="8";
uint8 nine[4]="9";
void firstlist_txt()
{
  oled_fill(0x00);
  sprintf((char*)txt,"Welcome");
  oled_p6x8str(0,0,txt);
  sprintf((char*)txt,"1.capacity&FTM");
  oled_p6x8str(0,1,txt);
  sprintf((char*)txt,"2.6050");
  oled_p6x8str(0,2,txt);
  sprintf((char*)txt,"3.Run");
  oled_p6x8str(0,3,txt);
  sprintf((char*)txt,"4.Reset");
  oled_p6x8str(0,4,txt);
  sprintf((char*)txt,"5.Mode");
  oled_p6x8str(0,5,txt);
  sprintf((char*)txt,"6.ReadFlash");
  oled_p6x8str(0,6,txt);
  sprintf((char*)txt,"7.Save/Load");
  oled_p6x8str(0,7,txt);
}
void firstlist()
{
  uint8 i=0,j=0;
  firstlist_txt();
  systick_delay_ms(500);
  
  while(i!=1)
  {
    systick_delay_ms(150);
    if(key_check(KEY_U)==KEY_DOWN && j!=0)
    {
      j--;
    }
    else if(key_check(KEY_U)==KEY_DOWN && j==0)
    {
      j=6;
    }
    if(key_check(KEY_D)==KEY_DOWN && j!=6)
    {
      j++;
    }
    else if(key_check(KEY_D)==KEY_DOWN && j==6)
    {
      j=0;
    }
    
    Clean_Key_PositionY();
    sprintf((char*)txt,"*");
    oled_p6x8str(120,j+1,txt);
    
    if(key_check(KEY_A)==KEY_DOWN)      
      switch (j)
      {                                 
      case 0:showcapacity_FTM();                break;
      case 1:show6050();                       break;
      case 2:i=1;oled_fill(0x00);Boma_check();Load();oled_printf_float(0,0,MODE,2,0);            break;
      case 3:oled_fill(0x00);reset_menu();break;
      case 4:oled_fill(0x00);mode_menu();break;
      case 5:oled_fill(0x00);Read_flash();read_flash();break;
      case 6:oled_fill(0x00);Save();break;
      }
  }
}
void showcapacity_FTM()
{
    oled_fill(0x00);
    systick_delay_ms(500);
  
    while(1)
    {
      Capacityread();
      FTMread();
      OLED_capacity_new();
      OLED_FTM_new();
      if(key_check(KEY_A) ==  KEY_DOWN)   break;
    } 
    firstlist_txt();
} 

void show6050()
{
    oled_fill(0x00);
    systick_delay_ms(500);
    
    while(1)
    {
      GetI_Angle();
      OLED_6050_new();
      //Oscilloscope();                 //示波器
      if(key_check(KEY_A) ==  KEY_DOWN)   break;
    }
    firstlist_txt();
}
void reset_menu()
{
  uint8 i=0,j=0;
  oled_fill(0x00);
  sprintf((char*)txt,"Please RESET");
  oled_p6x8str(0,0,txt);
  sprintf((char*)txt,"1.Reset1");
  oled_p6x8str(0,1,txt);
  sprintf((char*)txt,"2.Reset2");
  oled_p6x8str(0,2,txt);
  sprintf((char*)txt,"3.Reset3");
  oled_p6x8str(0,3,txt);
  sprintf((char*)txt,"4.Reset4");
  oled_p6x8str(0,4,txt);
  sprintf((char*)txt,"5.Back");
  oled_p6x8str(0,5,txt);
  
  systick_delay_ms(500);
  
  while(i!=1)
  {
    systick_delay_ms(150);
    if(key_check(KEY_U)==KEY_DOWN && j!=0)
    {
      j--;
    }
    else if(key_check(KEY_U)==KEY_DOWN && j==0)
    {
      j=4;
    }
    if(key_check(KEY_D)==KEY_DOWN && j!=4)
    {
      j++;
    }
    else if(key_check(KEY_D)==KEY_DOWN && j==4)
    {
      j=0;
    }
    
    Clean_Key_PositionY();
    sprintf((char*)txt,"*");
    oled_p6x8str(120,j+1,txt);
    
    if(key_check(KEY_A)==KEY_DOWN)      
      switch (j)
      {                                 
      case 0:RESET(1);                break;
      case 1:RESET(2);                       break;
      case 2:RESET(3);             break;
      case 3:RESET(4);break;
      case 4:i=1;oled_fill(0x00);break;
      }
  }
  firstlist_txt();
}
void mode_menu()
{
  uint8 i=0,j=0;
  
  sprintf((char*)txt,"Please Set Mode");
  oled_p6x8str(0,0,txt);
  sprintf((char*)txt,"1.Tick");
  oled_p6x8str(0,1,txt);
  sprintf((char*)txt,"2.Music");
  oled_p6x8str(0,2,txt);
  sprintf((char*)txt,"3.Record");
  oled_p6x8str(0,3,txt);
  sprintf((char*)txt,"4.F1");
  oled_p6x8str(0,4,txt);
  sprintf((char*)txt,"5.Back");
  oled_p6x8str(0,5,txt);
  
  systick_delay_ms(500);
  
  while(i!=1)
  {
    systick_delay_ms(150);
    if(key_check(KEY_U)==KEY_DOWN && j!=0)
    {
      j--;
    }
    else if(key_check(KEY_U)==KEY_DOWN && j==0)
    {
      j=4;
    }
    if(key_check(KEY_D)==KEY_DOWN && j!=4)
    {
      j++;
    }
    else if(key_check(KEY_D)==KEY_DOWN && j==4)
    {
      j=0;
    }
    
    Clean_Key_PositionY();
    sprintf((char*)txt,"*");
    oled_p6x8str(120,j+1,txt);
    
    if(key_check(KEY_A)==KEY_DOWN)      
      switch (j)
      {                                 
      case 0:Tick();                break;
      case 1:Music();                       break;
      case 2:Record();             break;
      case 3:F1_MODE();break;
      case 4:i=1;oled_fill(0x00);break;
      }
  }
  firstlist_txt();
}
void Tick()
{
  if(tick_mark==0)
  {
    tick_mark++;
    sprintf((char*)txt,"1.Tick_Activated");
    oled_p6x8str(0,1,txt);
    sprintf((char*)txt,"Please Set Mode");
    oled_p6x8str(0,0,txt);
    sprintf((char*)txt,"1.Tick");
    oled_p6x8str(0,1,txt);
    sprintf((char*)txt,"2.Music");
    oled_p6x8str(0,2,txt);
    sprintf((char*)txt,"3.Record");
    oled_p6x8str(0,3,txt);
    sprintf((char*)txt,"4.F1");
    oled_p6x8str(0,4,txt);
    sprintf((char*)txt,"5.Back");
    oled_p6x8str(0,5,txt);
  }
  else
  {
    tick_mark=0;
    oled_fill(0x00);
    sprintf((char*)txt,"1.Tick");
    oled_p6x8str(0,1,txt);
    sprintf((char*)txt,"Please Set Mode");
    oled_p6x8str(0,0,txt);
    sprintf((char*)txt,"1.Tick");
    oled_p6x8str(0,1,txt);
    sprintf((char*)txt,"2.Music");
    oled_p6x8str(0,2,txt);
    sprintf((char*)txt,"3.Record");
    oled_p6x8str(0,3,txt);
    sprintf((char*)txt,"4.F1");
    oled_p6x8str(0,4,txt);
    sprintf((char*)txt,"5.Back");
    oled_p6x8str(0,5,txt);
  }
}
void Music()
{
  if(music_mark==0)
  {
    music_mark++;
    sprintf((char*)txt,"2.Music_Activated");
    oled_p6x8str(0,2,txt);
    sprintf((char*)txt,"Please Set Mode");
    oled_p6x8str(0,0,txt);
    sprintf((char*)txt,"1.Tick");
    oled_p6x8str(0,1,txt);
    sprintf((char*)txt,"2.Music");
    oled_p6x8str(0,2,txt);
    sprintf((char*)txt,"3.Record");
    oled_p6x8str(0,3,txt);
    sprintf((char*)txt,"4.F1");
    oled_p6x8str(0,4,txt);
    sprintf((char*)txt,"5.Back");
    oled_p6x8str(0,5,txt);
  }
  else
  {
    music_mark=0;
    oled_fill(0x00);
    sprintf((char*)txt,"2.Music");
    oled_p6x8str(0,2,txt);
    sprintf((char*)txt,"Please Set Mode");
    oled_p6x8str(0,0,txt);
    sprintf((char*)txt,"1.Tick");
    oled_p6x8str(0,1,txt);
    sprintf((char*)txt,"2.Music");
    oled_p6x8str(0,2,txt);
    sprintf((char*)txt,"3.Record");
    oled_p6x8str(0,3,txt);
    sprintf((char*)txt,"4.F1");
    oled_p6x8str(0,4,txt);
    sprintf((char*)txt,"5.Back");
    oled_p6x8str(0,5,txt);
  }
}
void Record()
{
  if(record_mark==0)
  {
    gpio_set(B9,0);
    record_mark=1;
    sprintf((char*)txt,"3.Record_Activated");
    oled_p6x8str(0,3,txt);
    sprintf((char*)txt,"Please Set Mode");
    oled_p6x8str(0,0,txt);
    sprintf((char*)txt,"1.Tick");
    oled_p6x8str(0,1,txt);
    sprintf((char*)txt,"2.Music");
    oled_p6x8str(0,2,txt);
    sprintf((char*)txt,"3.Record");
    oled_p6x8str(0,3,txt);
    sprintf((char*)txt,"4.F1");
    oled_p6x8str(0,4,txt);
    sprintf((char*)txt,"5.Back");
    oled_p6x8str(0,5,txt);
  }
  else
  {
    gpio_set(B9,1);
    record_mark=0;
    oled_fill(0x00);
    sprintf((char*)txt,"3.Record");
    oled_p6x8str(0,3,txt);
    sprintf((char*)txt,"Please Set Mode");
    oled_p6x8str(0,0,txt);
    sprintf((char*)txt,"1.Tick");
    oled_p6x8str(0,1,txt);
    sprintf((char*)txt,"2.Music");
    oled_p6x8str(0,2,txt);
    sprintf((char*)txt,"3.Record");
    oled_p6x8str(0,3,txt);
    sprintf((char*)txt,"4.F1");
    oled_p6x8str(0,4,txt);
    sprintf((char*)txt,"5.Back");
    oled_p6x8str(0,5,txt);
  }
}
void F1_MODE()
{
  if(F1_mode==0)
  {
    gpio_set(B9,0);
    F1_mode=1;
    sprintf((char*)txt,"4.F1_Warning!!!");
    oled_p6x8str(0,4,txt);
    sprintf((char*)txt,"Please Set Mode");
    oled_p6x8str(0,0,txt);
    sprintf((char*)txt,"1.Tick");
    oled_p6x8str(0,1,txt);
    sprintf((char*)txt,"2.Music");
    oled_p6x8str(0,2,txt);
    sprintf((char*)txt,"3.Record");
    oled_p6x8str(0,3,txt);
    sprintf((char*)txt,"4.F1_Warning!!!");
    oled_p6x8str(0,4,txt);
    sprintf((char*)txt,"5.Back");
    oled_p6x8str(0,5,txt);
  }
  else
  {
    gpio_set(B9,1);
    F1_mode=0;
    oled_fill(0x00);
    sprintf((char*)txt,"4.F1");
    oled_p6x8str(0,4,txt);
    sprintf((char*)txt,"Please Set Mode");
    oled_p6x8str(0,0,txt);
    sprintf((char*)txt,"1.Tick");
    oled_p6x8str(0,1,txt);
    sprintf((char*)txt,"2.Music");
    oled_p6x8str(0,2,txt);
    sprintf((char*)txt,"3.Record");
    oled_p6x8str(0,3,txt);
    sprintf((char*)txt,"4.F1");
    oled_p6x8str(0,4,txt);
    sprintf((char*)txt,"5.Back");
    oled_p6x8str(0,5,txt);
  }
}
void read_flash()
{
  uint8 i=0,j=0,k=0;
  int8 Address=0;
  oled_fill(0x00);
  sprintf((char*)txt,"Road Length");
  oled_p6x8str(0,0,txt);
  sprintf((char*)txt,"%d.%d  %d",Address,record_read_buf2[Address],record_read_buf1[Address]);
  oled_p6x8str(0,1,txt);
  sprintf((char*)txt,"%d.%d  %d",Address+1,record_read_buf2[Address+1],record_read_buf1[Address+1]);
  oled_p6x8str(0,2,txt);
  sprintf((char*)txt,"%d.%d  %d",Address+2,record_read_buf2[Address+2],record_read_buf1[Address+2]);
  oled_p6x8str(0,3,txt);
  sprintf((char*)txt,"%d.%d  %d",Address+3,record_read_buf2[Address+3],record_read_buf1[Address+3]);
  oled_p6x8str(0,4,txt);
  sprintf((char*)txt,"%d.%d  %d",Address+4,record_read_buf2[Address+4],record_read_buf1[Address+4]);
  oled_p6x8str(0,5,txt);
  sprintf((char*)txt,"%d.%d  %d",Address+5,record_read_buf2[Address+5],record_read_buf1[Address+5]);
  oled_p6x8str(0,6,txt);
  sprintf((char*)txt,"%d.%d  %d",Address+6,record_read_buf2[Address+6],record_read_buf1[Address+6]);
  oled_p6x8str(0,7,txt);
  
  systick_delay_ms(500);
  
  while(i!=1)
  {
    systick_delay_ms(150);
    if(key_check(KEY_U)==KEY_DOWN && j!=0)
    {
      j--;
    }
    else if(key_check(KEY_U)==KEY_DOWN && j==0)
    {
      oled_fill(0x00);
      Address-=7;
      if(Address<0)
      {
        Address=0;
        j=0;
      }
      j=6;
    }
    if(key_check(KEY_D)==KEY_DOWN && j!=6)
    {
      j++;
    }
    else if(key_check(KEY_D)==KEY_DOWN && j==6)
    {
      j=0;
      oled_fill(0x00);
      Address+=7;
      if(Address>56)
        Address=56;
    }
    
    if(key_check(KEY_R)==KEY_DOWN && k!=0)
    {
      oled_fill(0x00);
      k--;
    }
    else if(key_check(KEY_R)==KEY_DOWN && k==0)
    {
      oled_fill(0x00);
      k++;
    }
    
    if(key_check(KEY_L)==KEY_DOWN && k!=0)
    {
      oled_fill(0x00);
      k--;
    }
    else if(key_check(KEY_R)==KEY_DOWN && k==0)
    {
      oled_fill(0x00);
      k++;
    }
    
    Clean_Key_PositionY();
    sprintf((char*)txt,"*");
    oled_p6x8str(120,j+1,txt);
    
    if(k==0)
    {
      sprintf((char*)txt," Road Length");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"%d.%d  %d",Address,record_read_buf2[Address],record_read_buf1[Address]);
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"%d.%d  %d",Address+1,record_read_buf2[Address+1],record_read_buf1[Address+1]);
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"%d.%d  %d",Address+2,record_read_buf2[Address+2],record_read_buf1[Address+2]);
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"%d.%d  %d",Address+3,record_read_buf2[Address+3],record_read_buf1[Address+3]);
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"%d.%d  %d",Address+4,record_read_buf2[Address+4],record_read_buf1[Address+4]);
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"%d.%d  %d",Address+5,record_read_buf2[Address+5],record_read_buf1[Address+5]);
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"%d.%d  %d",Address+6,record_read_buf2[Address+6],record_read_buf1[Address+6]);
      oled_p6x8str(0,7,txt);
    }
    else
    {
      sprintf((char*)txt," Curvature Apex");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"%d.%d %d",Address,record_read_buf3[Address],record_read_buf4[Address]);
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"%d.%d %d",Address+1,record_read_buf3[Address+1],record_read_buf4[Address+1]);
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"%d.%d %d",Address+2,record_read_buf3[Address+2],record_read_buf4[Address+2]);
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"%d.%d %d",Address+3,record_read_buf3[Address+3],record_read_buf4[Address+3]);
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"%d.%d %d",Address+4,record_read_buf3[Address+4],record_read_buf4[Address+4]);
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"%d.%d %d",Address+5,record_read_buf3[Address+5],record_read_buf4[Address+5]);
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"%d.%d %d",Address+6,record_read_buf3[Address+6],record_read_buf4[Address+6]);
      oled_p6x8str(0,7,txt);
    }
  
    if(key_check(KEY_A)==KEY_DOWN)      
    {
      i=1;oled_fill(0x00);
    }
  }
  firstlist_txt();
}
void RESET(uint8 reset_num)
{
    uint8 i=0,j=0;
    uint16 set1=0,setmatrix1[4];
    oled_fill(0x00);
    sprintf((char*)txt,"Please Reset:");
    oled_p6x8str(0,0,txt);
    sprintf((char*)txt,"*");
    oled_p6x8str(j*6,2,txt);
    while(i!=1)
   {
      systick_delay_ms(150);
    if(key_check(KEY_U)==KEY_DOWN && set1!=9)
    {
      set1++;
    }
    else if(key_check(KEY_U)==KEY_DOWN && set1==9)
    {
      set1=0;
    }
    if(key_check(KEY_D)==KEY_DOWN && set1!=0)
    {
      set1--;
    }
    else if(key_check(KEY_D)==KEY_DOWN && set1==0)
    {
      set1=9;
    }
    if(key_check(KEY_L)==KEY_DOWN && j!=0)
    {
      j--;
    }
    else if(key_check(KEY_L)==KEY_DOWN && j==0)
    {
      j=3;
    }
    if(key_check(KEY_R)==KEY_DOWN && j!=3)
    {
      j++;
    }
    else if(key_check(KEY_R)==KEY_DOWN  && j==3)
    {
      j=0;
    }
    Clean_Key_PositionX();
    sprintf((char*)txt,"*");
    oled_p6x8str(j*6,2,txt);
    /*strcpy(Setchar,intswitchchar(set));
    OLED_P6x8Str(j,1,Setchar);*/
    switch(set1)
    {
    case 0:oled_p6x8str(j*6,1,zero);    break;
    case 1:oled_p6x8str(j*6,1,one);    break;
    case 2:oled_p6x8str(j*6,1,two);    break;
    case 3:oled_p6x8str(j*6,1,three);    break;
    case 4:oled_p6x8str(j*6,1,four);    break;
    case 5:oled_p6x8str(j*6,1,five);    break;
    case 6:oled_p6x8str(j*6,1,six);    break;
    case 7:oled_p6x8str(j*6,1,seven);    break;
    case 8:oled_p6x8str(j*6,1,eight);    break;
    case 9:oled_p6x8str(j*6,1,nine);    break;
    }
    setmatrix1[j]=set1;
    if(key_check(KEY_A)==KEY_DOWN)
    {
      set1=matrixswitchint(setmatrix1);
      /*if(reset!=0)
   {
     while(reset>0)
    {
      gpio_set(PTE11, 1);
      DELAY_MS(80);
      gpio_set(PTE11, 0);
      DELAY_MS(80);
      reset--;
    }
   }*/
      oled_fill(0x00);
      
    sprintf((char*)txt,"Please RESET");
    oled_p6x8str(0,0,txt);
    sprintf((char*)txt,"1.Reset1");
    oled_p6x8str(0,1,txt);
    sprintf((char*)txt,"2.Reset2");
    oled_p6x8str(0,2,txt);
    sprintf((char*)txt,"3.Reset3");
    oled_p6x8str(0,3,txt);
    sprintf((char*)txt,"4.Reset4");
    oled_p6x8str(0,4,txt);
    sprintf((char*)txt,"5.Back");
    oled_p6x8str(0,5,txt);
      
      systick_delay_ms(500);
      i=1;
    }       
   }
   switch(reset_num)
   {
   case 1:reset1=set1;break;
   case 2:reset2=set1;break;
   case 3:reset3=set1;break;
   case 4:reset4=set1;break;
   //case 5:reset5=set1;break;
   //case 6:reset6=set1;break;
   //case 7:reset7=set1;break;
   case 8:record_parameter_save1[0]=set1;break;//速度
   case 9:record_parameter_save1[1]=set1;break;//加速度
   case 10:record_parameter_save1[4]=set1;break;//出环距离
   case 11:record_parameter_save1[5]=set1;break;
   case 12:record_parameter_save1[6]=set1;break;
   case 13:record_parameter_save1[7]=set1;break;
   case 14:record_parameter_save1[8]=set1;break;
   case 15:record_parameter_save1[9]=set1;break;
   case 16:record_parameter_save1[10]=set1;break;
   case 17:record_parameter_save1[11]=set1;break;
   case 18:record_parameter_save1[12]=set1;break;
   case 19:record_parameter_save1[13]=set1;break;
   case 20:record_parameter_save1[14]=set1;break;
   case 21:record_parameter_save1[15]=set1;break;
   case 22:record_parameter_save1[16]=set1;break;
   case 23:record_parameter_save1[17]=set1;break;
   case 24:record_parameter_save1[18]=set1;break;
   case 25:record_parameter_save1[19]=set1;break;
   case 26:record_parameter_save1[20]=set1;break;
   case 27:record_parameter_save1[21]=set1;break;
   case 28:record_parameter_save1[22]=set1;break;
   case 29:record_parameter_save1[23]=set1;break;
   case 30:record_parameter_save1[24]=set1;break;
   case 31:record_parameter_save1[25]=set1;break;
   }
}
void Save()
{
    uint8 i=0,j=0,k=0;
  oled_fill(0x00);
  sprintf((char*)txt,"Please SET");
  oled_p6x8str(0,0,txt);
  sprintf((char*)txt,"1.Zhi_Velocity");
  oled_p6x8str(0,1,txt);
  sprintf((char*)txt,"2.Acceleration");
  oled_p6x8str(0,2,txt);
  sprintf((char*)txt,"3.Chuku_direction");
  oled_p6x8str(0,3,txt);
  sprintf((char*)txt,"4.Chuhuan_distance");
  oled_p6x8str(0,4,txt);
  sprintf((char*)txt,"5.Save");
  oled_p6x8str(0,5,txt);
  sprintf((char*)txt,"6.Check");
  oled_p6x8str(0,6,txt);
  sprintf((char*)txt,"7.Back");
  oled_p6x8str(0,7,txt);
  systick_delay_ms(500);
  
  while(i!=1)
  {
    systick_delay_ms(150);
    if(key_check(KEY_U)==KEY_DOWN && j!=0)
    {
      j--;
    }
    else if(key_check(KEY_U)==KEY_DOWN && j==0)
    {
      oled_fill(0x00);
      j=6;
    }
    if(key_check(KEY_D)==KEY_DOWN && j!=6)
    {
      j++;
    }
    else if(key_check(KEY_D)==KEY_DOWN && j==6)
    {
      j=0;
      oled_fill(0x00);
    }
    
    if(key_check(KEY_L)==KEY_DOWN && k!=0)
    {
      oled_fill(0x00);
      k--;
    }
    else if(key_check(KEY_L)==KEY_DOWN && k==0)
    {
       oled_fill(0x00);
       k=6;
    }
    if(key_check(KEY_R)==KEY_DOWN && k!=6)
    {
      oled_fill(0x00);
      k++;
    }
    else if(key_check(KEY_R)==KEY_DOWN && k==6)
    {
      oled_fill(0x00);
      k=0;
    }
    
    Clean_Key_PositionY();
    sprintf((char*)txt,"*");
    oled_p6x8str(120,j+1,txt);
    
    if(k==0)
    {
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Zhi_Velocity");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Acceleration");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Chuku_direction");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Chuhuan_distance");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    if(key_check(KEY_A)==KEY_DOWN)  
    {
      switch (j)
      {                                 
      case 0:RESET(8);                break;
      case 1:RESET(9);                       break;
      case 2:Set_Chuku_direction();             break;
      case 3:RESET(10);break;
      case 4:
        {
         if(flash_check(PARAMETER_FLASH_SECTOR,EXAMPLE_FLASH_SECTOR_PAGE_1))//校验当前 扇区所在页是否有数据，如果有数据则擦除整个扇区
         {
            status2 = flash_erase_sector(PARAMETER_FLASH_SECTOR);//擦除扇区，如果扇区已经有数据则必须擦除扇区之后才能再次写入新的数据
            if(status2)  while(1);//擦除失败
         }
         status2 = flash_page_program(PARAMETER_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_1, record_parameter_save1, FLASH_SAVE_NUM);
         if(status2)  while(1);//写入失败
        break;
        }
      case 5:Check();break;
      case 6:i=1;oled_fill(0x00);break;
      }
      oled_fill(0x00);
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Zhi_Velocity");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Acceleration");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Chuku_direction");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Chuhuan_distance");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    }
    }
    else if(k==1)
    {
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Wan_P");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Wan_D");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Zhi_P");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Zhi_D");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    if(key_check(KEY_A)==KEY_DOWN)  
    {
      switch (j)
      {                                 
      case 0:RESET(11);                break;
      case 1:RESET(12);                       break;
      case 2:RESET(13);             break;
      case 3:RESET(14);break;
      case 4:
        {
         if(flash_check(PARAMETER_FLASH_SECTOR,EXAMPLE_FLASH_SECTOR_PAGE_1))//校验当前 扇区所在页是否有数据，如果有数据则擦除整个扇区
         {
            status2 = flash_erase_sector(PARAMETER_FLASH_SECTOR);//擦除扇区，如果扇区已经有数据则必须擦除扇区之后才能再次写入新的数据
            if(status2)  while(1);//擦除失败
         }
         status2 = flash_page_program(PARAMETER_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_1, record_parameter_save1, FLASH_SAVE_NUM);
         if(status2)  while(1);//写入失败
        break;
        }
      case 5:Check();break;
      case 6:i=1;oled_fill(0x00);break;
      }
      oled_fill(0x00);
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Wan_P");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Wan_D");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Zhi_P");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Zhi_D");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    }
    }
    else if(k==2)
    {
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Cross_address");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Ru_deviate_S");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Ru_deviate_M");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Ru_deviate_L");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    if(key_check(KEY_A)==KEY_DOWN)  
    {
      switch (j)
      {                                 
      case 0:RESET(15);                break;
      case 1:RESET(16);                       break;
      case 2:RESET(17);             break;
      case 3:RESET(18);break;
      case 4:
        {
         if(flash_check(PARAMETER_FLASH_SECTOR,EXAMPLE_FLASH_SECTOR_PAGE_1))//校验当前 扇区所在页是否有数据，如果有数据则擦除整个扇区
         {
            status2 = flash_erase_sector(PARAMETER_FLASH_SECTOR);//擦除扇区，如果扇区已经有数据则必须擦除扇区之后才能再次写入新的数据
            if(status2)  while(1);//擦除失败
         }
         status2 = flash_page_program(PARAMETER_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_1, record_parameter_save1, FLASH_SAVE_NUM);
         if(status2)  while(1);//写入失败
        break;
        }
      case 5:Check();break;
      case 6:i=1;oled_fill(0x00);break;
      }
      oled_fill(0x00);
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Cross_address");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Ru_deviate_S");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Ru_deviate_M");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Ru_deviate_L");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    }
    }
    else if(k==3)
    {
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Void");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Chu_deviate_S");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Chu_deviate_M");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Chu_deviate_L");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    if(key_check(KEY_A)==KEY_DOWN)  
    {
      switch (j)
      {                                 
      case 0:                break;
      case 1:RESET(19);                       break;
      case 2:RESET(20);             break;
      case 3:RESET(21);break;
      case 4:
        {
         if(flash_check(PARAMETER_FLASH_SECTOR,EXAMPLE_FLASH_SECTOR_PAGE_1))//校验当前 扇区所在页是否有数据，如果有数据则擦除整个扇区
         {
            status2 = flash_erase_sector(PARAMETER_FLASH_SECTOR);//擦除扇区，如果扇区已经有数据则必须擦除扇区之后才能再次写入新的数据
            if(status2)  while(1);//擦除失败
         }
         status2 = flash_page_program(PARAMETER_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_1, record_parameter_save1, FLASH_SAVE_NUM);
         if(status2)  while(1);//写入失败
        break;
        }
      case 5:Check();break;
      case 6:i=1;oled_fill(0x00);break;
      }
      oled_fill(0x00);
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Void");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Chu_deviate_S");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Chu_deviate_M");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Chu_deviate_L");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    }
    }
    else if(k==4)
      {
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Ru_record_distance");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Ru_record_S");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Ru_record_M");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Ru_record_L");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    if(key_check(KEY_A)==KEY_DOWN)  
    {
      switch (j)
      {                                 
      case 0:RESET(28);                break;
      case 1:RESET(22);                       break;
      case 2:RESET(23);             break;
      case 3:RESET(24);break;
      case 4:
        {
         if(flash_check(PARAMETER_FLASH_SECTOR,EXAMPLE_FLASH_SECTOR_PAGE_1))//校验当前 扇区所在页是否有数据，如果有数据则擦除整个扇区
         {
            status2 = flash_erase_sector(PARAMETER_FLASH_SECTOR);//擦除扇区，如果扇区已经有数据则必须擦除扇区之后才能再次写入新的数据
            if(status2)  while(1);//擦除失败
         }
         status2 = flash_page_program(PARAMETER_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_1, record_parameter_save1, FLASH_SAVE_NUM);
         if(status2)  while(1);//写入失败
        break;
        }
      case 5:Check();break;
      case 6:i=1;oled_fill(0x00);break;
      }
      oled_fill(0x00);
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Ru_record_distance");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Ru_record_S");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Ru_record_M");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Ru_record_L");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    }
    }
    else if(k==5)
    {
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Chu_record_distance");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Chu_record_S");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Chu_record_M");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Chu_record_L");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    if(key_check(KEY_A)==KEY_DOWN)  
    {
      switch (j)
      {                                 
      case 0:RESET(29);                break;
      case 1:RESET(25);                       break;
      case 2:RESET(26);             break;
      case 3:RESET(27);break;
      case 4:
        {
         if(flash_check(PARAMETER_FLASH_SECTOR,EXAMPLE_FLASH_SECTOR_PAGE_1))//校验当前 扇区所在页是否有数据，如果有数据则擦除整个扇区
         {
            status2 = flash_erase_sector(PARAMETER_FLASH_SECTOR);//擦除扇区，如果扇区已经有数据则必须擦除扇区之后才能再次写入新的数据
            if(status2)  while(1);//擦除失败
         }
         status2 = flash_page_program(PARAMETER_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_1, record_parameter_save1, FLASH_SAVE_NUM);
         if(status2)  while(1);//写入失败
        break;
        }
      case 5:Check();break;
      case 6:i=1;oled_fill(0x00);break;
      }
      oled_fill(0x00);
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Chu_record_distance");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Chu_record_S");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Chu_record_M");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Chu_record_L");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    }
    }
    else
    {
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Chuanqiu_distance");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Wan_Velocity");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Void");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Void");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    if(key_check(KEY_A)==KEY_DOWN)  
    {
      switch (j)
      {                                 
      case 0:RESET(30);                break;
      case 1:RESET(31);                       break;
      case 2:             break;
      case 3:break;
      case 4:
        {
         if(flash_check(PARAMETER_FLASH_SECTOR,EXAMPLE_FLASH_SECTOR_PAGE_1))//校验当前 扇区所在页是否有数据，如果有数据则擦除整个扇区
         {
            status2 = flash_erase_sector(PARAMETER_FLASH_SECTOR);//擦除扇区，如果扇区已经有数据则必须擦除扇区之后才能再次写入新的数据
            if(status2)  while(1);//擦除失败
         }
         status2 = flash_page_program(PARAMETER_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE_1, record_parameter_save1, FLASH_SAVE_NUM);
         if(status2)  while(1);//写入失败
        break;
        }
      case 5:Check();break;
      case 6:i=1;oled_fill(0x00);break;
      }
      oled_fill(0x00);
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Chuanqiu_distance");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Wan_Velocity");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Void");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Void");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
    }
    }
      
  }
  firstlist_txt();
}
void Set_Chuku_direction()
{
  uint8 i=0;
  oled_fill(0x00);
  sprintf((char*)txt,"Please set direction");
  oled_p6x8str(0,0,txt);
  
  systick_delay_ms(500);
  
  while(i!=1)
  {
    systick_delay_ms(150);
    if(key_check(KEY_U)==KEY_DOWN)
    {
      oled_fill(0x00);
      sprintf((char*)txt,"Please set direction");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"Chuku_disabled");
      oled_p6x8str(0,1,txt);
      record_parameter_save1[2]=0;
    }
    if(key_check(KEY_D)==KEY_DOWN)
    {
      oled_fill(0x00);
      sprintf((char*)txt,"Please set direction");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"Chuku_disabled");
      oled_p6x8str(0,1,txt);
      record_parameter_save1[2]=0;
    }
    
    if(key_check(KEY_R)==KEY_DOWN)
    {
      oled_fill(0x00);
      sprintf((char*)txt,"Please set direction");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"Chuku_Right");
      oled_p6x8str(0,1,txt);
      record_parameter_save1[2]=2;
      record_parameter_save1[3]=1;
    }
    if(key_check(KEY_L)==KEY_DOWN)
    {
      oled_fill(0x00);
      sprintf((char*)txt,"Please set direction");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"Chuku_Left");
      oled_p6x8str(0,1,txt);
      record_parameter_save1[2]=2;
      record_parameter_save1[3]=0;
    }
  
    if(key_check(KEY_A)==KEY_DOWN)      
    {
      i=1;oled_fill(0x00);
    }
  }
      oled_fill(0x00);
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Velocity");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Acceleration");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Chuku_direction");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Chuhuan_distance");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
}
void Check()
{
  uint8 i=0,k=0;
  oled_fill(0x00);
  sprintf((char*)txt,"Velocity:%d",record_parameter_save1[0]);
  oled_p6x8str(0,0,txt);
  sprintf((char*)txt,"Acceleration:%d",record_parameter_save1[1]);
  oled_p6x8str(0,1,txt);
  sprintf((char*)txt,"Chuku:%d",record_parameter_save1[2]);
  oled_p6x8str(0,2,txt);
  sprintf((char*)txt,"Chuku_direction:%d",record_parameter_save1[3]);
  oled_p6x8str(0,3,txt);
  sprintf((char*)txt,"Chuhuan_dis:%d*10^3",record_parameter_save1[4]);
  oled_p6x8str(0,4,txt);
  
  systick_delay_ms(500);
  
  while(i!=1)
  {  
    systick_delay_ms(150);
    if(key_check(KEY_A)==KEY_DOWN)      
    {
      i=1;oled_fill(0x00);
    }
    if(key_check(KEY_L)==KEY_DOWN && k!=0)
    {
      oled_fill(0x00);
      k--;
    }
    else if(key_check(KEY_L)==KEY_DOWN && k==0)
    {
       oled_fill(0x00);
       k=4;
    }
    if(key_check(KEY_R)==KEY_DOWN && k!=4)
    {
      oled_fill(0x00);
      k++;
    }
    else if(key_check(KEY_R)==KEY_DOWN && k==4)
    {
      oled_fill(0x00);
      k=0;
    }
    if(k==0)
    {
      sprintf((char*)txt,"Zhi_Velocity:%d",record_parameter_save1[0]);
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"Wan_Velocity:%d",record_parameter_save1[25]);
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"Acceleration:%d",record_parameter_save1[1]);
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"Chuku:%d",record_parameter_save1[2]);
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"Chuku_direction:%d",record_parameter_save1[3]);
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"Chuhuan_dis:%d*10^3",record_parameter_save1[4]);
      oled_p6x8str(0,5,txt);
    }
    else if(k==1)
    {
      sprintf((char*)txt,"Wan_P:%d",record_parameter_save1[5]);
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"Wan_D:%d",record_parameter_save1[6]);
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"Zhi_P:%d",record_parameter_save1[7]);
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"Zhi_D:%d",record_parameter_save1[8]);
      oled_p6x8str(0,3,txt);
    }
    else if(k==2)
    {
      sprintf((char*)txt,"Cross_address:%d",record_parameter_save1[9]);
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"R_deviate_S:%d",record_parameter_save1[10]);
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"R_deviate_M:%d/1000",record_parameter_save1[11]);
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"R_deviate_L:%d/1000",record_parameter_save1[12]);
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"Ch_deviate_S:%d/1000",record_parameter_save1[13]);
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"Ch_deviate_M:%d/1000",record_parameter_save1[14]);
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"Ch_deviate_L:%d/1000",record_parameter_save1[15]);
      oled_p6x8str(0,6,txt);
    }
    else if(k==3)
    {
      sprintf((char*)txt,"R_record_dis:%d",record_parameter_save1[22]);
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"R_record_S:%d",record_parameter_save1[16]);
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"R_record_M:%d/1000",record_parameter_save1[17]);
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"R_record_L:%d/1000",record_parameter_save1[18]);
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"Ch_record_dis:%d*10",record_parameter_save1[23]);
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"Ch_record_S:%d/1000",record_parameter_save1[19]);
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"Ch_record_M:%d/1000",record_parameter_save1[20]);
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"Ch_record_L:%d/1000",record_parameter_save1[21]);
      oled_p6x8str(0,7,txt);
    }
    else
      {
      sprintf((char*)txt,"Chuanqiu_dis:%d",record_parameter_save1[24]);
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"Void:%d",record_parameter_save1[16]);
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"Void:%d/1000",record_parameter_save1[17]);
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"Void:%d/1000",record_parameter_save1[18]);
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"Void:%d*10",record_parameter_save1[23]);
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"Void:%d/1000",record_parameter_save1[19]);
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"Void:%d/1000",record_parameter_save1[20]);
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"v:%d/1000",record_parameter_save1[21]);
      oled_p6x8str(0,7,txt);
    }
  }
      sprintf((char*)txt,"Please SET");
      oled_p6x8str(0,0,txt);
      sprintf((char*)txt,"1.Zhi_Velocity");
      oled_p6x8str(0,1,txt);
      sprintf((char*)txt,"2.Acceleration");
      oled_p6x8str(0,2,txt);
      sprintf((char*)txt,"3.Chuku_direction");
      oled_p6x8str(0,3,txt);
      sprintf((char*)txt,"4.Chuhuan_distance");
      oled_p6x8str(0,4,txt);
      sprintf((char*)txt,"5.Save");
      oled_p6x8str(0,5,txt);
      sprintf((char*)txt,"6.Check");
      oled_p6x8str(0,6,txt);
      sprintf((char*)txt,"7.Back");
      oled_p6x8str(0,7,txt);
}