#include "music.h"

uint16 count=0;
//sbit _Speak =P2^3 ; //讯响器控制脚
uint8 T[49][2]={{0,0},{0xF8,0x8B},{0xF8,0xF2},{0xF9,0x5B},{0xF9,0xB7},{0xFA,0x14},{0xFA,0x66},{0xFA,0xB9},{0xFB,0x03},{0xFB,0x4A},{0xFB,0x8F},{0xFB,0xCF},{0xFC,0x0B},
{0xFC,0x43},{0xFC,0x78},{0xFC,0xAB},{0xFC,0xDB},{0xFD,0x08},{0xFD,0x33},{0xFD,0x5B},{0xFD,0x81},{0xFD,0xA5},{0xFD,0xC7},{0xFD,0xE7},{0xFE,0x05},
{0xFE,0x21},{0xFE,0x3C},{0xFE,0x55},{0xFE,0x6D},{0xFE,0x84},{0xFE,0x99},{0xFE,0xAD},{0xFE,0xC0},{0xFE,0x02},{0xFE,0xE3},{0xFE,0xF3},{0xFF,0x02},
{0xFF,0x10},{0xFF,0x1D},{0xFF,0x2A},{0xFF,0x36},{0xFF,0x42},{0xFF,0x4C},{0xFF,0x56},{0xFF,0x60},{0xFF,0x69},{0xFF,0x71},{0xFF,0x79},{0xFF,0x81}
};
uint8 SONG_TONE[]={212,212,1904,212,159,169,212,212,190,212,142,159,212,212,106,126,159,169,190,119,119,126,159,142,159,0}; //生日快乐歌的音符频率表，不同频率由不同的延时来决定 

uint8 SONG_LONG[]={9,3,12,12,12,24,9,3,12,12,12,24, 

9,3,12,12,12,12,12,9,3,12,12,12,24,0};

uint8 PU[49][2]={{0,0},
{0xF8,0x8B},{0xF8,0xF2},{0xF9,0x5B},{0xF9,0xB7},{0xFA,0x14},{0xFA,0x66},{0xFA,0xB9},{0xFB,0x03},{0xFB,0x4A},{0xFB,0x8F},{0xFB,0xCF},{0xFC,0x0B},
{0xFC,0x43},{0xFC,0x78},{0xFC,0xAB},{0xFC,0xDB},{0xFD,0x08},{0xFD,0x33},{0xFD,0x5B},{0xFD,0x81},{0xFD,0xA5},{0xFD,0xC7},{0xFD,0xE7},{0xFE,0x05},
{0xFE,0x21},{0xFE,0x3C},{0xFE,0x55},{0xFE,0x6D},{0xFE,0x84},{0xFE,0x99},{0xFE,0xAD},{0xFE,0xC0},{0xFE,0x02},{0xFE,0xE3},{0xFE,0xF3},{0xFF,0x02},
{0xFF,0x10},{0xFF,0x1D},{0xFF,0x2A},{0xFF,0x36},{0xFF,0x42},{0xFF,0x4C},{0xFF,0x56},{0xFF,0x60},{0xFF,0x69},{0xFF,0x71},{0xFF,0x79},{0xFF,0x81}
};
uint16 Ddiao[]={83,92,98,110,124,147,165,185,196,220,247,277,294,330,370,392,440,494,554,587,659,740,784,880,988,1109,1175,1318,1480,1568,1760,1996,2218,2350,2636,2960,3136,3520,3992,4436,4700,5272};
uint8 CANON[][2]={{17,8},{16,8},{15,8},{14,8},{13,8},{12,8},{13,8},{14,8},{15,8},{14,8},{13,8},{12,8},{11,8},{10,8},{11,8},{9,8},{15,2},{14,2},{15,2},{8,2},{7,2},{12,2},{9,2},
{10,2},{8,2},{15,2},{14,2},{13,2},{14,2},{17,2},{19,2},{20,2},{18,2},{17,2},{16,2},{18,2},{18,2},{17,2},{15,2},{14,2},{13,2},{12,2},{11,2},{10,2},{9,2},{11,2},{10,2},{9,2},
{8,2},{9,2},{10,2},{11,2},{12,2},{9,2},{12,2},{11,2},{10,2},{13,2},{12,2},{11,2},{12,2},{11,2},{10,2},{9,2},{8,2},{6,2},{13,2},{14,2},{15,2},{14,2},{13,2},{12,2},{11,2},{10,2},{9,2},
{13,2},{12,2},{13,2},{12,2},{11,2},{10,4},{3,4},{2,8},{0xFF,8}};
uint8 hundouluo[][2]={{0,4},{34,2},{32,2},{29,2},{27,2},{29,2},{27,2},{25,2},{24,2},{25,2},{24,2},{22,2},{20,2},{22,2},{15,2},{17,2},{20,2},{22,16},{22,4},{29,2},
{0,2},{27,2},{29,4},{30,2},{32,24},{32,16},{22,16},{22,4},{29,2},{0,2},{27,2},{29,4},{30,2},{25,24},{25,16},{22,16},{22,4},{29,2},{0,2},
{27,2},{29,4},{30,2},{32,24},{32,16},{22,16},{22,4},{29,2},{0,2},{27,2},{29,4},{30,2},{25,24},{25,16},{29,2},{0,2},{29,6},{30,6},{32,2},
{0,2},{32,2},{0,8},{0,2},{30,2},{0,2},{30,6},{32,6},{34,2},{0,2},{34,2},{0,2},{32,2},{34,6},{29,2},{0,2},{29,6},{30,6},{32,2},{0,2},{32,2},
{0,8},{0,2},{30,2},{0,2},{30,6},{32,6},{34,8},{39,8},{36,2},{34,2},{42,2},{41,2},{39,2},{37,2},{36,2},{34,2},{36,2},{34,2},{30,2},{29,2},{30,2},
{29,2},{27,2},{25,2},{17,6},{0,2},{17,2},{18,4},{20,2},{24,2},{0,2},{22,2},{0,2},{20,2},{18,4},{15,2},{17,24},{17,16},{13,8},{15,2},{17,6},{18,8},{20,2},{22,6},{24,8},{25,2},{27,4},{29,2},{30,8},{29,2},{17,2},{20,2},{22,2},{24,2},{22,2},{24,2},{17,2},{0,2},{17,2},{20,2},{22,2},{24,4},{27,2},{25,2},{27,2},{25,4},{22,2},{24,24},{24,16},
{13,8},{15,2},{17,6},{18,8},{20,2},{22,6},{24,2},{24,2},{24,2},{0,2},{24,2},{25,4},{27,2},{30,8},{29,8},{27,2},{27,2},{27,2},{27,2},{0,4},{27,2},
{27,2},{27,2},{27,2},{0,2},{27,2},{0,2},{25,6},{0xFF,0xFF}};
uint8 daojianrumeng[][2]={{0,4},
{16,12},{23,16},{21,4},{23,4},{21,4},{19,12},{0,8},{19,4},{19,4},{19,4},{18,4},{16,8},{14,4},{16,16},{0,16},
{16,12},{28,16},{26,4},{28,4},{26,4},{23,8},{0,16},	
{21,4},{21,4},{21,4},{21,4},{19,8},{16,4},{18,16},{0,12},
{16,12},{23,16},{21,4},{23,4},{21,4},{19,8},{0,16},
{19,4},{19,4},{19,4},{18,4},{16,8},{14,4},{16,16},{0,12},
{16,12},{28,16},{26,4},{28,4},{26,4},{23,16},{0,12},
{28,4},{28,4},{28,4},{30,4},{28,8},{26,4},{28,24},{0,12},

{28,4},{28,2},{26,2},{23,4},{23,4},{28,4},{28,2},{26,2},{28,4},{31,4},{26,6},{23,2},{26,4},{28,4},{26,16},
{21,4},{21,2},{19,2},{16,4},{16,4},{21,4},{21,2},{19,2},{21,4},{23,4},{19,6},{16,2},{19,4},{16,4},{14,16},
{28,4},{28,2},{26,2},{23,4},{23,4},{28,4},{28,2},{26,2},{28,4},{28,4},
{31,4},{31,2},{28,2},{26,4},{26,4},{31,4},{31,2},{26,2},{31,4},{31,4},{0,4},{31,4},{31,4},{31,4},{30,4},{28,8},{26,4},{28,24},

{28,4},{28,2},{26,2},{23,4},{23,4},{28,4},{28,2},{26,2},{28,4},{31,4},{26,6},{23,2},{26,4},{28,4},{26,16},
{21,4},{21,2},{19,2},{16,4},{16,4},{21,4},{21,2},{19,2},{21,4},{23,4},{19,6},{16,2},{19,4},{16,4},{14,16},
{28,4},{28,2},{26,2},{23,4},{23,4},{28,4},{28,2},{26,2},{28,4},{28,4},
{31,4},{31,2},{28,2},{26,4},{26,4},{31,4},{31,2},{26,2},{31,4},{31,4},{0,4}, {31,4},{31,4},{31,4},{30,4},{28,8},{26,4},{28,20},	  
{0,4},{31,4},{31,4},{31,4},{30,4},{28,8},{26,4},{28,24},
{0xFF,0xFF}};
uint8 shenrikuaile[][2]={{0,4},
{17,6},{17,2},{19,8},{17,8},{22,8},{21,16},
{17,6},{17,2},{19,8},{17,8},{24,8},{22,16},{17,6},{17,2},{29,8},{26,8},{22,8},{21,8},{19,8},{27,6},{27,2},{26,8},{22,8},{24,8},{22,16},
{0xFF,0xFF}};
uint8 yiluxiangbei[][2]={{0,4},
{15,4},{15,4},{15,4},{17,4},{17,4},{20,4},{20,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{8,2},{10,2},
{13,4},{15,4},{15,4},{15,4},{17,4},{17,4},{20,4},{20,4},{20,4},{20,4},{18,4},{17,4},{18,4},{18,4},{18,4},{18,4},
{17,4},{0,8},{0,4},{18,4},{18,4},{17,4},{15,4},{13,4},{15,4},{17,4},{17,16},{0,8},{0,8},{0,4},{18,4},{18,4},{17,4},
{15,4},{13,4},{15,4},{13,4},{13,8},{0,4},{13,4},{17,4},{20,4},{20,6},{13,2},{13,8},{22,4},{20,4},{20,4},{17,4},{20,6},
{13,2},{13,8},{22,4},{20,4},{20,4},{17,4},{17,4},{15,4},{15,8},{22,4},{20,4},{20,4},{17,4},{17,6},{15,2},{15,8},{15,8},{0,8},
{0xFF,0xFF}};
uint8 chaojimali[][2]={{0,4},
{17,4},{17,4},{17,4},{13,2},{17,2},{18,8},{20,4},{13,4},{8,4},{12,8},{13,4},{12,8},{8,4},{6,8},{5,4},{6,8},{10,4},{12,4},{12,4},{10,4},{8,4},{17,4},{20,2},{22,2},{20,8},{18,2},{20,2},
{0xFF,0xFF}};

/*-------------------------------------------------
功能:歌曲播放子dao程序i为播放哪一段曲目
-------------------------------------------------*/
int ying=0;
uint8 m;
void Play_Song(uint8 i)
{
  uint8 m;
  uint16 n;
  uint16 address;
  count = 0; //中断计数器清0
  m=CANON[ying][0];
  
  //m=200;
  n=65535-(T[m][0]<<8)+T[m][1];
  if(m==0x00 ) //休止符
    systick_delay_us(80*CANON[ying][1]);
  else if(m == 0xFF ) //歌曲结束符
  {
    systick_delay_ms(1000);
    ying=0;
  }
  else
  {
    while(1)
    {
      fast_gpio_toggle(D15);
      systick_delay_us(n/2);
      if(count==80*CANON[ying][1])
        break;
    }  
  }
  systick_delay_ms(1);
  ying++;
  count=0;
}
void Play_Song2(uint8 i)
{
  uint8 m;
  uint16 n;
  uint16 address;
  count = 0; //中断计数器清0
  m=hundouluo[ying][0];
  
  //m=200;
  n=(65535-(T[m][0]<<8)+T[m][1])/4;
  //n=8*newtonSqrt(Ddiao[m]);
  if(m==0x00 ) //休止符
  {
    while(1)
    {
      if(count==80*hundouluo[ying][1])
        break;
    }
  }
  else if(m == 0xFF ) //歌曲结束符
  {
    systick_delay_ms(1000);
    ying=0;
  }
  else
  {
    pwm_freq(PWM1_MODULE1_CHB_D15, n,2000);
    while(1)
    {
      if(count==80*hundouluo[ying][1])
        break;
    }  
  }
  systick_delay_ms(10);
  ying++;
  count=0;
}