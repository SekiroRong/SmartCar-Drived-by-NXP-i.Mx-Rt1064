#include "ultrasonic.h"

uint8 example_rx_buffer;
lpuart_transfer_t   example_receivexfer;
lpuart_handle_t     example_g_lpuartHandle;

static uint8 uart_send;
static uint8 uart_data;
uint16 distance = 0;	//超声波接收端测得的距离值
uint8 dat[3];
uint8 num;



extern void example_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    if(kStatus_LPUART_RxIdle == status)
    {
        //数据已经被写入到了 之前返回的BUFF中
        //本例程使用的BUFF为 example_rx_buffer
        uart_data = example_rx_buffer;//将数据取出
		
		if(num == 3)					//接收完成，开始处理数据
		{
			num = 0;
			distance = dat[1]<<8 | dat[2];	//得到超声波模块测出的距离
		}
		if(num != 0)
		{
			dat[num] = uart_data;
			num ++;
		}
		if(uart_data == 0xa5)
		{
			num ++;
		}
    }
    
    handle->rxDataSize = example_receivexfer.dataSize;  //还原缓冲区长度
    handle->rxData = example_receivexfer.data;          //还原缓冲区地址
}
