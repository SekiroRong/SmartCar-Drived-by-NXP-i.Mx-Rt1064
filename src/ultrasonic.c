#include "ultrasonic.h"

uint8 example_rx_buffer;
lpuart_transfer_t   example_receivexfer;
lpuart_handle_t     example_g_lpuartHandle;

static uint8 uart_send;
static uint8 uart_data;
uint16 distance = 0;	//���������ն˲�õľ���ֵ
uint8 dat[3];
uint8 num;



extern void example_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    if(kStatus_LPUART_RxIdle == status)
    {
        //�����Ѿ���д�뵽�� ֮ǰ���ص�BUFF��
        //������ʹ�õ�BUFFΪ example_rx_buffer
        uart_data = example_rx_buffer;//������ȡ��
		
		if(num == 3)					//������ɣ���ʼ��������
		{
			num = 0;
			distance = dat[1]<<8 | dat[2];	//�õ�������ģ�����ľ���
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
    
    handle->rxDataSize = example_receivexfer.dataSize;  //��ԭ����������
    handle->rxData = example_receivexfer.data;          //��ԭ��������ַ
}
