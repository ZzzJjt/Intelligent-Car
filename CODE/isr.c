
 
/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            isr
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ3184284598)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        ADS v1.2.2
 * @Target core     TC364DP
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-11-23
 ********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"

//��isr.c���жϺ�������������ĵڶ��������̶�Ϊ0���벻Ҫ���ģ���ʹ����CPU1�����ж�Ҳ��Ҫ���ģ���ҪCPU1�����ж�ֻ��Ҫ��isr_config.h���޸Ķ�Ӧ�ĺ궨�弴��

//PIT�жϺ���  ʾ��
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    enableInterrupts();//�����ж�Ƕ��
    PIT_CLEAR_FLAG(CCU6_0, PIT_CH0);
    //�������
    Motor_Control();
}

IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    enableInterrupts();//�����ж�Ƕ��
    PIT_CLEAR_FLAG(CCU6_0, PIT_CH1);

    systick_start(STM1);//ʹ��STM1 ���м�ʱ

    if(mt9v03x_finish_flag == 0)
    {
        //�������
        Servocontrol();
    }
    runtime = systick_getval_us(STM1);         //��ȡSTM1��ʱʱ��(��ʼ��ʱ�����ڵ�ʱ��)
    if(runtime>runtimemax)runtimemax=runtime;  //��¼���ʱ��

}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    enableInterrupts();//�����ж�Ƕ��
    PIT_CLEAR_FLAG(CCU6_1, PIT_CH0);

     //�Ƕȼ��
    Angle_Detection();
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    enableInterrupts();//�����ж�Ƕ��
    PIT_CLEAR_FLAG(CCU6_1, PIT_CH1);

}



//I/O���ж�
IFX_INTERRUPT(eru_ch0_ch4_isr, 0, ERU_CH0_CH4_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    if(GET_GPIO_FLAG(ERU_CH0_REQ4_P10_7))//ͨ��0�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH0_REQ4_P10_7);
    }

    if(GET_GPIO_FLAG(ERU_CH4_REQ13_P15_5))//ͨ��4�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(eru_ch1_ch5_isr, 0, ERU_CH1_CH5_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    if(GET_GPIO_FLAG(ERU_CH1_REQ5_P10_8))//ͨ��1�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH1_REQ5_P10_8);
    }

    if(GET_GPIO_FLAG(ERU_CH5_REQ1_P15_8))//ͨ��5�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH5_REQ1_P15_8);
    }
}

//��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
//IFX_INTERRUPT(eru_ch2_ch6_isr, 0, ERU_CH2_CH6_INT_PRIO)
//{
//  enableInterrupts();//�����ж�Ƕ��
//  if(GET_GPIO_FLAG(ERU_CH2_REQ7_P00_4))//ͨ��2�ж�
//  {
//      CLEAR_GPIO_FLAG(ERU_CH2_REQ7_P00_4);
//
//  }
//  if(GET_GPIO_FLAG(ERU_CH6_REQ9_P20_0))//ͨ��6�ж�
//  {
//      CLEAR_GPIO_FLAG(ERU_CH6_REQ9_P20_0);
//
//  }
//}



IFX_INTERRUPT(eru_ch3_ch7_isr, 0, ERU_CH3_CH7_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    if(GET_GPIO_FLAG(ERU_CH3_REQ6_P02_0))//ͨ��3�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH3_REQ6_P02_0);
        if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_vsync();
        else if (CAMERA_BIN_UART  == camera_type)   ov7725_uart_vsync();
        else if (CAMERA_BIN       == camera_type)   ov7725_vsync();

    }
    if(GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1))//ͨ��7�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH7_REQ16_P15_1);

    }
}

IFX_INTERRUPT(dma_ch5_isr, 0, ERU_DMA_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��

    if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_dma();
    else if (CAMERA_BIN_UART  == camera_type)   ov7725_uart_dma();
    else if (CAMERA_BIN       == camera_type)   ov7725_dma();
}

//�����жϺ���  ʾ��
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart0_handle);

    //systick_start(STM1);//ʹ��STM1 ���м�ʱ

//    uart_getchar(UART_0,&data_TFmini);
//    for(int i = 0; i <= 7; i++) TFminidate[i] = TFminidate[i+1];
//    TFminidate[8] = data_TFmini;
//    for(int i=0;i<=7;i++)if(TFminidate[i]==89 && TFminidate[i+1]==89) count_UART0++;
//    if(count_UART0==1)
//    {
//        if(TFminidate[0]==89 && TFminidate[1]==89) distance=TFminidate[2]+TFminidate[3]*16*16;
//    }
//    count_UART0 = 0;

    //runtime = systick_getval_us(STM1); //��ȡSTM1��ʱʱ��(��ʼ��ʱ�����ڵ�ʱ��)

//    uart_getchar(UART_0,&data_TFmini);
//    TFminidate[count_UART0]=data_TFmini;
//    count_UART0++;
//    if(count_UART0==9)
//    {
//       for(i=0;i<=8;i++)
//       {
//            if(TFminidate[i]==89 && TFminidate[i+1]==89)
//            {
//                TFminiflag=i;
//                break;
//            }
//       }
//       if( TFminidate[0]==89 && TFminidate[8]==89)
//       {
//           TFminiflag=8;
//       }
//       for(i=0;i<=8;i++)
//       {
//            TFminidate_true[i]=TFminidate[TFminiflag];
//            TFminiflag++;
//            if(TFminiflag==9)TFminiflag=0;
//       }
//       distance=TFminidate_true[2]+TFminidate_true[3]*16*16;
//       count_UART0=0;
//    }
}
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}

//����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_uart_callback();
    else if (CAMERA_BIN_UART  == camera_type)   ov7725_uart_callback();
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}

//����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart2_handle);

    wireless_uart_callback();
    // �ֲ� �ⲿ���� ��������
    extern uint8 wireless_rx_buffer;
    extern uint8 get_data;
    extern uint8 get_flag;

    // ��ȡ���ߴ��ڵ����� ������λ���ձ�־
    get_data = wireless_rx_buffer;
    get_flag = 1;
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}



IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart3_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
