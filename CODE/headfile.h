/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		headfile
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC364DP
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-11-23
 ********************************************************************************************************************/
 
#ifndef _headfile_h
#define _headfile_h




#include "SEEKFREE_PRINTF.h"

#include "zf_assert.h"
#include "stdio.h"
#include "math.h"
//�ٷ�ͷ�ļ�
#include "ifxAsclin_reg.h"
#include "SysSe/Bsp/Bsp.h"
#include "IfxCcu6_Timer.h"
#include "IfxScuEru.h"
//------��ɿƼ���Ƭ����������ͷ�ļ�
#include "zf_gpio.h"
#include "zf_gtm_pwm.h"
#include "zf_uart.h"
#include "zf_ccu6_pit.h"
#include "zf_stm_systick.h"
#include "zf_spi.h"
#include "zf_eru.h"
#include "zf_eru_dma.h"
#include "zf_vadc.h"
#include "zf_gpt12.h"
#include "zf_eeprom.h"
//------��ɿƼ���Ʒ����ͷ�ļ�
#include "SEEKFREE_18TFT.h"
#include "SEEKFREE_FONT.h"
#include "SEEKFREE_FUN.h"
#include "SEEKFREE_IIC.h"
#include "SEEKFREE_IPS114_SPI.h"
#include "SEEKFREE_OLED.h"
#include "SEEKFREE_VIRSCO.h"
#include "SEEKFREE_MT9V03X.h"
#include "SEEKFREE_ICM20602.h"
#include "SEEKFREE_MPU6050.h"
#include "SEEKFREE_MMA8451.h"
#include "SEEKFREE_L3G4200D.h"
#include "SEEKFREE_WIRELESS.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "SEEKFREE_7725.h"
#include "SEEKFREE_RDA5807.h"
#include "SEEKFREE_7725_UART.h"
//------����Ӻ���
void Initialization(void);                            //��ʼ������
void Transport_Protocolse(float Send_variable);       //����Э��
float Bluetooth_Variable_Control(float Variable,int operation,float quantity);//������������
void Float_to_Byte(float f,unsigned char byte[]);     //��������fת��Ϊ4���ֽ����ݴ����byte[4]��
void Bluetooth_Trans_Control(void);                   //�������估����
void OLED(void);                                             //OLED��ʾ
int OLED_Function_One(int ins_change,int change_count);      //�Ӽ�����  ������Ҫ�仯�ı������仯ֵ(����)(OLED��ʾ��������1)
float OLED_Function_Two(float ins_change,float change_count);//�Ӽ�����(������)(OLED��ʾ��������2)
void OLED_Function_Three(void);                              //ָ�������ƶ�(OLED��ʾ��������3)
void OLED_Function_Four(void);                               //���ҷ�ҳ(OLED��ʾ��������4)
void OLED_Function_Five(void);                               //���ҷ�ҳ(OLED��ʾ��������5)
void OLED_Image(void);                                 //OLED����ͼ�����ʾ
void OLED_Image_Function_one(int line_function);       //OLEDͼ����ʾ��������1
void OLED_Image_Function_two(int line_function);       //OLEDͼ����ʾ��������2
int  OLED_Image_Function_three(int import);            //OLEDͼ����ʾ��������2(��������ʾ��oled�ϵĶ�Ӧ��)
float Slope_Calculate(uint8 begin,uint8 end,float *p);//��С���˷�
void Signal_Init(void);                               //��г�ʼ���������м䰴����ʼ�����
void Inductance_GetDeal(void);                        //����źŲɼ�������
void Servocontrol(void);                              //�������
void Motor_Control(void);                             //�������
void width_text(int line_function);                   //��Ȳ鿴
void Picture_Processing(void);                        //ͼ����
void LineHop_Average(int line_function);              //�洢�лҶȾ�ֵ
int Image_Ternary_Segmentation(int line_function);    //����ѡ�зֳ������������֣��Ա������ֵļӺ�ֵ��С
void Angle_Detection(void);                           //���ת�ǣ�������������
void Buzzing_Remind(void);                            //��������ʾ
void Zebra_Crossing_Search_One(int line_function);    //����������
void icm20602_clean(void);
void Line_Jump_Search(void);                          //�б߽�������
void Border_Slope_Calculate(void);                    //���������ұ߽�б��
void Annulus_Judge(void);                             //�ж�Բ��
//------����Ӷ���
//�������򰴼�����
#define KeyRight    P33_8
#define KeyUp       P33_6
#define KeyTip      P33_5
#define KeyDown     P33_7
#define KeyLeft     P33_4
//����������
#define S_MOTOR_PIN   ATOM1_CH1_P33_9
//����������
#define Diangan0   ADC0_CH1_A1
#define Diangan1   ADC0_CH2_A2
#define Diangan2   ADC0_CH0_A0
#define Diangan3   ADC0_CH3_A3

//------����ӱ���
extern uint8 bluetooth;
/////////////////////////////////////ͼ��ɼ�//////////////////////////////////////////
extern int practical_maxline ;            //����ͼ����Ϣ�����������
extern int practical_image[120][188];   //ͼ��ɼ��󽫲ɼ��������Ԥ�棨ת�棩
extern uint8 mt9v03x_image_out[188];
extern int mt9v03x_image_Resolution[188];
extern int leftline [120];  //ʵʱ��߽�
extern int rightline[120];  //ʵʱ�ұ߽�
extern int width_out[120];  //����ж�Ӧʵʱ�������
extern int width_stand_out[120];  //���б�׼��� ��Ϻ���+����ƫ��
extern int midline  [120];  //����ж�Ӧʵʱ����
extern int result_midline[120]; //�����������pid�е�����ֵ
extern int midline_stand;        //��׼����
extern int midline_error[120];  //����ж�Ӧʵʱƫ��***
extern int image_average[120];  //����е����ص��ֵ
extern int image_average_bottom ; //ǰ���Ƿ�Ϊȫ�ڵ���ֵ
extern int turn_flag;             //ת��������־λ����ת������ת
extern int line_white_dot_sum[120]; //һ���а׵����
extern int line_black_dot_sum[120]; //һ���кڵ����


extern int left_limit ;
extern int right_limit ;
extern int line_search_start[120];
extern int left_flag[120];
extern int right_flag[120];
extern int left_hop[120];
extern int right_hop[120];

extern int hop;             //����Ҷ�ֵ                                              //*
extern int hop_bottom ;
extern int front_text0;     //��Ч��0 (�����б�ǰ����Ϣ)                              //*//
extern int front_text1;     //��Ч��1                                                 //*//
extern int front_text2;     //��Ч��2                                                 //*
extern int front_text3;     //��Ч��3                                                 //*//
extern int front_text4;     //��Ч��4                                                 //*//
//���ұ߽��ȡ
extern float LeftlineArray[10];
extern float RightlineArray[10];
extern float LeftlineDerivativeLast ; //��߽���һ��б��
extern float RightlineDerivativeLast ;//�ұ߽���һ��б��
extern float LeftlineDerivative ;     //��߽�б��
extern float RightlineDerivative ;    //�ұ߽�б��

extern float LeftlineDerivative_limit  ;
extern float RightlineDerivative_limit ;
extern float LeftlineDerivative_wach;
extern float RightlineDerivative_wach;
//ͼ����Ϣ���
extern int three_row_white_flag; //���а�ɫ�������ȫ�ף�

extern int image_average_limit_bottom;  //�����ؾ�ֵ���ޣ�С����ȫ�ڣ�
//�߽�б�ʽ���
extern int  LeftLineSlope[120];    //��¼������߽�б��
extern int  RightLineSlope[120];   //��¼�����ұ߽�б��
extern int LeftSlopeLimit[4]; //��߽�б�ʷ���߽�    �ɵ�
extern int RrightSlopeLimit[4]; //�ұ߽�б�ʷ���߽�    �ɵ�
extern int Line_Demarcation[3];      //�зֽ���
extern int LeftSlopeSortCount[5][5];      //��߽�б�ʷ���ͳ�ƣ�[��������][��������]ƫ˳��ͳ�ƣ�    ����
extern int RrightSlopeSortCount[5][5];    //��߽�б�ʷ���ͳ�ƣ�[��������][��������]ƫ˳��ͳ�ƣ�    ����
/////////////////////////////////////���//////////////////////////////////////////////
extern int16 encoder_speed; //������
//���������
extern int duty_front ; //��ת
extern int duty_behind ;   //��ת
//�ٶ�ֵ
extern float SpeedSet;    //�ٶ�����
extern float SpeedSet_Nnomal;
extern float SpeedSet_ramp[6];
extern float SpeedSet_Annulu[4];     //Բ���ٶ�
extern int   OutgarageSpeed; //�����ٶ�
extern int   ZebraSpeed ;    //����ٶ�
extern float MotorNowSpeed ; //ʵʱ�ٶ�
extern float MotorExpSpeed ; //�����ٶ�
//�ٶȱջ�
extern float MotorErrArray[10] ;//�ٶ�ƫ��洢
extern float MotorErrDerivativeLast ;//��һ���ٶ�ƫ��仯��
extern float MotorErrDerivative ;//�ٶ�ƫ��仯��

extern float MotorK  ; //�ٶȱջ�kp�����Ƹ���仯ʱ�Ĺ���ƫ�
extern float MotorD  ; //�ٶȱջ�kd�����ƾ�̬������
extern float MotorI  ; //�ٶȱջ�ki��Ӳ�ȣ�
//�ٶ����
extern float MotorOutLast ;
extern float MotorOut ;
/////////////////////////////////////���//////////////////////////////////////////////
//�����ƫ��
extern int SERVOLEFT   ; //(+145)
extern int SERVOMIDDLE ;
extern int SERVORIGHT  ; //(-145)
//FuzzyPID
extern float ServoFuzzyErr,ServoFuzzyErrStore[10],ServoFuzzyErrDerivative;
extern float ServoFuzzyOutput;
extern float ServoKp;
extern float ServoKd;
extern float ServoKp_induc[2];                                       //*
extern float ServoKd_induc[2];                                       //*

extern int PID_Flag;
extern int PID_Option; //pidѡ��0Ϊ�ֶ�ʽpid��1Ϊ��������ܶ�pid��2Ϊ����µ�pid��3Ϊ����pid��Ĭ��Ϊ0
extern float ServoKp_image1;                                      //*
extern float ServoKd_image1;                                      //*
extern float ServoKp_image2;                                      //*
extern float ServoKd_image2;                                      //*
extern float ServoKp_image3;                                      //*
extern float ServoKd_image3;                                      //*
extern float ServoKp_all;                                      //
extern float ServoKd_all;                                      //

extern int   Sub_PID[4];

////�ֶ�PID
//float ServoKpSmall,ServoKdSmall;
//float ServoKpMid,ServoKdMid;
//float ServoKpBig,ServoKdBig;
//float ServoKpArray[3] ;
//float ServoKpRamp ;
//float ServoKpErrin ;
////����������������
//int   OutTrack;                     //���������ҵ��
//int   ThrowLine;                   //�ж��Ѷ���//2000
//int   ThrowLineCompensate;           //���߲���
//float DeviationPiecewise[2];   //���ƫ��ֶ�
//������
extern int   ServoDuty;//���ʵ����
extern int   Duty;   //���������
/////////////////////////////////////�����Ϣ//////////////////////////////////////////
extern uint16 InductanceGetlook[4];                         //��ʼ���ֵ
extern uint16 InductanceGetlookMax[4];                      //������ֵ
extern uint16 InductanceGet[4];                             //��һ�����
extern float InductanceSAVE[4];                              //����ʵ������ʱ�洢��вɼ�ֵ
extern float InductanceDeal[1] ;                           //��Ȼ����(ƫ��)
extern float InductanceUse[1] ;                            //ƫ���������
extern float InductanceUseRecult ;                           //�����������ߵ�����
extern float InductanceUseLast;                               //��һ���������ߵ�����
extern float InductanceUseWi;                                //һ���ͺ��˲�ϵ��(ϵ��*���+(1-ϵ��)*�ϴ�)
extern int   OutTrack;                                       //���������ҵ��
/////////////////////////////////////��С���˷�////////////////////////////////////////
//��б����ֵ����
extern int LSDepth;
//��˵�б仯��
extern float InductanceGetStoreLeft[10];
extern float InductanceLeftDerivative;
extern float InductanceLeftDerivative_last;
extern float InductanceLeftDerivative_k;
//�Ҷ˵�б仯��
extern float InductanceGetStoreRight[10];
extern float InductanceRightDerivative;
extern float InductanceRightDerivative_last;
extern float InductanceRightDerivative_k;
//���˵�б仯��(��)
extern float InductanceGetStoreNear[10];
extern float InductanceNearDerivative;
extern float InductanceNearDerivative_last;
extern float InductanceNearDerivative_k;
//Զ�˵�б仯�ʣ�����
extern float InductanceGetStoreFar[10];
extern float InductanceFarDerivative;
extern float InductanceFarDerivative_last;
extern float InductanceFarDerivative_k;
/////////////////////////////////////����////////////////////////////////////////////
/*�̶�����*/
extern int OutgarageFlag ;       //�����Ƿ���⣬0δ����1�ѳ�
/*���ڲ���*/
extern int OutgarageDirection;       //���ⷽ��0Ϊ����⣬1Ϊ�ҳ��⣬Ĭ��Ϊ����⣨0��
extern int OutgarageServoDuty[2]  ;  //����ʱ����趨ֵ
extern float OutgarageAngle;         //����Ƕ�
extern  float OutgarageAngleLimit;   //����Ƕ�����
//////////////////////////////////ʶ�������/////////////////////////////////////////
extern int ZebraCrossingLine1;
extern int ZebraWtoB_count[120]; //���аױ�ڸ���
extern int ZebraBtoW_count[120]; //���кڱ�׸���
extern int ZebraWtoB_column[120][15];//�ױ��������
extern int ZebraBtoW_column[120][15];//�ڱ��������
extern int ZebraCrossingFlag;    //�����߱��
extern int ZebraErrorSum;        //������ǰ����ƫ���
extern int ZebraErrorCount;      //������ǰ��Ч��
extern int ZebraError;           //������ǰ����ƫ��
extern float ZebraAngle;         //���ǽǶ�
extern int ZebraAngleLimit;      //�������Ƕ�
extern int ZebraDistance_Real;   //�����ʻ����
extern int ZebraDistance_Limit;  //����һ�׶���ʻ��������
extern int EndLeftturn;
extern int EndRightturn;
/////////////////////////////////////Բ��////////////////////////////////////////
extern int Annulu_Direction;    //Բ�������ǣ�0Ϊ������1Ϊ��Բ����2Ϊ��Բ��
extern int Annulu_Flag;         //Բ����ǣ�0Ϊ������
extern int Annulu_Flag_supplement;   //Բ����ǻ����е�С����
extern int SlopePASSMargin[2][2];       //�߽�б�������������ƣ����ڴ˲�����Ч��
extern int Annulu_Radius;       //Բ���뾶ѡ��0Ϊ50��1Ϊ60��2Ϊ70��3Ϊ80
extern float AnnuluAngle;
extern float AnnuluAngleLimit_Left[3];   //Բ�������ǽǶ����ƣ��󣩣��ж��ڻ��У������������ѳ�����
extern float AnnuluAngleLimit_Right[3];  //Բ�������ǽǶ����ƣ��ң����ж��ڻ��У������������ѳ�����
//׼���뻷
extern int Annulu_one_count;     //׼���뻷����һ���б߽�������
extern int Annulu_one_error_sum; //׼���뻷����Ч�е�ƫ���
extern int Annulu_one_error;     //׼���뻷����Ч�е�ƫ��
//��ʼ�뻷
//extern int Annulu_Two_count;     //��ʼ�뻷����һ���б߽�������
//extern int Annulu_Two_error_sum; //��ʼ�뻷����Ч�е�ƫ���
//extern int Annulu_Two_error;     //��ʼ�뻷����Ч�е�ƫ��
//extern int Annulu_Two_error_last;//��ʼ�뻷����Ч�е���һ��ƫ��
extern int Annulu_Two_LeftTurn[4];/////////****************��Ҫ�ɵ���ͬʱ��Ҫ���ݲ�ͬ�뾶ȡ��ֵͬ
extern int Annulu_Two_RightTurn[4];
extern float ServoKp_Annulu[4];//��ͬ�뾶����Ӧ��ͬ���KP
extern float ServoKd_Annulu[4];//��ͬ�뾶����Ӧ��ͬ���KD
/////////////////////////////////////ICM20602//////////////////////////////////////////////
extern float angle_re_last[2];
extern float icm_gyro_y_last[3];
extern float icm_gyro_z_last[3];
extern float icm_gyro_x_clean;
extern float icm_gyro_y_clean;
extern float icm_gyro_z_clean;
extern float w_y_re_use;                         //���õ�y����ٶ�
extern float w_z_re_use;                         //���õ�z����ٶ�
extern float angle_acc;
extern float angle_gyro;
extern float angle_re;
extern float angle_re_use;
extern float acc_unit_conversion ;           //���ٶȵ�λת�����ȣ�
extern float gyro_unit_conversion;           //���ٶȵ�λת�����ȣ�
extern float gyro_integral_time  ;          //���ٶȻ���ʱ��
extern float angle_k1;                     //���ٶ�����ٶ��ںϣ����ٶ�ռ��Ȩ��
/////////////////////////////////////�µ�ʶ��//////////////////////////////////////////
extern int ramp_flag;                   //�µ���־
extern int ramp_flag_real_distance;     //�µ���ʻʵʱ����
extern int ramp_flag_limit_distance;    //�µ���ʻ���ƾ���
extern int ramp_discern_top;            //�µ�ʶ���ϱ߽�
extern int ramp_discern_bottom;         //�µ�ʶ���±߽�

/////////////////////////////////////FLASH/////////////////////////////////////////////

/////////////////////////////////////OLED//////////////////////////////////////////////
extern int count_oled_time ;                   //��������
extern int count_oled;                         //��������
extern int optionOne;                          //��һ����ʾ
extern int optionTwo;                          //�ڶ�����ʾ
extern int optionThree;                        //��������ʾ
/////////////////////////////////////����//////////////////////////////////////////////
extern int   count_UART2_time;                 //��������
extern int   count_UART2;                      //��������
extern uint8 bluetooth ;                       //���ݽ��ܱ���
extern unsigned char byte[4];                  //�ֽ����
/////////////////////////////////////��������ʾ////////////////////////////////////////
extern int Buzzing_count;        //��������ʱ��
extern int Buzzing_count_limit;  //��������ʱ����ʱʱ���趨
extern int Buzzing_start_flag;   //������������־
/////////////////////////////////////��������//////////////////////////////////////////
extern int runtime;                            //��ʱ����������ʱ��
extern int runtimemax;
extern int8 ch1[100];
/////////////////////////////////////ͣ��//////////////////////////////////////////
extern long int car_stop_real_distance;        //������ʻʵ�ʾ���
extern long int car_stop_limit_distance;       //������ʻ���ƾ���
extern int car_stop_reset;                     //��ʻ��������
////////////////////////////////////////TFmini/////////////////////////////////////////
extern uint8  TFminidate[9];                   //TFminidate[4]Ϊ����
extern uint8  TFminidate_true[9];
extern uint8  data_TFmini;
extern int    TFminiflag;
extern int    count_UART0;
extern int    distance;
extern int    tfmini_return;

extern int left_add;
extern int middle_add;
extern int right_add;
extern int comparative_result0;

#endif

