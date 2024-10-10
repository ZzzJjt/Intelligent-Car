/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		headfile
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
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
//官方头文件
#include "ifxAsclin_reg.h"
#include "SysSe/Bsp/Bsp.h"
#include "IfxCcu6_Timer.h"
#include "IfxScuEru.h"
//------逐飞科技单片机外设驱动头文件
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
//------逐飞科技产品驱动头文件
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
//------自添加函数
void Initialization(void);                            //初始化函数
void Transport_Protocolse(float Send_variable);       //传输协议
float Bluetooth_Variable_Control(float Variable,int operation,float quantity);//蓝牙变量控制
void Float_to_Byte(float f,unsigned char byte[]);     //将浮点数f转化为4个字节数据存放在byte[4]中
void Bluetooth_Trans_Control(void);                   //蓝牙传输及控制
void OLED(void);                                             //OLED显示
int OLED_Function_One(int ins_change,int change_count);      //加减变量  输入所要变化的变量及变化值(整型)(OLED显示辅助函数1)
float OLED_Function_Two(float ins_change,float change_count);//加减变量(浮点型)(OLED显示辅助函数2)
void OLED_Function_Three(void);                              //指针上下移动(OLED显示辅助函数3)
void OLED_Function_Four(void);                               //左右翻页(OLED显示辅助函数4)
void OLED_Function_Five(void);                               //左右翻页(OLED显示辅助函数5)
void OLED_Image(void);                                 //OLED关于图像的显示
void OLED_Image_Function_one(int line_function);       //OLED图像显示辅助函数1
void OLED_Image_Function_two(int line_function);       //OLED图像显示辅助函数2
int  OLED_Image_Function_three(int import);            //OLED图像显示辅助函数2(求余数显示在oled上的对应点)
float Slope_Calculate(uint8 begin,uint8 end,float *p);//最小二乘法
void Signal_Init(void);                               //电感初始化，按下中间按键初始化完成
void Inductance_GetDeal(void);                        //电感信号采集、处理
void Servocontrol(void);                              //舵机控制
void Motor_Control(void);                             //电机控制
void width_text(int line_function);                   //宽度查看
void Picture_Processing(void);                        //图像处理
void LineHop_Average(int line_function);              //存储行灰度均值
int Image_Ternary_Segmentation(int line_function);    //对所选列分成左中右三部分，对比三部分的加和值大小
void Angle_Detection(void);                           //检测转角（俯仰，翻滚）
void Buzzing_Remind(void);                            //蜂鸣器警示
void Zebra_Crossing_Search_One(int line_function);    //搜索斑马线
void icm20602_clean(void);
void Line_Jump_Search(void);                          //行边界跳变检测
void Border_Slope_Calculate(void);                    //求解各行左右边界斜率
void Annulus_Judge(void);                             //判断圆环
//------自添加定量
//定义五向按键引脚
#define KeyRight    P33_8
#define KeyUp       P33_6
#define KeyTip      P33_5
#define KeyDown     P33_7
#define KeyLeft     P33_4
//定义舵机引脚
#define S_MOTOR_PIN   ATOM1_CH1_P33_9
//定义电感引脚
#define Diangan0   ADC0_CH1_A1
#define Diangan1   ADC0_CH2_A2
#define Diangan2   ADC0_CH0_A0
#define Diangan3   ADC0_CH3_A3

//------自添加变量
extern uint8 bluetooth;
/////////////////////////////////////图像采集//////////////////////////////////////////
extern int practical_maxline ;            //进行图像信息搜索的最大行
extern int practical_image[120][188];   //图像采集后将采集结果进行预存（转存）
extern uint8 mt9v03x_image_out[188];
extern int mt9v03x_image_Resolution[188];
extern int leftline [120];  //实时左边界
extern int rightline[120];  //实时右边界
extern int width_out[120];  //检测行对应实时赛道宽度
extern int width_stand_out[120];  //各行标准宽度 拟合函数+补偿偏移
extern int midline  [120];  //检测行对应实时中线
extern int result_midline[120]; //处理后运用在pid中的中线值
extern int midline_stand;        //标准中线
extern int midline_error[120];  //检测行对应实时偏差***
extern int image_average[120];  //检测行的像素点均值
extern int image_average_bottom ; //前方是否为全黑的阈值
extern int turn_flag;             //转弯打死舵标志位及左转还是右转
extern int line_white_dot_sum[120]; //一行中白点个数
extern int line_black_dot_sum[120]; //一行中黑点个数


extern int left_limit ;
extern int right_limit ;
extern int line_search_start[120];
extern int left_flag[120];
extern int right_flag[120];
extern int left_hop[120];
extern int right_hop[120];

extern int hop;             //跳变灰度值                                              //*
extern int hop_bottom ;
extern int front_text0;     //有效行0 (用于判别前方信息)                              //*//
extern int front_text1;     //有效行1                                                 //*//
extern int front_text2;     //有效行2                                                 //*
extern int front_text3;     //有效行3                                                 //*//
extern int front_text4;     //有效行4                                                 //*//
//左右边界读取
extern float LeftlineArray[10];
extern float RightlineArray[10];
extern float LeftlineDerivativeLast ; //左边界上一次斜率
extern float RightlineDerivativeLast ;//右边界上一次斜率
extern float LeftlineDerivative ;     //左边界斜率
extern float RightlineDerivative ;    //右边界斜率

extern float LeftlineDerivative_limit  ;
extern float RightlineDerivative_limit ;
extern float LeftlineDerivative_wach;
extern float RightlineDerivative_wach;
//图像信息检测
extern int three_row_white_flag; //三行白色（检测行全白）

extern int image_average_limit_bottom;  //行像素均值下限（小于则全黑）
//边界斜率解算
extern int  LeftLineSlope[120];    //记录各行左边界斜率
extern int  RightLineSlope[120];   //记录各行右边界斜率
extern int LeftSlopeLimit[4]; //左边界斜率分类边界    可调
extern int RrightSlopeLimit[4]; //右边界斜率分类边界    可调
extern int Line_Demarcation[3];      //行分界线
extern int LeftSlopeSortCount[5][5];      //左边界斜率分类统计（[由上至下][由内至外]偏顺序统计）    可视
extern int RrightSlopeSortCount[5][5];    //左边界斜率分类统计（[由上至下][由内至外]偏顺序统计）    可视
/////////////////////////////////////电机//////////////////////////////////////////////
extern int16 encoder_speed; //编码器
//电机测试用
extern int duty_front ; //正转
extern int duty_behind ;   //反转
//速度值
extern float SpeedSet;    //速度设置
extern float SpeedSet_Nnomal;
extern float SpeedSet_ramp[6];
extern float SpeedSet_Annulu[4];     //圆环速度
extern int   OutgarageSpeed; //出库速度
extern int   ZebraSpeed ;    //入库速度
extern float MotorNowSpeed ; //实时速度
extern float MotorExpSpeed ; //期望速度
//速度闭环
extern float MotorErrArray[10] ;//速度偏差存储
extern float MotorErrDerivativeLast ;//上一次速度偏差变化率
extern float MotorErrDerivative ;//速度偏差变化率

extern float MotorK  ; //速度闭环kp（抑制跟随变化时的过度偏差）
extern float MotorD  ; //速度闭环kd（抑制静态抖动）
extern float MotorI  ; //速度闭环ki（硬度）
//速度输出
extern float MotorOutLast ;
extern float MotorOut ;
/////////////////////////////////////舵机//////////////////////////////////////////////
//舵机的偏角
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
extern int PID_Option; //pid选择，0为分段式pid，1为电磁正常跑动pid，2为电磁坡道pid，3为出库pid，默认为0
extern float ServoKp_image1;                                      //*
extern float ServoKd_image1;                                      //*
extern float ServoKp_image2;                                      //*
extern float ServoKd_image2;                                      //*
extern float ServoKp_image3;                                      //*
extern float ServoKd_image3;                                      //*
extern float ServoKp_all;                                      //
extern float ServoKd_all;                                      //

extern int   Sub_PID[4];

////分段PID
//float ServoKpSmall,ServoKdSmall;
//float ServoKpMid,ServoKdMid;
//float ServoKpBig,ServoKdBig;
//float ServoKpArray[3] ;
//float ServoKpRamp ;
//float ServoKpErrin ;
////其他变量参数设置
//int   OutTrack;                     //出赛道左右电感
//int   ThrowLine;                   //判断已丢线//2000
//int   ThrowLineCompensate;           //丢线补偿
//float DeviationPiecewise[2];   //舵机偏差分段
//舵机输出
extern int   ServoDuty;//舵机实际用
extern int   Duty;   //舵机测试用
/////////////////////////////////////电感信息//////////////////////////////////////////
extern uint16 InductanceGetlook[4];                         //初始电感值
extern uint16 InductanceGetlookMax[4];                      //电感最大值
extern uint16 InductanceGet[4];                             //归一化结果
extern float InductanceSAVE[4];                              //用于实际运行时存储电感采集值
extern float InductanceDeal[1] ;                           //差比积结果(偏差)
extern float InductanceUse[1] ;                            //偏差处理后的期望
extern float InductanceUseRecult ;                           //最终用于搜线的期望
extern float InductanceUseLast;                               //上一次用于搜线的期望
extern float InductanceUseWi;                                //一阶滞后滤波系数(系数*这次+(1-系数)*上次)
extern int   OutTrack;                                       //出赛道左右电感
/////////////////////////////////////最小二乘法////////////////////////////////////////
//求斜率数值个数
extern int LSDepth;
//左端电感变化率
extern float InductanceGetStoreLeft[10];
extern float InductanceLeftDerivative;
extern float InductanceLeftDerivative_last;
extern float InductanceLeftDerivative_k;
//右端电感变化率
extern float InductanceGetStoreRight[10];
extern float InductanceRightDerivative;
extern float InductanceRightDerivative_last;
extern float InductanceRightDerivative_k;
//近端电感变化率(横)
extern float InductanceGetStoreNear[10];
extern float InductanceNearDerivative;
extern float InductanceNearDerivative_last;
extern float InductanceNearDerivative_k;
//远端电感变化率（竖）
extern float InductanceGetStoreFar[10];
extern float InductanceFarDerivative;
extern float InductanceFarDerivative_last;
extern float InductanceFarDerivative_k;
/////////////////////////////////////出库////////////////////////////////////////////
/*固定参数*/
extern int OutgarageFlag ;       //车辆是否出库，0未出；1已出
/*调节参数*/
extern int OutgarageDirection;       //车库方向0为左出库，1为右出库，默认为左出库（0）
extern int OutgarageServoDuty[2]  ;  //出库时舵机设定值
extern float OutgarageAngle;         //出库角度
extern  float OutgarageAngleLimit;   //出库角度限制
//////////////////////////////////识别斑马线/////////////////////////////////////////
extern int ZebraCrossingLine1;
extern int ZebraWtoB_count[120]; //各行白变黑个数
extern int ZebraBtoW_count[120]; //各行黑变白个数
extern int ZebraWtoB_column[120][15];//白变黑跳变列
extern int ZebraBtoW_column[120][15];//黑变白跳变列
extern int ZebraCrossingFlag;    //斑马线标记
extern int ZebraErrorSum;        //斑马线前车辆偏差和
extern int ZebraErrorCount;      //斑马线前有效行
extern int ZebraError;           //斑马线前车辆偏差
extern float ZebraAngle;         //入库记角度
extern int ZebraAngleLimit;      //入库结束角度
extern int ZebraDistance_Real;   //入库行驶距离
extern int ZebraDistance_Limit;  //入库第一阶段行驶距离限制
extern int EndLeftturn;
extern int EndRightturn;
/////////////////////////////////////圆环////////////////////////////////////////
extern int Annulu_Direction;    //圆环方向标记，0为正常，1为左圆环，2为右圆环
extern int Annulu_Flag;         //圆环标记，0为正常，
extern int Annulu_Flag_supplement;   //圆环标记环节中的小环节
extern int SlopePASSMargin[2][2];       //边界斜率提升行数限制（大于此才算有效）
extern int Annulu_Radius;       //圆环半径选择，0为50，1为60，2为70，3为80
extern float AnnuluAngle;
extern float AnnuluAngleLimit_Left[3];   //圆环陀螺仪角度限制（左）（判断在环中，即将出环，已出环）
extern float AnnuluAngleLimit_Right[3];  //圆环陀螺仪角度限制（右）（判断在环中，即将出环，已出环）
//准备入环
extern int Annulu_one_count;     //准备入环中另一边有边界行数量
extern int Annulu_one_error_sum; //准备入环中有效行的偏差和
extern int Annulu_one_error;     //准备入环中有效行的偏差
//开始入环
//extern int Annulu_Two_count;     //开始入环中另一边有边界行数量
//extern int Annulu_Two_error_sum; //开始入环中有效行的偏差和
//extern int Annulu_Two_error;     //开始入环中有效行的偏差
//extern int Annulu_Two_error_last;//开始入环中有效行的上一次偏差
extern int Annulu_Two_LeftTurn[4];/////////****************需要可调，同时需要根据不同半径取不同值
extern int Annulu_Two_RightTurn[4];
extern float ServoKp_Annulu[4];//不同半径环对应不同舵机KP
extern float ServoKd_Annulu[4];//不同半径环对应不同舵机KD
/////////////////////////////////////ICM20602//////////////////////////////////////////////
extern float angle_re_last[2];
extern float icm_gyro_y_last[3];
extern float icm_gyro_z_last[3];
extern float icm_gyro_x_clean;
extern float icm_gyro_y_clean;
extern float icm_gyro_z_clean;
extern float w_y_re_use;                         //利用的y轴角速度
extern float w_z_re_use;                         //利用的z轴角速度
extern float angle_acc;
extern float angle_gyro;
extern float angle_re;
extern float angle_re_use;
extern float acc_unit_conversion ;           //加速度单位转换（度）
extern float gyro_unit_conversion;           //角速度单位转换（度）
extern float gyro_integral_time  ;          //角速度积分时间
extern float angle_k1;                     //加速度与角速度融合，加速度占比权重
/////////////////////////////////////坡道识别//////////////////////////////////////////
extern int ramp_flag;                   //坡道标志
extern int ramp_flag_real_distance;     //坡道行驶实时距离
extern int ramp_flag_limit_distance;    //坡道行驶限制距离
extern int ramp_discern_top;            //坡道识别上边界
extern int ramp_discern_bottom;         //坡道识别下边界

/////////////////////////////////////FLASH/////////////////////////////////////////////

/////////////////////////////////////OLED//////////////////////////////////////////////
extern int count_oled_time ;                   //消抖总数
extern int count_oled;                         //消抖计数
extern int optionOne;                          //第一层显示
extern int optionTwo;                          //第二层显示
extern int optionThree;                        //第三层显示
/////////////////////////////////////蓝牙//////////////////////////////////////////////
extern int   count_UART2_time;                 //消抖总数
extern int   count_UART2;                      //消抖计数
extern uint8 bluetooth ;                       //数据接受变量
extern unsigned char byte[4];                  //分解变量
/////////////////////////////////////蜂鸣器警示////////////////////////////////////////
extern int Buzzing_count;        //蜂鸣器计时器
extern int Buzzing_count_limit;  //蜂鸣器计时器延时时间设定
extern int Buzzing_start_flag;   //开启蜂鸣器标志
/////////////////////////////////////其他变量//////////////////////////////////////////
extern int runtime;                            //计时部分总运行时间
extern int runtimemax;
extern int8 ch1[100];
/////////////////////////////////////停车//////////////////////////////////////////
extern long int car_stop_real_distance;        //正常行驶实际距离
extern long int car_stop_limit_distance;       //正常行驶限制距离
extern int car_stop_reset;                     //行驶距离清零
////////////////////////////////////////TFmini/////////////////////////////////////////
extern uint8  TFminidate[9];                   //TFminidate[4]为距离
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

