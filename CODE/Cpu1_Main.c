/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC364DP
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-11-23
 ********************************************************************************************************************/

#include "headfile.h"
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#pragma section all "cpu1_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中
/////////////////////////////////////mpu6050//////////////////////////////////////////////
int angle_start_count=0;           //角度检测计时器
int angle_start_count_limit=200;   //大于200*5ms后角度检测有校
double  acc_return_x=0.0;          //加速度计值转型转存
double  acc_return_y=0.0;
double  acc_return_z=0.0;
double  acc_angle_x=0.0;           //加速度计角度
double  acc_angle_y=0.0;
double  acc_compensate_x=+4.5;     //加速度计角度消零（由于安装出现的偏差）
double  acc_compensate_y=-4.0;
double  acc_angle_x_min= 5.0;      //角度最大最小值记录
double  acc_angle_x_max=-5.0;
double  acc_angle_y_min= 5.0;
double  acc_angle_y_max=-5.0;
//陀螺仪相关变量（暂时不用）（未成为全局变量）
float g_accuracy=0.518;                    //转换常量
int   compensate_x=-123;                   //消除零飘
int   compensate_y=-105;
int   compensate_z=0;
float relly_gyro_x=0.0;                    //消零飘后的陀螺仪值
float relly_gyro_y=0.0;
float relly_gyro_z=0.0;
float g_angle_x=0.0;                       //陀螺仪角度
float g_angle_y=0.0;
float g_angle_z=0.0;

/***********************************************************************************
姿态解算
***********************************************************************************/
float angle_re_last[2]={0};
float icm_gyro_y_last[3]={0};
float icm_gyro_z_last[3]={0};
float icm_gyro_x_clean=0.22;
float icm_gyro_y_clean=0.88;
float icm_gyro_z_clean=1.38;
float w_y_re_use=0;                         //利用的y轴角速度
float w_z_re_use=0;                         //利用的z轴角速度
float angle_acc=0;
float angle_gyro=0;
float angle_re=0;
float angle_re_use=0;
float acc_unit_conversion = 57.3;           //加速度单位转换（度）
float gyro_unit_conversion=-0.50;           //角速度单位转换（度）
float gyro_integral_time  = 0.002;          //角速度积分时间
float angle_k1=0.007;                       //加速度与角速度融合，加速度占比权重
/////////////////////////////////////OLED//////////////////////////////////////////////
int count_oled_time =200;                   //消抖总数
int count_oled=0;                          //消抖计数
int optionOne=0;                           //第一层显示
int optionTwo=0;                           //第二层显示
int optionThree=0;                         //第三层显示
int8 ch1[100];
/////////////////////////////////////蓝牙//////////////////////////////////////////////
int   count_UART2_time=100;                //消抖总数
int   count_UART2=0;                       //消抖计数
uint8 bluetooth = 0;                       //数据接受变量
unsigned char byte[4]={0};                 //分解变量
unsigned char tail[4]={0x00,0x00,0x80,0x7f};//帧尾
uint8 test1[] = "seekfree wireless to uart test\n";
uint8 get_data = 0;                             // 无线串口获取数据的变量 在对应无线串口的中断服务函数使用 uart2_rx_isr
uint8 get_flag = 0;                             // 无线串口获取数据的标志 在对应无线串口的中断服务函数使用 uart2_rx_isr
/////////////////////////////////////定义结构体////////////////////////////////////////
typedef union
{
    float fdata;
    unsigned long ldata;
}FloatLongType;


void core1_main(void)
{
	disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    //用户在此处调用各种初始化函数等

    //摄像头图像传输串口初始化//激光雷达串口中断
    //uart_init(UART_0, 115200, UART0_TX_P14_0, UART0_RX_P14_1);

    //无线转串口模块相关引脚定义在 wireless.h文件中
    seekfree_wireless_init();
    //OLED初始化
    oled_init();
    //icm206020初始化
    icm20602_init_spi();
//    icm20602_clean();

    //使用CCU6_0模块的通道0 产生一个2ms的周期中断
    pit_interrupt_ms(CCU6_0, PIT_CH0, 2); //速度闭环
    //使用CCU6_1模块的通道0 产生一个5ms的周期中断
    pit_interrupt_ms(CCU6_1, PIT_CH0, 2); //mpu6050

	//等待所有核心初始化完毕
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    enableInterrupts();

//    systick_delay_ms(STM0, 1000);
//    seekfree_wireless_send_buff(test1,sizeof(test1)-1);//由于sizeof计算字符串的长度包含了最后一个0，因此需要减一

    while (TRUE)
    {
        //OLED显示
//        if(OutgarageFlag != 2 || ZebraCrossingFlag!=0)
          OLED();
//        else
//        {
//        }
        //蓝牙传输及控制
//        Bluetooth_Trans_Control();
    }
}
/***************************************************
icm清零漂
****************************************************/
void icm20602_clean()
{
    uint8 icm_i=0;
    for(icm_i=0;icm_i<100;icm_i++){
        get_icm20602_gyro_spi();
        icm_gyro_x_clean+=(icm_gyro_x/8.0);
        icm_gyro_y_clean+=(icm_gyro_y/8.0);
        icm_gyro_z_clean+=(icm_gyro_z/8.0);
        systick_delay_ms(STM0, 5);
    }
    icm_gyro_x_clean/=100;
    icm_gyro_y_clean/=100;
    icm_gyro_z_clean/=100;
}
/***************************************************************************************
函 数 名 :void Angle_Detection(void)
功     能  :检测转角（俯仰，翻滚）
传     入  :无
返 回  值 :无
整定参数 :无
***************************************************************************************/
void Angle_Detection()
{
    angle_re_last[1]=angle_re_last[0];
    angle_re_last[0]=angle_re;

    icm_gyro_y_last[2]=icm_gyro_y_last[1];
    icm_gyro_y_last[1]=icm_gyro_y_last[0];
    icm_gyro_y_last[0]=icm_gyro_y;

    icm_gyro_z_last[2]=icm_gyro_z_last[1];
    icm_gyro_z_last[1]=icm_gyro_z_last[0];
    icm_gyro_z_last[0]=icm_gyro_z;

    get_icm20602_gyro_spi();
    get_icm20602_accdata_spi();


    w_y_re_use=icm_gyro_y/8.0;
    w_z_re_use=icm_gyro_z/8.0;

//  icm_gyro_x=icm_gyro_x-icm_gyro_x_clean;
    w_y_re_use=w_y_re_use-icm_gyro_y_clean;  //消除零飘
    w_z_re_use=w_z_re_use-icm_gyro_z_clean;

    angle_acc=(atan2(icm_acc_x,icm_acc_z))*acc_unit_conversion; //加速度得角度 单位转换
    angle_acc=(angle_acc+angle_re_last[0]+angle_re_last[1])/3; //滤波
    angle_gyro=w_y_re_use*gyro_unit_conversion ;

    angle_re=angle_k1*angle_acc+(1-angle_k1)*(angle_re+angle_gyro*0.002);       //0.002为pit时间！！！！

    //斑马线记角度
    if(ZebraCrossingFlag==1 || ZebraCrossingFlag==2)ZebraAngle+=w_z_re_use*gyro_unit_conversion*0.002;
    else ZebraAngle=0;

    //出库记角度
    if(OutgarageFlag == 1)OutgarageAngle+=w_z_re_use*gyro_unit_conversion*0.002;
    else OutgarageAngle=0;
    //圆环记角度
    if(Annulu_Flag != 0 &&  Annulu_Flag != 1)AnnuluAngle+=w_z_re_use*gyro_unit_conversion*0.002;
    else AnnuluAngle=0;

//    if(angle_start_count<angle_start_count_limit)angle_start_count++;
//    if(angle_start_count>=angle_start_count_limit)
//    {
//    }
}
/***************************************************************************************
函 数 名 :Bluetooth_Trans_Control(void)
功     能  :蓝牙传输及控制
传     入  :无
返 回  值 :无
整定参数 :无
***************************************************************************************/
void Bluetooth_Trans_Control(void)
{
    //电机测试
//        Transport_Protocolse((float)duty_front);
    //摄像头信息读取
//        Transport_Protocolse((float)hop);
//        Transport_Protocolse((float)front_text2);
//        Transport_Protocolse((float)leftline[front_text2]);
//        Transport_Protocolse((float)rightline[front_text2]);
//        Transport_Protocolse((float)width_out[front_text2]);
    //电机闭环调试
//        Transport_Protocolse((float)encoder_speed);
//        Transport_Protocolse((float)SpeedSet);
//        Transport_Protocolse((float)MotorD);
//        Transport_Protocolse((float)MotorK);
//        Transport_Protocolse((float)MotorI);
    //舵机PID调节
//    Transport_Protocolse((float)ServoKp);
//    Transport_Protocolse((float)ServoKd);
//    Transport_Protocolse((float)ServoDuty);
    //左右边界变化率
    //Transport_Protocolse((float)SpeedSet);
    //Transport_Protocolse((float)RightlineDerivative);
//    //弯道检测
//    Transport_Protocolse((float)ServoFuzzyErr);
//    Transport_Protocolse((float)ServoFuzzyOutput);
     //角度检测
//    Transport_Protocolse((float)icm_gyro_x_clean);
//    Transport_Protocolse((float)icm_gyro_y_clean);
//    Transport_Protocolse((float)icm_gyro_z_clean);
//    Transport_Protocolse((float)w_y_re_use);
//    Transport_Protocolse((float)w_z_re_use);
//    Transport_Protocolse((float)angle_gyro);
//    Transport_Protocolse((float)angle_acc);
    Transport_Protocolse((float)angle_re);
    Transport_Protocolse((float)angle_re_use);


    //协议帧尾
    seekfree_wireless_send_buff(&tail[0],1);
    seekfree_wireless_send_buff(&tail[1],1);
    seekfree_wireless_send_buff(&tail[2],1);
    seekfree_wireless_send_buff(&tail[3],1);

//    sprintf(ch1,"get_data: %d    ",get_data);
//    oled_p6x8str(0,2,ch1);
//    sprintf(ch1,"MotorD: %.2f    ",MotorD);
//    oled_p6x8str(0,3,ch1);
//    sprintf(ch1,"MotorK: %.2f    ",MotorK);
//    oled_p6x8str(0,4,ch1);
//    sprintf(ch1,"MotorI: %.2f    ",MotorI);
//    oled_p6x8str(0,5,ch1);
    //dat蓝牙控制
    if(get_flag)                                    // 监测数据接收标志是否被置位
    {
        switch(get_data)
        {
            case 0:count_UART2=0;break;//消抖计数清零
            case 1:SpeedSet  =Bluetooth_Variable_Control((float)SpeedSet,1,1);break;//速度期望上升
            case 2:SpeedSet  =Bluetooth_Variable_Control((float)SpeedSet,2,1);break;//速度期望下降
            case 3:SpeedSet  =0;break;   //停止
            case 4:SpeedSet  =65;break; //开启
            case 5:hop       =(int)(Bluetooth_Variable_Control((float)hop,1,5));break;//阈值上升
            case 6:hop       =(int)(Bluetooth_Variable_Control((float)hop,2,5));break;//阈值下降
            case 7 :MotorD=Bluetooth_Variable_Control((float)MotorD,1,0.1);break;//速度闭环D上升
            case 8 :MotorD=Bluetooth_Variable_Control((float)MotorD,2,0.1);break;//速度闭环D下降
            case 9 :MotorK=Bluetooth_Variable_Control((float)MotorK,1,1);break;  //速度闭环K上升
            case 10:MotorK=Bluetooth_Variable_Control((float)MotorK,2,1);break;  //速度闭环K下降
            case 11:MotorI=Bluetooth_Variable_Control((float)MotorI,1,0.1);break;//速度闭环I上升
            case 12:MotorI=Bluetooth_Variable_Control((float)MotorI,2,0.1);break;//速度闭环I下降
            default:{}
        }
        get_flag = 0;                               // 复位数据接收标志
    }
}
/***************************************************************************************
函 数 名 :void OLED_Image(void)int line_function
功    能 :OLED关于图像的显示
传    入 :无
返 回 值 :无
整定参数 :无
***************************************************************************************/
void OLED_Image()
{
    //x对应列，y对应行
    for(int j=0;j<188;j+=2)
    {
        oled_putpixel((uint8)(j/2),(uint8)(practical_image[ZebraCrossingLine1][j]>>2>>3),OLED_Image_Function_three(practical_image[ZebraCrossingLine1][j]>>2));
        oled_putpixel((uint8)(j/2),(uint8)(hop>>2>>3),OLED_Image_Function_three(hop>>2));
    }
    sprintf(ch1,"hop:      %d   *",hop);
    oled_p6x8str(0,7,ch1);
    hop=OLED_Function_One(hop,5);                    //**//
    oled_fill(0x00);
    //左右翻页(right和left翻页)
    OLED_Function_Five();
}
void OLED_Image_Function_one(int line_function)
{
    oled_putpixel((uint8)(leftline [line_function]/2),(uint8)(line_function>>3),OLED_Image_Function_three(line_function));
    //oled_putpixel((uint8)(midline  [line_function]/2),(uint8)(line_function)>>3,OLED_Image_Function_three(line_function));
    oled_putpixel((uint8)(rightline[line_function]/2),(uint8)(line_function>>3),OLED_Image_Function_three(line_function));
}
void OLED_Image_Function_two(int line_function)
{
    sprintf(ch1," %d     %d     %d      ",leftline [line_function],midline [line_function],rightline [line_function]);
    oled_p6x8str(0,line_function%6,ch1);
}
int OLED_Image_Function_three(int import)
{
    int output;
    int remainder; //求余数
    remainder=import-(import>>3<<3);
    if(remainder==0)output=1;
    else output=2<<remainder-1;
    return output ;
}
/***************************************************************************************
函 数 名 :void OLED()
功    能 :OLED显示
传    入 :无
返 回 值 :无
整定参数 :无
***************************************************************************************/
void OLED()
{
    if(optionOne==4)//显示搜线行灰度值情况
    {
        OLED_Image();
    }
    if(optionOne==1)//显示轨迹图（前50行）
    {
        OLED_Image_Function_one(0);OLED_Image_Function_one(1);OLED_Image_Function_one(2);OLED_Image_Function_one(3);OLED_Image_Function_one(4);
        OLED_Image_Function_one(5);OLED_Image_Function_one(6);OLED_Image_Function_one(7);OLED_Image_Function_one(8);OLED_Image_Function_one(9);
        OLED_Image_Function_one(10);OLED_Image_Function_one(11);OLED_Image_Function_one(12);OLED_Image_Function_one(13);OLED_Image_Function_one(14);
        OLED_Image_Function_one(15);OLED_Image_Function_one(16);OLED_Image_Function_one(17);OLED_Image_Function_one(18);OLED_Image_Function_one(19);
        OLED_Image_Function_one(20);OLED_Image_Function_one(21);OLED_Image_Function_one(22);OLED_Image_Function_one(23);OLED_Image_Function_one(24);
        OLED_Image_Function_one(25);OLED_Image_Function_one(26);OLED_Image_Function_one(27);OLED_Image_Function_one(28);OLED_Image_Function_one(29);
        OLED_Image_Function_one(30);OLED_Image_Function_one(31);OLED_Image_Function_one(32);OLED_Image_Function_one(33);OLED_Image_Function_one(34);
        OLED_Image_Function_one(35);OLED_Image_Function_one(36);OLED_Image_Function_one(36);OLED_Image_Function_one(37);OLED_Image_Function_one(38);
        OLED_Image_Function_one(39);OLED_Image_Function_one(40);OLED_Image_Function_one(41);OLED_Image_Function_one(42);OLED_Image_Function_one(43);
        OLED_Image_Function_one(44);OLED_Image_Function_one(45);OLED_Image_Function_one(46);OLED_Image_Function_one(47);OLED_Image_Function_one(48);
        OLED_Image_Function_one(49);OLED_Image_Function_one(50);
        sprintf(ch1,"hop:      %d   *",hop);
        oled_p6x8str(0,7,ch1);
        hop=OLED_Function_One(hop,5);                    //**//
        oled_fill(0x00);
        //左右翻页(right和left翻页)
        OLED_Function_Five();
    }
    if(optionOne==2)//搜线信息显示
    {
        sprintf(ch1," %d %d %d %d %d %d%d  ",leftline[front_text0],midline[front_text0],rightline[front_text0],width_out[front_text0]
                                            ,result_midline[front_text0],left_flag[front_text0],right_flag[front_text0]);
        oled_p6x8str(0,0,ch1);
        sprintf(ch1," %d %d %d %d %d %d%d  ",leftline[front_text1],midline[front_text1],rightline[front_text1],width_out[front_text1]
                                             ,result_midline[front_text1],left_flag[front_text1],right_flag[front_text1]);
        oled_p6x8str(0,1,ch1);
        sprintf(ch1," %d %d %d %d %d %d%d  ",leftline[front_text2],midline[front_text2],rightline[front_text2],width_out[front_text2]
                                             ,result_midline[front_text2],left_flag[front_text2],right_flag[front_text2]);
        oled_p6x8str(0,2,ch1);
        sprintf(ch1," %d %d %d %d %d %d%d  ",leftline[front_text3],midline[front_text3],rightline[front_text3],width_out[front_text3]
                                             ,result_midline[front_text3],left_flag[front_text3],right_flag[front_text3]);
        oled_p6x8str(0,3,ch1);
        sprintf(ch1," %d %d %d %d %d %d%d  ",leftline[front_text4],midline[front_text4],rightline[front_text4],width_out[front_text4]
                                             ,result_midline[front_text4],left_flag[front_text4],right_flag[front_text4]);
        oled_p6x8str(0,4,ch1);
        sprintf(ch1," %d %d %d %d %d %d     ",line_white_dot_sum[front_text2-2],line_black_dot_sum[front_text1-2]
                                             ,line_white_dot_sum[front_text2  ],line_black_dot_sum[front_text2  ]
                                             ,line_white_dot_sum[front_text2+2],line_black_dot_sum[front_text2+2]);
        oled_p6x8str(0,5,ch1);
        sprintf(ch1," %d               ",runtimemax);
        oled_p6x8str(0,6,ch1);
        if(optionTwo==0)
        {
            sprintf(ch1," %d %d %d %d %d %d  ",front_text0,front_text1,front_text2,front_text3,front_text4,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            front_text0=OLED_Function_One(front_text0,1);                    //**//
        }
        if(optionTwo==1)
        {
            sprintf(ch1," %d %d %d %d %d %d  ",front_text0,front_text1,front_text2,front_text3,front_text4,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            front_text1=OLED_Function_One(front_text1,1);                    //**//
        }
        if(optionTwo==2)
        {
            sprintf(ch1," %d %d %d %d %d %d  ",front_text0,front_text1,front_text2,front_text3,front_text4,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            front_text2=OLED_Function_One(front_text2,1);                    //**//
        }
        if(optionTwo==3)
        {
            sprintf(ch1," %d %d %d %d %d %d  ",front_text0,front_text1,front_text2,front_text3,front_text4,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            front_text3=OLED_Function_One(front_text3,1);                    //**//
        }
        if(optionTwo==4)
        {
            sprintf(ch1," %d %d %d %d %d %d  ",front_text0,front_text1,front_text2,front_text3,front_text4,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            front_text4=OLED_Function_One(front_text4,1);                    //**//
        }
        if(optionTwo==5)
        {

            sprintf(ch1," %d %d %d %d %d %d  ",hop,midline_stand,width_stand_out[front_text2],0,0,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            hop=OLED_Function_One(hop,1);                                    //**//
        }
        if(optionTwo==6)
        {

            sprintf(ch1," %d %d %d %d %d %d  ",hop,midline_stand,width_stand_out[front_text2],0,0,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            midline_stand=OLED_Function_One(midline_stand,1);                                    //**//
        }
        if(optionTwo==7)
        {

            sprintf(ch1," %d %d %d %d %d %d  ",hop,midline_stand,width_stand_out[front_text2],0,0,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            width_stand_out[front_text2]=OLED_Function_One(width_stand_out[front_text2],1);                                    //**//
        }
        //内限幅
        if(optionTwo>7)optionTwo=0;
        if(optionTwo<0)optionTwo=7;
        //指针左右移动
        OLED_Function_Three();
        //左右翻页
        OLED_Function_Four();
    }
    if(optionOne==3)
    {
        sprintf(ch1,"%.2f %d             ",SpeedSet,encoder_speed);
        oled_p6x8str(0,0,ch1);

        if(optionTwo==0)
        {
            sprintf(ch1,"*%.0f %.0f %.0f %.0f %d ",SpeedSet_Nnomal,SpeedSet_ramp[0],SpeedSet_ramp[1],SpeedSet_ramp[2],optionTwo%4+1);
            oled_p6x8str(0,7,ch1);
            SpeedSet_Nnomal=OLED_Function_Two(SpeedSet_Nnomal,1);                             //**//
        }
        if(optionTwo==1)
        {
            sprintf(ch1,"%.0f*%.0f %.0f %.2f %d ",SpeedSet_Nnomal,SpeedSet_ramp[0],SpeedSet_ramp[1],SpeedSet_ramp[2],optionTwo%4+1);
            oled_p6x8str(0,7,ch1);
            SpeedSet_ramp[0]=OLED_Function_Two(SpeedSet_ramp[0],1);                             //**//
        }
        if(optionTwo==2)
        {
            sprintf(ch1,"%.0f %.0f*%.0f %.0f %d ",SpeedSet_Nnomal,SpeedSet_ramp[0],SpeedSet_ramp[1],SpeedSet_ramp[2],optionTwo%4+1);
            oled_p6x8str(0,7,ch1);
            SpeedSet_ramp[1]=OLED_Function_Two(SpeedSet_ramp[1],1);                             //**//
        }
        if(optionTwo==3)
        {
            sprintf(ch1,"%.0f %.0f %.0f*%.0f %d ",SpeedSet_Nnomal,SpeedSet_ramp[0],SpeedSet_ramp[1],SpeedSet_ramp[2],optionTwo%4+1);
            oled_p6x8str(0,7,ch1);
            SpeedSet_ramp[2]=OLED_Function_Two(SpeedSet_ramp[2],1);                             //**//
        }
        if(optionTwo==4)
        {
            sprintf(ch1,"*%.0f %.0f %.0f  %d ",SpeedSet_ramp[3],SpeedSet_ramp[4],SpeedSet_ramp[5],optionTwo%4+1);
            oled_p6x8str(0,7,ch1);
            SpeedSet_ramp[3]=OLED_Function_Two(SpeedSet_ramp[3],1);                             //**//
        }
        if(optionTwo==5)
        {
            sprintf(ch1,"*%.0f %.0f %.0f  %d ",SpeedSet_ramp[3],SpeedSet_ramp[4],SpeedSet_ramp[5],optionTwo%4+1);
            oled_p6x8str(0,7,ch1);
            SpeedSet_ramp[4]=OLED_Function_Two(SpeedSet_ramp[4],1);                             //**//
        }
        if(optionTwo==6)
        {
            sprintf(ch1,"*%.0f %.0f %.0f  %d ",SpeedSet_ramp[3],SpeedSet_ramp[4],SpeedSet_ramp[5],optionTwo%4+1);
            oled_p6x8str(0,7,ch1);
            SpeedSet_ramp[5]=OLED_Function_Two(SpeedSet_ramp[5],1);                             //**//
        }
        //内限幅
        if(optionTwo>6)optionTwo=0;
        if(optionTwo<0)optionTwo=6;
        //指针左右移动
        OLED_Function_Three();
        //左右翻页
        OLED_Function_Four();
    }
    if(optionOne==0)
    {
        sprintf(ch1,"%.2f %.2f %d ",ServoFuzzyErr,AnnuluAngle,Annulu_Flag);
        oled_p6x8str(0,0,ch1);
        sprintf(ch1,"%.2f %.2f ",ServoKp,ServoKd);
        oled_p6x8str(0,1,ch1);

        if(optionTwo==0)
        {
            sprintf(ch1,"*%d %d %d %d %d  ",Annulu_Radius,Annulu_Two_LeftTurn[0],Annulu_Two_RightTurn[0],Annulu_Two_LeftTurn[1],Annulu_Two_RightTurn[1]);
            oled_p6x8str(0,7,ch1);
            Annulu_Radius=OLED_Function_One(Annulu_Radius,1);                             //**//
            if(Annulu_Radius>3)Annulu_Radius=0;
            if(Annulu_Radius<0)Annulu_Radius=3;
        }
        if(optionTwo==1)
        {
            sprintf(ch1,"%d*%d %d %d %d  ",Annulu_Radius,Annulu_Two_LeftTurn[0],Annulu_Two_RightTurn[0],Annulu_Two_LeftTurn[1],Annulu_Two_RightTurn[1]);
            oled_p6x8str(0,7,ch1);
            Annulu_Two_LeftTurn[0]=OLED_Function_One(Annulu_Two_LeftTurn[0],5);                             //**//
        }
        if(optionTwo==2)
        {
            sprintf(ch1,"%d %d*%d %d %d  ",Annulu_Radius,Annulu_Two_LeftTurn[0],Annulu_Two_RightTurn[0],Annulu_Two_LeftTurn[1],Annulu_Two_RightTurn[1]);
            oled_p6x8str(0,7,ch1);
            Annulu_Two_RightTurn[0]=OLED_Function_One(Annulu_Two_RightTurn[0],5);                             //**//
        }
        if(optionTwo==3)
        {
            sprintf(ch1,"%d %d %d*%d %d  ",Annulu_Radius,Annulu_Two_LeftTurn[0],Annulu_Two_RightTurn[0],Annulu_Two_LeftTurn[1],Annulu_Two_RightTurn[1]);
            oled_p6x8str(0,7,ch1);
            Annulu_Two_LeftTurn[1]=OLED_Function_One(Annulu_Two_LeftTurn[1],5);                             //**//
        }
        if(optionTwo==4)
        {
            sprintf(ch1,"%d %d %d %d*%d  ",Annulu_Radius,Annulu_Two_LeftTurn[0],Annulu_Two_RightTurn[0],Annulu_Two_LeftTurn[1],Annulu_Two_RightTurn[1]);
            oled_p6x8str(0,7,ch1);
            Annulu_Two_RightTurn[1]=OLED_Function_One(Annulu_Two_RightTurn[1],5);                             //**//
        }
        if(optionTwo==5)
        {
            sprintf(ch1,"*%d %d %d %d  ",Annulu_Two_LeftTurn[2],Annulu_Two_RightTurn[2],Annulu_Two_LeftTurn[3],Annulu_Two_RightTurn[3]);
            oled_p6x8str(0,7,ch1);
            Annulu_Two_LeftTurn[2]=OLED_Function_One(Annulu_Two_LeftTurn[2],5);                             //**//
        }
        if(optionTwo==6)
        {
            sprintf(ch1,"%d*%d %d %d   ",Annulu_Two_LeftTurn[2],Annulu_Two_RightTurn[2],Annulu_Two_LeftTurn[3],Annulu_Two_RightTurn[3]);
            oled_p6x8str(0,7,ch1);
            Annulu_Two_RightTurn[2]=OLED_Function_One(Annulu_Two_RightTurn[2],5);                             //**//
        }
        if(optionTwo==7)
        {
            sprintf(ch1,"%d %d*%d %d   ",Annulu_Two_LeftTurn[2],Annulu_Two_RightTurn[2],Annulu_Two_LeftTurn[3],Annulu_Two_RightTurn[3]);
            oled_p6x8str(0,7,ch1);
            Annulu_Two_LeftTurn[3]=OLED_Function_One(Annulu_Two_LeftTurn[3],5);                             //**//
        }
        if(optionTwo==8)
        {
            sprintf(ch1,"%d %d %d*%d   ",Annulu_Two_LeftTurn[2],Annulu_Two_RightTurn[2],Annulu_Two_LeftTurn[3],Annulu_Two_RightTurn[3]);
            oled_p6x8str(0,7,ch1);
            Annulu_Two_RightTurn[3]=OLED_Function_One(Annulu_Two_RightTurn[3],5);                             //**//
        }
        if(optionTwo==9)
        {
            sprintf(ch1,"*%.2f %.2f %.2f   ",AnnuluAngleLimit_Left[0],AnnuluAngleLimit_Left[1],AnnuluAngleLimit_Left[2]);
            oled_p6x8str(0,7,ch1);
            AnnuluAngleLimit_Left[0]=OLED_Function_Two(AnnuluAngleLimit_Left[0],5);                             //**//
        }
        if(optionTwo==10)
        {
            sprintf(ch1,"%.2f*%.2f %.2f   ",AnnuluAngleLimit_Left[0],AnnuluAngleLimit_Left[1],AnnuluAngleLimit_Left[2]);
            oled_p6x8str(0,7,ch1);
            AnnuluAngleLimit_Left[1]=OLED_Function_Two(AnnuluAngleLimit_Left[1],5);                             //**//
        }
        if(optionTwo==11)
        {
            sprintf(ch1,"%.2f %.2f*%.2f   ",AnnuluAngleLimit_Left[0],AnnuluAngleLimit_Left[1],AnnuluAngleLimit_Left[2]);
            oled_p6x8str(0,7,ch1);
            AnnuluAngleLimit_Left[2]=OLED_Function_Two(AnnuluAngleLimit_Left[2],5);                             //**//
        }
        if(optionTwo==12)
        {
            sprintf(ch1,"*%.2f %.2f %.2f   ",AnnuluAngleLimit_Right[0],AnnuluAngleLimit_Right[1],AnnuluAngleLimit_Right[2]);
            oled_p6x8str(0,7,ch1);
            AnnuluAngleLimit_Right[0]=OLED_Function_Two(AnnuluAngleLimit_Right[0],5);                             //**//
        }
        if(optionTwo==13)
        {
            sprintf(ch1,"%.2f*%.2f %.2f   ",AnnuluAngleLimit_Right[0],AnnuluAngleLimit_Right[1],AnnuluAngleLimit_Right[2]);
            oled_p6x8str(0,7,ch1);
            AnnuluAngleLimit_Right[1]=OLED_Function_Two(AnnuluAngleLimit_Right[1],5);                             //**//
        }
        if(optionTwo==14)
        {
            sprintf(ch1,"%.2f %.2f*%.2f   ",AnnuluAngleLimit_Right[0],AnnuluAngleLimit_Right[1],AnnuluAngleLimit_Right[2]);
            oled_p6x8str(0,7,ch1);
            AnnuluAngleLimit_Right[2]=OLED_Function_Two(AnnuluAngleLimit_Right[2],5);                             //**//
        }
        if(optionTwo==15)
        {
            sprintf(ch1,"*%.2f %.2f   ",ServoKp_Annulu[Annulu_Radius],ServoKd_Annulu[Annulu_Radius]);
            oled_p6x8str(0,7,ch1);
            ServoKp_Annulu[Annulu_Radius]=OLED_Function_Two(ServoKp_Annulu[Annulu_Radius],0.05);                             //**//
        }
        if(optionTwo==16)
        {
            sprintf(ch1," %.2f*%.2f   ",ServoKp_Annulu[Annulu_Radius],ServoKd_Annulu[Annulu_Radius]);
            oled_p6x8str(0,7,ch1);
            ServoKd_Annulu[Annulu_Radius]=OLED_Function_Two(ServoKd_Annulu[Annulu_Radius],1);                             //**//
        }
        //内限幅
        if(optionTwo>16)optionTwo=0;
        if(optionTwo<0)optionTwo=16;
        //指针左右移动
        OLED_Function_Three();
        //左右翻页
        OLED_Function_Four();
    }

    if(optionOne==5)
    {
        sprintf(ch1," %.2f %d %.2f ",OutgarageAngle,ZebraDistance_Real,ServoFuzzyOutput);
        oled_p6x8str(0,0,ch1);
        sprintf(ch1," %d %d %d %d %d %d ",ZebraWtoB_count[ZebraCrossingLine1-6],ZebraBtoW_count[ZebraCrossingLine1-6]
                                         ,ZebraWtoB_count[ZebraCrossingLine1-5],ZebraBtoW_count[ZebraCrossingLine1-5]
                                         ,ZebraWtoB_count[ZebraCrossingLine1-4],ZebraBtoW_count[ZebraCrossingLine1-4]);
        oled_p6x8str(0,1,ch1);
        sprintf(ch1," %d %d %d %d %d %d ",ZebraWtoB_count[ZebraCrossingLine1-3],ZebraBtoW_count[ZebraCrossingLine1-3]
                                         ,ZebraWtoB_count[ZebraCrossingLine1-2],ZebraBtoW_count[ZebraCrossingLine1-2]
                                         ,ZebraWtoB_count[ZebraCrossingLine1-1],ZebraBtoW_count[ZebraCrossingLine1-1]);
        oled_p6x8str(0,2,ch1);
        sprintf(ch1," %d %d %d %d %d %d ",ZebraWtoB_count[ZebraCrossingLine1],ZebraBtoW_count[ZebraCrossingLine1]
                                         ,ZebraWtoB_count[ZebraCrossingLine1+1],ZebraBtoW_count[ZebraCrossingLine1+1]
                                         ,0,0                                                                        );
        oled_p6x8str(0,3,ch1);
        sprintf(ch1,"%d %d %d %.2f %d   ",ZebraCrossingFlag,ZebraErrorCount,ZebraError,ZebraAngle,PID_Option);
        oled_p6x8str(0,4,ch1);
        if(optionTwo==0)
        {
            sprintf(ch1," %d %d %d %d %d %d  ",OutgarageFlag,OutgarageDirection,OutgarageServoDuty[0],OutgarageServoDuty[1],OutgarageSpeed,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            OutgarageFlag=OLED_Function_One(OutgarageFlag,1);                                    //**//
            if(OutgarageFlag>2)OutgarageFlag=0;
            if(OutgarageFlag<0)OutgarageFlag=2;
        }
        if(optionTwo==1)
        {
            sprintf(ch1," %d %d %d %d %d %d  ",OutgarageFlag,OutgarageDirection,OutgarageServoDuty[0],OutgarageServoDuty[1],OutgarageSpeed,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            OutgarageDirection=OLED_Function_One(OutgarageDirection,1);                                    //**//
            if(OutgarageDirection>1)OutgarageDirection=0;
            if(OutgarageDirection<0)OutgarageDirection=1;
        }
        if(optionTwo==2)
        {
            sprintf(ch1," %d %d %d %d %d %d  ",OutgarageFlag,OutgarageDirection,OutgarageServoDuty[0],OutgarageServoDuty[1],OutgarageSpeed,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            OutgarageServoDuty[0]=OLED_Function_One(OutgarageServoDuty[0],5);                             //**//
        }
        if(optionTwo==3)
        {
            sprintf(ch1," %d %d %d %d %d %d  ",OutgarageFlag,OutgarageDirection,OutgarageServoDuty[0],OutgarageServoDuty[1],OutgarageSpeed,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            OutgarageServoDuty[1]=OLED_Function_One(OutgarageServoDuty[1],5);                             //**//
        }
        if(optionTwo==4)
        {
            sprintf(ch1," %d %d %d %d %d %d  ",OutgarageFlag,OutgarageDirection,OutgarageServoDuty[0],OutgarageServoDuty[1],OutgarageSpeed,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            OutgarageSpeed=OLED_Function_One(OutgarageSpeed,5);                                           //**//
        }
        if(optionTwo==5)
        {
            sprintf(ch1," %.2f %d %d %d %d ",OutgarageAngleLimit,ZebraDistance_Limit,ZebraCrossingLine1,ZebraSpeed,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            OutgarageAngleLimit=OLED_Function_Two(OutgarageAngleLimit,5);                                           //**//
        }
        if(optionTwo==6)
        {
            sprintf(ch1," %.2f %d %d %d %d ",OutgarageAngleLimit,ZebraDistance_Limit,ZebraCrossingLine1,ZebraSpeed,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            ZebraDistance_Limit=OLED_Function_One(ZebraDistance_Limit,100);                                           //**//
        }
        if(optionTwo==7)
        {
            sprintf(ch1," %.2f %d %d %d %d ",OutgarageAngleLimit,ZebraDistance_Limit,ZebraCrossingLine1,ZebraSpeed,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            ZebraCrossingLine1=OLED_Function_One(ZebraCrossingLine1,1);                                           //**//
        }
        if(optionTwo==8)
        {
            sprintf(ch1," %.2f %d %d %d %d ",OutgarageAngleLimit,ZebraDistance_Limit,ZebraCrossingLine1,ZebraSpeed,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            ZebraSpeed=OLED_Function_One(ZebraSpeed,1);                                           //**//
        }
        if(optionTwo==9)
        {
            sprintf(ch1," %d %d %d %d %d ",EndLeftturn,EndRightturn,0,0,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            EndLeftturn=OLED_Function_One(EndLeftturn,5);                                           //**//
        }
        if(optionTwo==10)
        {
            sprintf(ch1," %d %d %d %d %d ",EndLeftturn,EndRightturn,0,0,optionTwo%5+1);
            oled_p6x8str(0,7,ch1);
            EndRightturn=OLED_Function_One(EndRightturn,5);                                           //**//
        }
        //内限幅
        if(optionTwo>10)optionTwo=0;
        if(optionTwo<0)optionTwo=10;
        //指针左右移动
        OLED_Function_Three();
        //左右翻页
        OLED_Function_Four();
    }
    if(optionOne==6)
    {
        sprintf(ch1," %d %d %d %d %d  ",ZebraWtoB_column[ZebraCrossingLine1][0],ZebraWtoB_column[ZebraCrossingLine1][1]
                                       ,ZebraWtoB_column[ZebraCrossingLine1][2],ZebraWtoB_column[ZebraCrossingLine1][3]
                                       ,ZebraWtoB_column[ZebraCrossingLine1][4]);
        oled_p6x8str(0,0,ch1);
        sprintf(ch1," %d %d %d %d %d  ",ZebraWtoB_column[ZebraCrossingLine1][5],ZebraWtoB_column[ZebraCrossingLine1][5]
                                       ,ZebraWtoB_column[ZebraCrossingLine1][8],ZebraWtoB_column[ZebraCrossingLine1][7]
                                       ,ZebraWtoB_column[ZebraCrossingLine1][9]);
        oled_p6x8str(0,1,ch1);
        sprintf(ch1," %d %d %d %d %d  ",ZebraWtoB_column[ZebraCrossingLine1][10],ZebraWtoB_column[ZebraCrossingLine1][11]
                                       ,ZebraWtoB_column[ZebraCrossingLine1][12],ZebraWtoB_column[ZebraCrossingLine1][13]
                                       ,ZebraWtoB_column[ZebraCrossingLine1][14]);
        oled_p6x8str(0,2,ch1);
        sprintf(ch1," %d %d %d %d %d  ",ZebraBtoW_column[ZebraCrossingLine1][0],ZebraBtoW_column[ZebraCrossingLine1][1]
                                       ,ZebraBtoW_column[ZebraCrossingLine1][2],ZebraBtoW_column[ZebraCrossingLine1][3]
                                       ,ZebraBtoW_column[ZebraCrossingLine1][4]);
        oled_p6x8str(0,3,ch1);
        sprintf(ch1," %d %d %d %d %d  ",ZebraBtoW_column[ZebraCrossingLine1][5],ZebraBtoW_column[ZebraCrossingLine1][5]
                                       ,ZebraBtoW_column[ZebraCrossingLine1][8],ZebraBtoW_column[ZebraCrossingLine1][7]
                                       ,ZebraBtoW_column[ZebraCrossingLine1][9]);
        oled_p6x8str(0,4,ch1);
        sprintf(ch1," %d %d %d %d %d  ",ZebraBtoW_column[ZebraCrossingLine1][10],ZebraBtoW_column[ZebraCrossingLine1][11]
                                       ,ZebraBtoW_column[ZebraCrossingLine1][12],ZebraBtoW_column[ZebraCrossingLine1][13]
                                       ,ZebraBtoW_column[ZebraCrossingLine1][14]);
        oled_p6x8str(0,5,ch1);
        //左右翻页(right和left翻页)
        OLED_Function_Five();
    }
    if(optionOne==7)
    {
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[1],LeftLineSlope[2],LeftLineSlope[3],LeftLineSlope[4]);
        oled_p6x8str(0,0,ch1);
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[5],LeftLineSlope[6],LeftLineSlope[7],LeftLineSlope[8]);
        oled_p6x8str(0,1,ch1);
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[9],LeftLineSlope[10],LeftLineSlope[11],LeftLineSlope[12]);
        oled_p6x8str(0,2,ch1);
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[13],LeftLineSlope[14],LeftLineSlope[15],LeftLineSlope[16]);
        oled_p6x8str(0,3,ch1);
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[17],LeftLineSlope[18],LeftLineSlope[19],LeftLineSlope[20]);
        oled_p6x8str(0,4,ch1);
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[21],LeftLineSlope[22],LeftLineSlope[23],LeftLineSlope[24]);
        oled_p6x8str(0,5,ch1);
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[25],LeftLineSlope[26],LeftLineSlope[27],LeftLineSlope[28]);
        oled_p6x8str(0,6,ch1);
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[29],LeftLineSlope[30],LeftLineSlope[31],LeftLineSlope[32]);
        oled_p6x8str(0,7,ch1);
        //左右翻页(right和left翻页)
        OLED_Function_Five();
    }
    if(optionOne==8)
    {
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[33],LeftLineSlope[34],LeftLineSlope[35],LeftLineSlope[36]);
        oled_p6x8str(0,0,ch1);
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[37],LeftLineSlope[38],LeftLineSlope[39],LeftLineSlope[40]);
        oled_p6x8str(0,1,ch1);
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[41],LeftLineSlope[42],LeftLineSlope[43],LeftLineSlope[44]);
        oled_p6x8str(0,2,ch1);
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[45],LeftLineSlope[46],LeftLineSlope[47],LeftLineSlope[48]);
        oled_p6x8str(0,3,ch1);
        sprintf(ch1,"%d %d %d %d ",LeftLineSlope[49],0,0,0);
        oled_p6x8str(0,4,ch1);
        //左右翻页(right和left翻页)
        OLED_Function_Five();
    }
    if(optionOne==9)
    {
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[1],RightLineSlope[2],RightLineSlope[3],RightLineSlope[4]);
        oled_p6x8str(0,0,ch1);
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[5],RightLineSlope[6],RightLineSlope[7],RightLineSlope[8]);
        oled_p6x8str(0,1,ch1);
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[9],RightLineSlope[10],RightLineSlope[11],RightLineSlope[12]);
        oled_p6x8str(0,2,ch1);
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[13],RightLineSlope[14],RightLineSlope[15],RightLineSlope[16]);
        oled_p6x8str(0,3,ch1);
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[17],RightLineSlope[18],RightLineSlope[19],RightLineSlope[20]);
        oled_p6x8str(0,4,ch1);
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[21],RightLineSlope[22],RightLineSlope[23],RightLineSlope[24]);
        oled_p6x8str(0,5,ch1);
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[25],RightLineSlope[26],RightLineSlope[27],RightLineSlope[28]);
        oled_p6x8str(0,6,ch1);
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[29],RightLineSlope[30],RightLineSlope[31],RightLineSlope[32]);
        oled_p6x8str(0,7,ch1);
        //左右翻页(right和left翻页)
        OLED_Function_Five();
    }
    if(optionOne==10)
    {
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[33],RightLineSlope[34],RightLineSlope[35],RightLineSlope[36]);
        oled_p6x8str(0,0,ch1);
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[37],RightLineSlope[38],RightLineSlope[39],RightLineSlope[40]);
        oled_p6x8str(0,1,ch1);
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[41],RightLineSlope[42],RightLineSlope[43],RightLineSlope[44]);
        oled_p6x8str(0,2,ch1);
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[45],RightLineSlope[46],RightLineSlope[47],RightLineSlope[48]);
        oled_p6x8str(0,3,ch1);
        sprintf(ch1," %d %d %d %d     ",RightLineSlope[49],0,0,0);
        oled_p6x8str(0,4,ch1);
        //左右翻页(right和left翻页)
        OLED_Function_Five();
    }
    if(optionOne==11)
    {
        sprintf(ch1,"%d %d %d %d %d   ",LeftSlopeSortCount[0][4],LeftSlopeSortCount[0][3],LeftSlopeSortCount[0][2],LeftSlopeSortCount[0][1],LeftSlopeSortCount[0][0]);
        oled_p6x8str(0,0,ch1);
        sprintf(ch1,"%d %d %d %d %d   ",LeftSlopeSortCount[1][4],LeftSlopeSortCount[1][3],LeftSlopeSortCount[1][2],LeftSlopeSortCount[1][1],LeftSlopeSortCount[1][0]);
        oled_p6x8str(0,1,ch1);
        sprintf(ch1,"%d %d %d %d %d   ",LeftSlopeSortCount[2][4],LeftSlopeSortCount[2][3],LeftSlopeSortCount[2][2],LeftSlopeSortCount[2][1],LeftSlopeSortCount[2][0]);
        oled_p6x8str(0,2,ch1);
        sprintf(ch1,"%d %d %d %d %d   ",LeftSlopeSortCount[3][4],LeftSlopeSortCount[3][3],LeftSlopeSortCount[3][2],LeftSlopeSortCount[3][1],LeftSlopeSortCount[3][0]);
        oled_p6x8str(0,3,ch1);
        sprintf(ch1,"%d %d %d %d %d   ",LeftSlopeSortCount[4][4],LeftSlopeSortCount[4][3],LeftSlopeSortCount[4][2],LeftSlopeSortCount[4][1],LeftSlopeSortCount[4][0]);
        oled_p6x8str(0,4,ch1);
        //左右翻页(right和left翻页)
        OLED_Function_Five();
    }
    if(optionOne==12)
    {
        sprintf(ch1,"%d %d %d %d %d   ",RrightSlopeSortCount[0][0],RrightSlopeSortCount[0][1],RrightSlopeSortCount[0][2],RrightSlopeSortCount[0][3],RrightSlopeSortCount[0][4]);
        oled_p6x8str(0,0,ch1);
        sprintf(ch1,"%d %d %d %d %d   ",RrightSlopeSortCount[1][0],RrightSlopeSortCount[1][1],RrightSlopeSortCount[1][2],RrightSlopeSortCount[1][3],RrightSlopeSortCount[1][4]);
        oled_p6x8str(0,1,ch1);
        sprintf(ch1,"%d %d %d %d %d   ",RrightSlopeSortCount[2][0],RrightSlopeSortCount[2][1],RrightSlopeSortCount[2][2],RrightSlopeSortCount[2][3],RrightSlopeSortCount[2][4]);
        oled_p6x8str(0,2,ch1);
        sprintf(ch1,"%d %d %d %d %d   ",RrightSlopeSortCount[3][0],RrightSlopeSortCount[3][1],RrightSlopeSortCount[3][2],RrightSlopeSortCount[3][3],RrightSlopeSortCount[3][4]);
        oled_p6x8str(0,3,ch1);
        sprintf(ch1,"%d %d %d %d %d   ",RrightSlopeSortCount[4][0],RrightSlopeSortCount[4][1],RrightSlopeSortCount[4][2],RrightSlopeSortCount[4][3],RrightSlopeSortCount[4][4]);
        oled_p6x8str(0,4,ch1);
        //左右翻页(right和left翻页)
        OLED_Function_Five();
    }
    if(optionOne==13)
    {
        sprintf(ch1,"SAVE0: %.2f     ",InductanceSAVE[0]);
        oled_p6x8str(1,0,ch1);
        sprintf(ch1,"SAVE1: %.2f     ",InductanceSAVE[1]);
        oled_p6x8str(1,1,ch1);
        sprintf(ch1,"SAVE2: %.2f     ",InductanceSAVE[2]);
        oled_p6x8str(1,2,ch1);
        sprintf(ch1,"SAVE3: %.2f     ",InductanceSAVE[3]);
        oled_p6x8str(1,3,ch1);
        sprintf(ch1,"Deal : %.2f     ",InductanceDeal[0]);
        oled_p6x8str(1,4,ch1);
        sprintf(ch1,"UseRecult: %.2f ",InductanceUseRecult);
        oled_p6x8str(1,5,ch1);
        sprintf(ch1,"UseLast  : %.2f ",InductanceUseLast);
        oled_p6x8str(1,6,ch1);
        sprintf(ch1,"OutTrack : %d   *",OutTrack);
        oled_p6x8str(1,7,ch1);
        OutTrack=OLED_Function_One(OutTrack,5);
        //左右翻页(right和left翻页)
        OLED_Function_Five();
    }
//    if(optionOne==9)
//    {
//        sprintf(ch1,"%d %d %d %d %d %d ",LeftSlopeNOCount,RrightSlopeNOCount,LeftSlopeUPCount,RrightSlopeUPCount,Annulu_Direction,Annulu_Flag);
//        oled_p6x8str(0,0,ch1);
//
//        if(optionTwo==0)
//        {
//            sprintf(ch1,"%d %d %d %d %d %d ",AnnuluOneStart,AnnuluOneMiddle,AnnuluOneEnd,LeftSlopeUPLimit[0],LeftSlopeUPLimit[1],optionTwo%5+1);
//            oled_p6x8str(0,7,ch1);
//            AnnuluOneStart=OLED_Function_One(AnnuluOneStart,1);                                           //**//
//        }
//        if(optionTwo==1)
//        {
//            sprintf(ch1,"%d %d %d %d %d %d ",AnnuluOneStart,AnnuluOneMiddle,AnnuluOneEnd,LeftSlopeUPLimit[0],LeftSlopeUPLimit[1],optionTwo%5+1);
//            oled_p6x8str(0,7,ch1);
//            AnnuluOneMiddle=OLED_Function_One(AnnuluOneMiddle,1);                                           //**//
//        }
//        if(optionTwo==2)
//        {
//            sprintf(ch1,"%d %d %d %d %d %d ",AnnuluOneStart,AnnuluOneMiddle,AnnuluOneEnd,LeftSlopeUPLimit[0],LeftSlopeUPLimit[1],optionTwo%5+1);
//            oled_p6x8str(0,7,ch1);
//            AnnuluOneEnd=OLED_Function_One(AnnuluOneEnd,1);                                           //**//
//        }
//        if(optionTwo==3)
//        {
//            sprintf(ch1,"%d %d %d %d %d %d ",AnnuluOneStart,AnnuluOneMiddle,AnnuluOneEnd,LeftSlopeUPLimit[0],LeftSlopeUPLimit[1],optionTwo%5+1);
//            oled_p6x8str(0,7,ch1);
//            LeftSlopeUPLimit[0]=OLED_Function_One(LeftSlopeUPLimit[0],1);                                           //**//
//        }
//        if(optionTwo==4)
//        {
//            sprintf(ch1,"%d %d %d %d %d %d ",AnnuluOneStart,AnnuluOneMiddle,AnnuluOneEnd,LeftSlopeUPLimit[0],LeftSlopeUPLimit[1],optionTwo%5+1);
//            oled_p6x8str(0,7,ch1);
//            LeftSlopeUPLimit[1]=OLED_Function_One(LeftSlopeUPLimit[1],1);                                           //**//
//        }
//        if(optionTwo==5)
//        {
//            sprintf(ch1,"%d %d %d %d %d %d ",RrightSlopeUPLimit[0],RrightSlopeUPLimit[1],SlopeUPMargin,SlopeNOMargin[0],SlopeNOMargin[1],optionTwo%5+1);
//            oled_p6x8str(0,7,ch1);
//            RrightSlopeUPLimit[0]=OLED_Function_One(RrightSlopeUPLimit[0],1);                                           //**//
//        }
//        if(optionTwo==6)
//        {
//            sprintf(ch1,"%d %d %d %d %d %d ",RrightSlopeUPLimit[0],RrightSlopeUPLimit[1],SlopeUPMargin,SlopeNOMargin[0],SlopeNOMargin[1],optionTwo%5+1);
//            oled_p6x8str(0,7,ch1);
//            RrightSlopeUPLimit[1]=OLED_Function_One(RrightSlopeUPLimit[1],1);                                           //**//
//        }
//        if(optionTwo==7)
//        {
//            sprintf(ch1,"%d %d %d %d %d %d ",RrightSlopeUPLimit[0],RrightSlopeUPLimit[1],SlopeUPMargin,SlopeNOMargin[0],SlopeNOMargin[1],optionTwo%5+1);
//            oled_p6x8str(0,7,ch1);
//            SlopeUPMargin=OLED_Function_One(SlopeUPMargin,1);                                           //**//
//        }
//        if(optionTwo==8)
//        {
//            sprintf(ch1,"%d %d %d %d %d %d ",RrightSlopeUPLimit[0],RrightSlopeUPLimit[1],SlopeUPMargin,SlopeNOMargin[0],SlopeNOMargin[1],optionTwo%5+1);
//            oled_p6x8str(0,7,ch1);
//            SlopeNOMargin[0]=OLED_Function_One(SlopeNOMargin[0],1);                                           //**//
//        }
//        if(optionTwo==9)
//        {
//            sprintf(ch1,"%d %d %d %d %d %d ",RrightSlopeUPLimit[0],RrightSlopeUPLimit[1],SlopeUPMargin,SlopeNOMargin[0],SlopeNOMargin[1],optionTwo%5+1);
//            oled_p6x8str(0,7,ch1);
//            SlopeNOMargin[1]=OLED_Function_One(SlopeNOMargin[1],1);                                           //**//
//        }
//        //内限幅
//        if(optionTwo>9)optionTwo=0;
//        if(optionTwo<0)optionTwo=9;
//    }
//    if(optionOne==11)
//    {
//        sprintf(ch1,"%.2f %.2f      ",ServoKp,ServoKd);
//        oled_p6x8str(0,6,ch1);
//        if(optionTwo==0)
//        {
//            sprintf(ch1," *%d %d %d %d         ",Sub_PID[0],Sub_PID[1],Sub_PID[2],Sub_PID[3]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKp_image1,ServoKd_image1,ServoKp_image2);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKd_image2,ServoKp_image3,ServoKd_image3);
//            oled_p6x8str(0,2,ch1);
//            Sub_PID[0]=OLED_Function_One(Sub_PID[0],1);
//        }
//        if(optionTwo==1)
//        {
//            sprintf(ch1,"  %d *%d %d %d        ",Sub_PID[0],Sub_PID[1],Sub_PID[2],Sub_PID[3]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKp_image1,ServoKd_image1,ServoKp_image2);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKd_image2,ServoKp_image3,ServoKd_image3);
//            oled_p6x8str(0,2,ch1);
//            Sub_PID[1]=OLED_Function_One(Sub_PID[1],1);
//        }
//        if(optionTwo==2)
//        {
//            sprintf(ch1,"  %d %d *%d %d        ",Sub_PID[0],Sub_PID[1],Sub_PID[2],Sub_PID[3]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKp_image1,ServoKd_image1,ServoKp_image2);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKd_image2,ServoKp_image3,ServoKd_image3);
//            oled_p6x8str(0,2,ch1);
//            Sub_PID[2]=OLED_Function_One(Sub_PID[2],1);
//        }
//        if(optionTwo==3)
//        {
//            sprintf(ch1,"  %d %d %d *%d        ",Sub_PID[0],Sub_PID[1],Sub_PID[2],Sub_PID[3]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKp_image1,ServoKd_image1,ServoKp_image2);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKd_image2,ServoKp_image3,ServoKd_image3);
//            oled_p6x8str(0,2,ch1);
//            Sub_PID[3]=OLED_Function_One(Sub_PID[3],1);
//        }
//        if(optionTwo==4)
//        {
//            sprintf(ch1,"  %d %d %d  %d        ",Sub_PID[0],Sub_PID[1],Sub_PID[2],Sub_PID[3]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"  *%.2f %.2f %.2f     ",ServoKp_image1,ServoKd_image1,ServoKp_image2);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKd_image2,ServoKp_image3,ServoKd_image3);
//            oled_p6x8str(0,2,ch1);
//            ServoKp_image1=OLED_Function_Two(ServoKp_image1,0.05);
//
//        }
//        if(optionTwo==5)
//        {
//            sprintf(ch1,"  %d %d %d  %d        ",Sub_PID[0],Sub_PID[1],Sub_PID[2],Sub_PID[3]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"  %.2f *%.2f %.2f     ",ServoKp_image1,ServoKd_image1,ServoKp_image2);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKd_image2,ServoKp_image3,ServoKd_image3);
//            oled_p6x8str(0,2,ch1);
//            ServoKd_image1=OLED_Function_Two(ServoKd_image1,1);
//        }
//        if(optionTwo==6)
//        {
//            sprintf(ch1,"  %d %d %d  %d        ",Sub_PID[0],Sub_PID[1],Sub_PID[2],Sub_PID[3]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"  %.2f %.2f *%.2f     ",ServoKp_image1,ServoKd_image1,ServoKp_image2);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKd_image2,ServoKp_image3,ServoKd_image3);
//            oled_p6x8str(0,2,ch1);
//            ServoKp_image2=OLED_Function_Two(ServoKp_image2,0.05);
//        }
//        if(optionTwo==7)
//        {
//            sprintf(ch1,"  %d %d %d  %d        ",Sub_PID[0],Sub_PID[1],Sub_PID[2],Sub_PID[3]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKp_image1,ServoKd_image1,ServoKp_image2);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"  *%.2f %.2f %.2f     ",ServoKd_image2,ServoKp_image3,ServoKd_image3);
//            oled_p6x8str(0,2,ch1);
//            ServoKd_image2=OLED_Function_Two(ServoKd_image2,1);
//        }
//        if(optionTwo==8)
//        {
//            sprintf(ch1,"  %d %d %d  %d        ",Sub_PID[0],Sub_PID[1],Sub_PID[2],Sub_PID[3]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKp_image1,ServoKd_image1,ServoKp_image2);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"  %.2f *%.2f %.2f     ",ServoKd_image2,ServoKp_image3,ServoKd_image3);
//            oled_p6x8str(0,2,ch1);
//            ServoKp_image3=OLED_Function_Two(ServoKp_image3,0.05);
//        }
//        if(optionTwo==9)
//        {
//            sprintf(ch1,"  %d %d %d  %d        ",Sub_PID[0],Sub_PID[1],Sub_PID[2],Sub_PID[3]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"  %.2f %.2f %.2f      ",ServoKp_image1,ServoKd_image1,ServoKp_image2);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"  %.2f %.2f *%.2f     ",ServoKd_image2,ServoKp_image3,ServoKd_image3);
//            oled_p6x8str(0,2,ch1);
//            ServoKd_image3=OLED_Function_Two(ServoKd_image3,1);
//        }
//        if(optionTwo==10)
//        {
//            sprintf(ch1,"PID_Flag: %d     *",PID_Flag);
//            oled_p6x8str(0,3,ch1);
//            sprintf(ch1,"SeKp_all: %.2f    ",ServoKp_all);
//            oled_p6x8str(0,4,ch1);
//            sprintf(ch1,"SeKd_all: %.2f    ",ServoKd_all);
//            oled_p6x8str(0,5,ch1);
//            sprintf(ch1,"StoReset: %d    *",car_stop_reset);
//            oled_p6x8str(0,7,ch1);
//            PID_Flag=OLED_Function_One(PID_Flag,1);
//            if(PID_Flag>1)PID_Flag=0;
//            if(PID_Flag<0)PID_Flag=1;
//        }
//        if(optionTwo==11)
//        {
//            sprintf(ch1,"PID_Flag: %d       ",PID_Flag);
//            oled_p6x8str(0,3,ch1);
//            sprintf(ch1,"SeKp_all: %.2f    *",ServoKp_all);
//            oled_p6x8str(0,4,ch1);
//            sprintf(ch1,"SeKd_all: %.2f     ",ServoKd_all);
//            oled_p6x8str(0,5,ch1);
//            sprintf(ch1,"StoReset: %d    *",car_stop_reset);
//            oled_p6x8str(0,7,ch1);
//            ServoKp_all=OLED_Function_Two(ServoKp_all,0.05);
//        }
//        if(optionTwo==12)
//        {
//            sprintf(ch1,"PID_Flag: %d       ",PID_Flag);
//            oled_p6x8str(0,3,ch1);
//            sprintf(ch1,"SeKp_all: %.2f     ",ServoKp_all);
//            oled_p6x8str(0,4,ch1);
//            sprintf(ch1,"SeKd_all: %.2f    *",ServoKd_all);
//            oled_p6x8str(0,5,ch1);
//            sprintf(ch1,"StoReset: %d       ",car_stop_reset);
//            oled_p6x8str(0,7,ch1);
//            ServoKd_all=OLED_Function_Two(ServoKd_all,1);
//        }
//        if(optionTwo==13)
//        {
//            sprintf(ch1,"PID_Flag: %d       ",PID_Flag);
//            oled_p6x8str(0,3,ch1);
//            sprintf(ch1,"SeKp_all: %.2f     ",ServoKp_all);
//            oled_p6x8str(0,4,ch1);
//            sprintf(ch1,"SeKd_all: %.2f     ",ServoKd_all);
//            oled_p6x8str(0,5,ch1);
//            sprintf(ch1,"StoReset: %d      *",car_stop_reset);
//            oled_p6x8str(0,7,ch1);
//            if(gpio_get(KeyUp)==0)
//            {
//                count_oled++;
//                if(count_oled==1){car_stop_reset=0;car_stop_real_distance=0;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//            if(gpio_get(KeyDown)==0)
//            {
//                count_oled++;
//                if(count_oled==1){car_stop_reset=0;car_stop_real_distance=0;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//        }
//        //内限幅
//        if(optionTwo>13)optionTwo=0;
//        if(optionTwo<0)optionTwo=13;
//        //指针左右移动
//        OLED_Function_Three();
//        //左右翻页
//        OLED_Function_Four();
//    }

    //外限幅
    if(optionOne>13)optionOne=0;
    if(optionOne<0 )optionOne=13;
    //按键延迟计数参数归零
    if(gpio_get(KeyTip)==1 && gpio_get(KeyUp)==1 && gpio_get(KeyDown)==1 && gpio_get(KeyRight)==1 &&  gpio_get(KeyLeft)==1)count_oled=0;

//    if(optionOne==4)
//    {
//        if(optionTwo==0)
//        {
//            sprintf(ch1,"SKp_ind : %.2f    *",ServoKp_induc[0]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"SKd_ind : %.2f     ",ServoKd_induc[0]);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[1]);
//            oled_p6x8str(0,2,ch1);
//            sprintf(ch1,"SKd_ind : %.2f     ",ServoKd_induc[1]);
//            oled_p6x8str(0,3,ch1);
//            sprintf(ch1,"ServDuty: %d       ",ServoDuty);
//            oled_p6x8str(0,4,ch1);
//            sprintf(ch1,"SEMIDD  : %d       ",SERVOMIDDLE);
//            oled_p6x8str(0,5,ch1);
//            sprintf(ch1,"SELEFT  : %d       ",SERVOLEFT);
//            oled_p6x8str(0,6,ch1);
//            sprintf(ch1,"SERIGHT : %d       ",SERVORIGHT);
//            oled_p6x8str(0,7,ch1);
//            if(gpio_get(KeyRight)==0)
//            {
//                count_oled++;
//                if(count_oled==1){ServoKp_induc[0]+=0.5;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//            if(gpio_get(KeyLeft)==0)
//            {
//                count_oled++;
//                if(count_oled==1){ServoKp_induc[0]-=0.5;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//        }
//        if(optionTwo==1)
//        {
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[0]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"SKd_ind : %.2f    *",ServoKd_induc[0]);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[1]);
//            oled_p6x8str(0,2,ch1);
//            sprintf(ch1,"SKd_ind : %.2f     ",ServoKd_induc[1]);
//            oled_p6x8str(0,3,ch1);
//            sprintf(ch1,"ServDuty: %d       ",ServoDuty);
//            oled_p6x8str(0,4,ch1);
//            sprintf(ch1,"SEMIDD  : %d       ",SERVOMIDDLE);
//            oled_p6x8str(0,5,ch1);
//            sprintf(ch1,"SELEFT  : %d       ",SERVOLEFT);
//            oled_p6x8str(0,6,ch1);
//            sprintf(ch1,"SERIGHT : %d       ",SERVORIGHT);
//            oled_p6x8str(0,7,ch1);
//            if(gpio_get(KeyRight)==0)
//            {
//                count_oled++;
//                if(count_oled==1){ServoKd_induc[0]+=1;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//            if(gpio_get(KeyLeft)==0)
//            {
//                count_oled++;
//                if(count_oled==1){ServoKd_induc[0]-=1;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//        }
//        if(optionTwo==2)
//        {
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[0]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"SKd_ind : %.2f     ",ServoKd_induc[0]);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"SKp_ind : %.2f    *",ServoKp_induc[1]);
//            oled_p6x8str(0,2,ch1);
//            sprintf(ch1,"SKd_ind : %.2f     ",ServoKd_induc[1]);
//            oled_p6x8str(0,3,ch1);
//            sprintf(ch1,"ServDuty: %d       ",ServoDuty);
//            oled_p6x8str(0,4,ch1);
//            sprintf(ch1,"SEMIDD  : %d       ",SERVOMIDDLE);
//            oled_p6x8str(0,5,ch1);
//            sprintf(ch1,"SELEFT  : %d       ",SERVOLEFT);
//            oled_p6x8str(0,6,ch1);
//            sprintf(ch1,"SERIGHT : %d       ",SERVORIGHT);
//            oled_p6x8str(0,7,ch1);
//            if(gpio_get(KeyRight)==0)
//            {
//                count_oled++;
//                if(count_oled==1){ServoKp_induc[1]+=0.5;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//            if(gpio_get(KeyLeft)==0)
//            {
//                count_oled++;
//                if(count_oled==1){ServoKp_induc[1]-=0.5;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//        }
//        if(optionTwo==3)
//        {
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[0]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"SKd_ind : %.2f     ",ServoKd_induc[0]);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[1]);
//            oled_p6x8str(0,2,ch1);
//            sprintf(ch1,"SKd_ind : %.2f    *",ServoKd_induc[1]);
//            oled_p6x8str(0,3,ch1);
//            sprintf(ch1,"ServDuty: %d       ",ServoDuty);
//            oled_p6x8str(0,4,ch1);
//            sprintf(ch1,"SEMIDD  : %d       ",SERVOMIDDLE);
//            oled_p6x8str(0,5,ch1);
//            sprintf(ch1,"SELEFT  : %d       ",SERVOLEFT);
//            oled_p6x8str(0,6,ch1);
//            sprintf(ch1,"SERIGHT : %d       ",SERVORIGHT);
//            oled_p6x8str(0,7,ch1);
//            if(gpio_get(KeyRight)==0)
//            {
//                count_oled++;
//                if(count_oled==1){ServoKd_induc[1]+=1;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//            if(gpio_get(KeyLeft)==0)
//            {
//                count_oled++;
//                if(count_oled==1){ServoKd_induc[1]-=1;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//        }
//
//        if(optionTwo==4)
//        {
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[0]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"SKd_ind : %.2f     ",ServoKd_induc[0]);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[1]);
//            oled_p6x8str(0,2,ch1);
//            sprintf(ch1,"SKd_ind : %.2f    ",ServoKd_induc[1]);
//            oled_p6x8str(0,3,ch1);
//            sprintf(ch1,"ServDuty: %d       ",ServoDuty);
//            oled_p6x8str(0,4,ch1);
//            sprintf(ch1,"SEMIDD  : %d      *",SERVOMIDDLE);
//            oled_p6x8str(0,5,ch1);
//            sprintf(ch1,"SELEFT  : %d       ",SERVOLEFT);
//            oled_p6x8str(0,6,ch1);
//            sprintf(ch1,"SERIGHT : %d       ",SERVORIGHT);
//            oled_p6x8str(0,7,ch1);
//            if(gpio_get(KeyRight)==0)
//            {
//                count_oled++;
//                if(count_oled==1){SERVOMIDDLE+=5;Duty=SERVOMIDDLE;    oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//            if(gpio_get(KeyLeft)==0)
//            {
//                count_oled++;
//                if(count_oled==1){SERVOMIDDLE-=5;Duty=SERVOMIDDLE;    oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//        }
//        if(optionTwo==5)
//        {
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[0]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"SKd_ind : %.2f     ",ServoKd_induc[0]);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[1]);
//            oled_p6x8str(0,2,ch1);
//            sprintf(ch1,"SKd_ind : %.2f    ",ServoKd_induc[1]);
//            oled_p6x8str(0,3,ch1);
//            sprintf(ch1,"ServDuty: %d       ",ServoDuty);
//            oled_p6x8str(0,4,ch1);
//            sprintf(ch1,"SEMIDD  : %d       ",SERVOMIDDLE);
//            oled_p6x8str(0,5,ch1);
//            sprintf(ch1,"SELEFT  : %d      *",SERVOLEFT);
//            oled_p6x8str(0,6,ch1);
//            sprintf(ch1,"SERIGHT : %d       ",SERVORIGHT);
//            oled_p6x8str(0,7,ch1);
//            if(gpio_get(KeyRight)==0)
//            {
//                count_oled++;
//                if(count_oled==1){SERVOLEFT+=10;Duty=SERVOLEFT;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//            if(gpio_get(KeyLeft)==0)
//            {
//                count_oled++;
//                if(count_oled==1){SERVOLEFT-=10;Duty=SERVOLEFT;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//        }
//        if(optionTwo==6)
//        {
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[0]);
//            oled_p6x8str(0,0,ch1);
//            sprintf(ch1,"SKd_ind : %.2f     ",ServoKd_induc[0]);
//            oled_p6x8str(0,1,ch1);
//            sprintf(ch1,"SKp_ind : %.2f     ",ServoKp_induc[1]);
//            oled_p6x8str(0,2,ch1);
//            sprintf(ch1,"SKd_ind : %.2f    ",ServoKd_induc[1]);
//            oled_p6x8str(0,3,ch1);
//            sprintf(ch1,"ServDuty: %d       ",ServoDuty);
//            oled_p6x8str(0,4,ch1);
//            sprintf(ch1,"SEMIDD  : %d       ",SERVOMIDDLE);
//            oled_p6x8str(0,5,ch1);
//            sprintf(ch1,"SELEFT  : %d       ",SERVOLEFT);
//            oled_p6x8str(0,6,ch1);
//            sprintf(ch1,"SERIGHT : %d      *",SERVORIGHT);
//            oled_p6x8str(0,7,ch1);
//            if(gpio_get(KeyRight)==0)
//            {
//                count_oled++;
//                if(count_oled==1){SERVORIGHT+=10;Duty=SERVOMIDDLE;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//            if(gpio_get(KeyLeft)==0)
//            {
//                count_oled++;
//                if(count_oled==1){SERVORIGHT-=10;Duty=SERVORIGHT;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//        }
//        //内限幅
//        if(optionTwo>6)optionTwo=0;
//        if(optionTwo<0)optionTwo=6;
//    }
//    if(optionOne==10)
//    {
//        sprintf(ch1,"%.2f %.2f       ",LeftlineDerivative_wach,RightlineDerivative_wach);
//        oled_p6x8str(0,5,ch1);
//        sprintf(ch1,"            %d      ",tfmini_return);
//        oled_p6x8str(0,6,ch1);
//        sprintf(ch1,"runtime:   %d       ",runtime);
//        oled_p6x8str(0,7,ch1);
//
//        if(optionTwo==0)
//        {
//            sprintf(ch1,"  %d %d %d %d *%d    ",left_add,middle_add,right_add,comparative_result0,front_text0);
//            oled_p6x8str(0,2,ch1);
//            sprintf(ch1,"  %.2f    %.2f       ",LeftlineDerivative_limit,RightlineDerivative_limit);
//            oled_p6x8str(0,4,ch1);
//            if(gpio_get(KeyRight)==0)
//            {
//                count_oled++;
//                if(count_oled==1){front_text0+=1;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//            if(gpio_get(KeyLeft)==0)
//            {
//                count_oled++;
//                if(count_oled==1){front_text0-=1;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//        }
//        if(optionTwo==1)
//        {
//            sprintf(ch1," *%.2f    %.2f       ",LeftlineDerivative_limit,RightlineDerivative_limit);
//            oled_p6x8str(0,4,ch1);
//            if(gpio_get(KeyRight)==0)
//            {
//                count_oled++;
//                if(count_oled==1){LeftlineDerivative_limit+=0.05;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//            if(gpio_get(KeyLeft)==0)
//            {
//                count_oled++;
//                if(count_oled==1){LeftlineDerivative_limit-=0.05;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//        }
//        if(optionTwo==2)
//        {
//            sprintf(ch1," %.2f    *%.2f       ",LeftlineDerivative_limit,RightlineDerivative_limit);
//            oled_p6x8str(0,4,ch1);
//            if(gpio_get(KeyRight)==0)
//            {
//                count_oled++;
//                if(count_oled==1){RightlineDerivative_limit+=0.05;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//            if(gpio_get(KeyLeft)==0)
//            {
//                count_oled++;
//                if(count_oled==1){RightlineDerivative_limit-=0.05;   oled_fill(0x00);}
//                else if(count_oled>count_oled_time)count_oled=0;
//            }
//        }
//        //内限幅
//        if(optionTwo>2)optionTwo=0;
//        if(optionTwo<0)optionTwo=2;
//    }
//    if(optionOne==5)
//    {
//        sprintf(ch1,"BuzzCount: %d       ",Buzzing_count);
//        oled_p6x8str(0,0,ch1);
//        sprintf(ch1,"BuCoLimit: %d       ",Buzzing_count_limit);
//        oled_p6x8str(0,1,ch1);
//        sprintf(ch1,"BuzziFlag: %d       ",Buzzing_start_flag);
//        oled_p6x8str(0,2,ch1);
//        sprintf(ch1,"ramp_flag: %d       ",ramp_flag);
//        oled_p6x8str(0,3,ch1);
//
//        sprintf(ch1,"image_ave: %d       ",image_average[front_text1]);
//        oled_p6x8str(0,4,ch1);
//        sprintf(ch1,"image_ave: %d       ",image_average[front_text2]);
//        oled_p6x8str(0,5,ch1);
//        sprintf(ch1,"angle=use: %.2f     ",angle_re_use);
//        oled_p6x8str(0,6,ch1);
//        sprintf(ch1,"imavebott: %d       ",image_average_bottom);
//        oled_p6x8str(0,7,ch1);
//        if(gpio_get(KeyRight)==0)
//        {
//            count_oled++;
//            if(count_oled==1){image_average_bottom+=10;  oled_fill(0x00);}
//            else if(count_oled>count_oled_time)count_oled=0;
//        }
//        if(gpio_get(KeyLeft)==0)
//        {
//            count_oled++;
//            if(count_oled==1){image_average_bottom-=10;  oled_fill(0x00);}
//            else if(count_oled>count_oled_time)count_oled=0;
//        }
//
//    }

//    if(optionOne==3)
//    {
//        sprintf(ch1,"%.2f     ",icm_gyro_y_clean);
//        oled_p6x8str(0,0,ch1);
//        sprintf(ch1,"%.2f     ",icm_gyro_z_clean);
//        oled_p6x8str(0,1,ch1);
//        sprintf(ch1,"%.2f     ",w_y_re_use);
//        oled_p6x8str(0,2,ch1);
//        sprintf(ch1,"%.2f     ",w_z_re_use);
//        oled_p6x8str(0,3,ch1);
//        sprintf(ch1,"%.2f     ",angle_gyro);
//        oled_p6x8str(0,4,ch1);
//        sprintf(ch1,"%.2f     ",angle_acc);
//        oled_p6x8str(0,5,ch1);
//        sprintf(ch1,"%.2f     ",angle_re);
//        oled_p6x8str(0,6,ch1);
//    }
}
//加减变量  输入所要变化的变量及变化值
//整型
int OLED_Function_One(int ins_change,int change_count)
{
    if(gpio_get(KeyUp)==0)
    {
        count_oled++;
        if(count_oled==1){ins_change+=change_count;   oled_fill(0x00);}
        else if(count_oled>count_oled_time)count_oled=0;
    }
    if(gpio_get(KeyDown)==0)
    {
        count_oled++;
        if(count_oled==1){ins_change-=change_count;   oled_fill(0x00);}
        else if(count_oled>count_oled_time)count_oled=0;
    }
    return ins_change;
}
//浮点型
float OLED_Function_Two(float ins_change,float change_count)
{
    if(gpio_get(KeyUp)==0)
    {
        count_oled++;
        if(count_oled==1){ins_change+=change_count;   oled_fill(0x00);}
        else if(count_oled>count_oled_time)count_oled=0;
    }
    if(gpio_get(KeyDown)==0)
    {
        count_oled++;
        if(count_oled==1){ins_change-=change_count;   oled_fill(0x00);}
        else if(count_oled>count_oled_time)count_oled=0;
    }
    return ins_change;
}
//指针移动
void OLED_Function_Three(void)
{
    if(gpio_get(KeyRight)==0)
    {
        count_oled++;
        if(count_oled==1){optionTwo++;   oled_fill(0x00);}
        else if(count_oled>count_oled_time)count_oled=0;
    }
    if(gpio_get(KeyLeft)==0)
    {
        count_oled++;
        if(count_oled==1){optionTwo--;   oled_fill(0x00);}
        else if(count_oled>count_oled_time)count_oled=0;
    }
}
//左右翻页（tip翻页，left和right已经被使用）
void OLED_Function_Four(void)
{
    if(gpio_get(KeyTip)==0)
    {
        count_oled++;
        if(count_oled==1){optionOne++;optionTwo=0; oled_fill(0x00);}
        else if(count_oled>count_oled_time)count_oled=0;
    }
}
//左右翻页（left和right翻页）
void OLED_Function_Five(void)
{
    if(gpio_get(KeyRight)==0)
    {
        count_oled++;
        if(count_oled==1){optionOne++;   oled_fill(0x00);}
        else if(count_oled>count_oled_time)count_oled=0;
    }
    if(gpio_get(KeyLeft)==0)
    {
        count_oled++;
        if(count_oled==1){optionOne--;   oled_fill(0x00);}
        else if(count_oled>count_oled_time)count_oled=0;
    }
}
/***************************************************************************************
函 数 名 : void Transport_Protocolse(float Send_variable)
功     能 :分解待发送变量，同时将分解后各部分通过串口发送
传     入 :待发送变量
返 回 值 :无
整定参数 :无
***************************************************************************************/
void Transport_Protocolse(float Send_variable)
{
    Float_to_Byte(Send_variable,byte);
    seekfree_wireless_send_buff(&byte[0],1);
    seekfree_wireless_send_buff(&byte[1],1);
    seekfree_wireless_send_buff(&byte[2],1);
    seekfree_wireless_send_buff(&byte[3],1);
}
/***************************************************************************************
函 数 名 : void Float_to_Byte(float f,unsigned char byte[])
功    能 :将浮点数f转化为4个字节数据存放在byte[4]中
传    入 :无
返 回 值 :无
整定参数 :无
***************************************************************************************/
void Float_to_Byte(float f,unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata=f;
    byte[0]=(unsigned char)fl.ldata;
    byte[1]=(unsigned char)(fl.ldata>>8);
    byte[2]=(unsigned char)(fl.ldata>>16);
    byte[3]=(unsigned char)(fl.ldata>>24);
}
/***************************************************************************************
函 数 名 :int Bluetooth_Variable_Control(int Variable,int operation,int quantity)
功    能 :蓝牙变量控制
传    入 :被控制变量，加或减（1：加；2：减），加减数
返 回 值 :返回被控制变量
整定参数 :无
***************************************************************************************/
float Bluetooth_Variable_Control(float Variable,int operation,float quantity)
{
     if(operation==1)
     {
         count_UART2++;
         if(count_UART2==1) Variable+=quantity;
         else if(count_UART2>count_UART2_time)count_UART2=0;
     }
     else if(operation==2)
     {
         count_UART2++;
         if(count_UART2==1)Variable-=quantity;
         else if(count_UART2>count_UART2_time)count_UART2=0;
     }
     return Variable;
}
#pragma section all restore
