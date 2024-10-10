#include "Cpu0_Main.h"
#include "headfile.h"
#include "stdlib.h"
#pragma section all "cpu0_dsram"

/////////////////////////////////////正常运行标记//////////////////////////////////////////
int NormalOperationflag=0;      //正常运行标记 0为正常运行，1为在元素中
/////////////////////////////////////图像采集//////////////////////////////////////////
int practical_maxline = 50;            //进行图像信息搜索的最大行
int practical_image[120][188]={{0}};   //图像采集后将采集结果进行预存（转存）
uint8 mt9v03x_image_out[188]={0};
int mt9v03x_image_Resolution[188]={0};
int leftline [120]={0};  //实时左边界
int rightline[120]={0};  //实时右边界
int width_out[120]={0};  //检测行对应实时赛道宽度
int width_stand_out[120] ={0};  //各行标准宽度 拟合函数+补偿偏移
int midline  [120]={0};  //检测行对应实时中线
int result_midline[120]={0}; //处理后运用在pid中的中线值
int midline_stand=91;        //标准中线
int midline_error[120]={0};  //检测行对应实时偏差***
int image_average[120]={0};  //检测行的像素点均值***************************
int image_average_bottom =180; //前方是否为全黑的阈值
int turn_flag=0;             //转弯打死舵标志位及左转还是右转
int line_white_dot_sum[120]={0}; //一行中白点个数
int line_black_dot_sum[120]={0}; //一行中黑点个数

int left_limit = 10;
int right_limit = 180;
int line_search_start[120]={91};
int left_flag[120]={0};
int right_flag[120]={0};
int left_hop[120]={0};
int right_hop[120]={0};
int hop=115;             //跳变灰度值                                              //*
int hop_bottom = 10 ;//*//

int front_text0=10;     //有效行0 (用于判别前方信息)                              //*//
int front_text1=15;     //有效行1                                                 //*//
int front_text2=20;     //有效行2                                                 //*
int front_text3=35;     //有效行3                                                 //*//
int front_text4=55;     //有效行4                                                 //*//
int front_text5=80;     //有效行5                                                 //*//
int front_text6=100;    //有效行6                                                 //*//
//左右边界读取
float LeftlineArray[10]={0.0};
float RightlineArray[10]={0.0};
float LeftlineDerivativeLast = 0.0; //左边界上一次斜率
float RightlineDerivativeLast = 0.0;//右边界上一次斜率
float LeftlineDerivative = 0.0;     //左边界斜率
float RightlineDerivative = 0.0;    //右边界斜率

float LeftlineDerivative_limit  = -1.5;
float RightlineDerivative_limit =  1.5;
//图像信息检测
int three_row_white_flag = 0; //三行白色（检测行全白）
int image_average_limit_bottom=100;  //行像素均值下限（小于则全黑）
//边界斜率解算
int  LeftLineSlope[120]={-200};    //记录各行左边界斜率
int  RightLineSlope[120]={200};   //记录各行右边界斜率
int LeftSlopeLimit[4]=  {-1,-7,-20,-50}; //左边界斜率分类边界    可调
int RrightSlopeLimit[4]={ 1, 7, 20, 50}; //右边界斜率分类边界    可调
int Line_Demarcation[3]={10,20,35};      //行分界线
int LeftSlopeSortCount[5][5]={{0}};      //左边界斜率分类统计（[由上至下][由内至外]偏顺序统计）    可视
int RrightSlopeSortCount[5][5]={{0}};    //左边界斜率分类统计（[由上至下][由内至外]偏顺序统计）    可视
/////////////////////////////////////电机//////////////////////////////////////////////
int16 encoder_speed=0; //编码器
//电机测试用
int duty_front = 1500; //正转
int duty_behind = 0;   //反转
//速度值
float SpeedSet = 0;                         //速度设置***********************
float SpeedSet_Nnomal=75;                   //正常速度
float SpeedSet_ramp[6]={75,50,50,75,75,75}; //坡道速度
float SpeedSet_Annulu[4]={60,65,85,60};     //圆环速度
int   OutgarageSpeed = 75;                  //出库速度
int   ZebraSpeed = 60;                      //入库速度
float MotorNowSpeed = 0; //实时速度
float MotorExpSpeed = 0; //期望速度
//速度闭环
float MotorErrArray[10] = {0};//速度偏差存储
float MotorErrDerivativeLast = 0;//上一次速度偏差变化率
float MotorErrDerivative = 0;//速度偏差变化率

float MotorK =  25.0 ; //速度闭环kp（抑制跟随变化时的过度偏差）
float MotorD =  0.0 ; //速度闭环kd（抑制静态抖动）
float MotorI =  1.6 ; //速度闭环ki（硬度）
//速度输出
float MotorOutLast = 0;
float MotorOut = 0;
/////////////////////////////////////舵机//////////////////////////////////////////////
//舵机的偏角
int SERVOLEFT   = 560; //(-135)
int SERVOMIDDLE = 695;
int SERVORIGHT  = 830; //(+135)
//FuzzyPID

float ServoFuzzyErr = 0.0,ServoFuzzyErrStore[10] = {0.0},ServoFuzzyErrDerivative = 0.0;
float ServoFuzzyOutput = 0.0;
float ServoKp = 0.0;                                              //最终使用pid
float ServoKd = 0.0;
float ServoKp_induc[2] = {1.50,5.0};                                       //*
float ServoKd_induc[2] = {0.0,0.0};                                        //*
int PID_Flag=0;
int PID_Option=0; //pid选择，0为分段式pid，1为电磁正常跑动pid，2为电磁坡道pid，3为出库pid，默认为0
float ServoKp_image1 = 1.20;                                      //*1.6  1.4
float ServoKd_image1 = 15.0;                                      //*25   2.0
float ServoKp_image2 = 2.20;                                      //*2.6  2.6
float ServoKd_image2 = 30.0;                                      //*30   30
float ServoKp_image3 = 2.55;                                      //*1.6  1.7
float ServoKd_image3 = 40.0;                                      //*30   30
float ServoKp_all = 1.80;                                         //*
float ServoKd_all = 25.0;                                         //*
int Sub_PID[4]={-30,-8,8,30};
//舵机输出
int   ServoDuty = 0;//舵机实际用
int   Duty = 695;   //舵机测试用
/////////////////////////////////////电感信息//////////////////////////////////////////
uint16 InductanceGetlook[4] = {0};                         //初始电感值
uint16 InductanceGetlookMax[4] = {0};                      //电感最大值
uint16 InductanceGet[4] = {0};                             //归一化结果
float InductanceSAVE[4]= {0};                              //用于实际运行时存储电感采集值
float InductanceDeal[1] = {0.0};                           //差比积结果(偏差)
float InductanceUse[1] = {0.0};                            //偏差处理后的期望
float InductanceUseRecult = 0.0;                           //最终用于搜线的期望
float InductanceUseLast=0.0;                               //上一次用于搜线的期望
float InductanceUseWi=0.95;                                //一阶滞后滤波系数(系数*这次+(1-系数)*上次)
int   OutTrack =300;                                       //出赛道左右电感
/////////////////////////////////////最小二乘法////////////////////////////////////////
//求斜率数值个数
int LSDepth = 10;
//左端电感变化率
float InductanceGetStoreLeft[10] = {0.0};
float InductanceLeftDerivative= 0.0;
float InductanceLeftDerivative_last= 0.0;
float InductanceLeftDerivative_k= 0.0;
//右端电感变化率
float InductanceGetStoreRight[10] = {0.0};
float InductanceRightDerivative= 0.0;
float InductanceRightDerivative_last= 0.0;
float InductanceRightDerivative_k= 0.0;
//近端电感变化率(横)
float InductanceGetStoreNear[10] = {0.0};
float InductanceNearDerivative= 0.0;
float InductanceNearDerivative_last= 0.0;
float InductanceNearDerivative_k= 0.0;
//远端电感变化率（竖）
float InductanceGetStoreFar[10] = {0.0};
float InductanceFarDerivative= 0.0;
float InductanceFarDerivative_last= 0.0;
float InductanceFarDerivative_k= 0.0;
/////////////////////////////////////出库////////////////////////////////////////////
/*固定参数*/
int OutgarageFlag = 1;       //车辆是否出库，0准备出；1出，2结束
/*调节参数*/
int OutgarageDirection=1;    //车库方向0为左出库，1为右出库，默认为右出库（1）
int OutgarageServoDuty[2] = {110,-110};  //出库时舵机设定值
float OutgarageAngle=0;        //出库角度
float OutgarageAngleLimit=70;  //出库角度限制
//////////////////////////////////识别斑马线/////////////////////////////////////////
/*固定参数*/
int ZebraWtoB_count[120]={0}; //各行白变黑个数
int ZebraBtoW_count[120]={0}; //各行黑变白个数
int ZebraWtoB_column[120][15]={{0}};//白变黑跳变列
int ZebraBtoW_column[120][15]={{0}};//黑变白跳变列
int ZebraErrorSum=0;         //斑马线前车辆偏差和
int ZebraErrorCount=0;       //斑马线前有效行数
int ZebraError=0;            //斑马线前车辆偏差
int ZebraCrossingFlag=0;     //斑马线标记
/*调节参数*/
int ZebraCrossingLine1 =20;  //斑马线搜索行
float ZebraAngle=0.0;        //入库记角度
int ZebraAngleLimit=70;      //入库结束角度
int ZebraDistance_Real=0;    //入库行驶距离
int ZebraDistance_Limit=2500;//入库第一阶段行驶距离限制
int EndLeftturn=155;         //左入库打死舵机
int EndRightturn=-120;       //右入库打死舵机
/////////////////////////////////////圆环////////////////////////////////////////
int Annulu_Direction=0;    //圆环方向标记，0为正常，1为左圆环，2为右圆环
int Annulu_Flag=0;         //圆环标记，0为正常，
int Annulu_Flag_supplement=0;   //圆环标记环节中的小环节
int SlopePASSMargin[2][2]={{5,8},{2,13}};  //边界斜率行有效判定条件（大于此才算有效）(分为严格要求及非严格要求)
float AnnuluAngle=0;         //圆环陀螺仪角度
float AnnuluAngleLimit_Left[3]={-45,-230,-270};   //圆环陀螺仪角度限制（左）（判断在环中，即将出环，已出环）
float AnnuluAngleLimit_Right[3]={ 45, 230, 270};   //圆环陀螺仪角度限制（右）（判断在环中，即将出环，已出环）
//准备入环
int Annulu_one_count=0;     //准备入环中另一边有边界行数量
int Annulu_one_error_sum=0; //准备入环中有效行的偏差和
int Annulu_one_error=0;     //准备入环中有效行的偏差
//开始入环
//int Annulu_Two_count=0;     //开始入环中另一边有边界行数量
//int Annulu_Two_error_sum=0; //开始入环中有效行的偏差和
//int Annulu_Two_error=0;     //开始入环中有效行的偏差
//int Annulu_Two_error_last=0;//开始入环中有效行的上一次偏差
int Annulu_Radius=1;       //圆环半径选择，0为50，1为60，2为70，3为80 (默认为1)
int Annulu_Two_LeftTurn[4]  = { 50, 35, 35, 30};//40为测试完毕，其余待测
int Annulu_Two_RightTurn[4] = {-50,-30,-35,-30};   //左转pwm范围小，打得急，变化得小;右转pwm范围大，打得缓,变化大（现pid更合适右转）可考虑修改pid也可考虑归一化左右转pwm极限
float ServoKp_Annulu[4]= {1.8,1.8,1.8,1.8};//不同半径环对应不同舵机KP
float ServoKd_Annulu[4]= { 25, 25, 25, 25};//不同半径环对应不同舵机KD

/////////////////////////////////////坡道识别//////////////////////////////////////////
int ramp_flag=0;              //坡道标志
int ramp_flag_real_distance=0;       //坡道行驶实时距离
int ramp_flag_limit_distance=20000;  //坡道行驶限制距离
int ramp_discern_top=12;      //坡道识别上边界
int ramp_discern_bottom=1;    //坡道识别下边界
/////////////////////////////////////FLASH/////////////////////////////////////////////

/////////////////////////////////////蜂鸣器警示////////////////////////////////////////
int Buzzing_count=0;        //蜂鸣器计时器
int Buzzing_count_limit=2000; //蜂鸣器计时器延时时间设定
int Buzzing_start_flag=0;   //开启蜂鸣器标志
/////////////////////////////////////其他变量//////////////////////////////////////////
int runtime=0;                             //计时部分总运行时间
int runtimemax=0;
int i=0;
int j=0;
/////////////////////////////////////停车//////////////////////////////////////////
long int car_stop_real_distance=0;          //正常行驶实际距离
long int car_stop_limit_distance=23000*12;  //正常行驶限制距离12.5
int car_stop_reset=0;                       //行驶距离清零
////////////////////////////////////////TFmini/////////////////////////////////////////
uint8  TFminidate[9]={0};      //TFminidate[4]为距离
uint8  TFminidate_true[9]={0};
uint8  data_TFmini=0;
int    TFminiflag=0;
int    count_UART0=0;
int    distance=0;
int tfmini_return=0;

int left_add = 0;
int middle_add = 0;
int right_add = 0;
int comparative_result0=0;

void Initialization()
{
    /********************----------------常量初始化（计算一次）----------------********************/
    for(int i=0;i<=120;i++)
    {
        width_stand_out[i]=(int)(2.8895*i + 27.174+2);    //行标准宽度 拟合函数+补偿偏移                     //*//
        if(width_stand_out[i]>188)width_stand_out[i]=188;
    }
    width_stand_out[front_text2]=92;
    /********************---------------uart初始化-----------------********************/
    //串口1默认连接到摄像头配置串口
    /********************----------------FTM初始化-----------------********************/
    //编码器初始化
    gpt12_init(GPT12_T5, GPT12_T5INB_P10_3, GPT12_T5EUDB_P10_1);
    /********************----------------PWM初始化-----------------********************/
    //舵机居中
    gtm_pwm_init(S_MOTOR_PIN, 50, SERVOMIDDLE);  //中750 右520（-145）左980（+145）
    //电机初始化
    gtm_pwm_init(ATOM0_CH3_P21_5, 13000, 0);     //正（上）
    gtm_pwm_init(ATOM0_CH0_P21_2, 13000, 0);     //反（下）
    /********************----------------ADC 初始化-----------------********************/
    adc_init(ADC_0, ADC0_CH0_A0);                //初始化ADC0 通道0 使用A0引脚
    adc_init(ADC_0, ADC0_CH1_A1);
    adc_init(ADC_0, ADC0_CH2_A2);
    adc_init(ADC_0, ADC0_CH3_A3);

    adc_init(ADC_0, ADC0_CH6_A6);
    /********************---------------gpio初始化-----------------********************/
    //LED灯
    gpio_init(P20_8, GPO, 0, PUSHPULL);          //设置P20_8为输出 默认输出低电平  PUSHPULL：推挽输出
    gpio_init(P20_9, GPO, 0, PUSHPULL);
    //蜂鸣器
    gpio_init(P21_6, GPO, 0, NO_PULL);           //有源蜂鸣器
    //gtm_pwm_init(ATOM1_CH7_P02_7, 4000, 9000); //无源蜂鸣器
    //五个按键初始化GPIO为输入
    //按键需要对引脚进行初始化，并且把引脚置成输入、高电平（高低电平实际无所谓，讲究则根据硬件上下拉情况）
    gpio_init(KeyTip,  GPI, 1, PULLUP);
    gpio_init(KeyUp,   GPI, 1, PULLUP);
    gpio_init(KeyDown, GPI, 1, PULLUP);
    gpio_init(KeyLeft, GPI, 1, PULLUP);
    gpio_init(KeyRight,GPI, 1, PULLUP);
    /********************-----------其他引脚或函数初始化-----------********************/
    //初始化摄像头
    mt9v03x_init();
    /********************----------------中断初始化----------------********************/
    //使用CCU6_0模块的通道1 产生一个3ms的周期中断
    pit_interrupt_ms(CCU6_0, PIT_CH1, 2);   //控制程序  //每次改变中断周期时间则注意修改设定速度（编码器值发生改变）
    /********************-------现场环境条件初始化信息采集---------********************/
//    Signal_Init();  //电感初始化
}
/***************************************************************************************
函 数 名 :int core0_main(void)
功    能 :主函数
传    入 :无
返 回 值 :无
整定参数 :无
***************************************************************************************/
int core0_main(void)
{
    get_clk();//获取时钟频率  务必保留
    //用户在此处调用各种初始化函数等
    Initialization(); //各类初始化

    //等待所有核心初始化完毕
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    enableInterrupts();
    while (TRUE)
    {
        //TFmini采集距离(大于25cm为高电平，否则为低电平)
        tfmini_return=adc_mean_filter(ADC_0, ADC0_CH6_A6, ADC_12BIT, 1);
        //图像处理
        Picture_Processing();
        //电感处理
        Inductance_GetDeal();
        //蜂鸣器警示
        Buzzing_Remind();
    }
}
/***************************************************************************************
函 数 名 :void Picture_Processing(void)
功     能  :图像处理(图像信息分析处理)
传     入  :无
返 回  值 :无
整定参数 :无
说明：（目前最大2.16ms 时间变化原因：搜索到即终止搜索）
    //图像传输(观察图像)
//        if(mt9v03x_finish_flag)
//        {
//          seekfree_sendimg_03x(UART_0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//          mt9v03x_finish_flag = 0;//在图像使用完毕后  务必清除标志位，否则不会开始采集下一幅图像
//          //注意：一定要在图像使用完毕后在清除此标志位
//        }
    //图像串口输出(用于观察图像有效上下边界)(在线调试)
//        if(mt9v03x_finish_flag)
//        {
//          for(int j=0;j<MT9V03X_W;j++)mt9v03x_image_out[j]=mt9v03x_image[front_text2][j];
//          mt9v03x_finish_flag = 0;//在图像使用完毕后  务必清除标志位，否则不会开始采集下一幅图像
//          //注意：一定要在图像使用完毕后在清除此标志位
//        }
***************************************************************************************/
void Picture_Processing(void)
{
    if(mt9v03x_finish_flag)
    {
//       systick_start(STM1);//使用STM1 进行计时

       //seekfree_sendimg_03x(UART_0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);

        //分析行左中右
        //comparative_result0=Image_Ternary_Segmentation(front_text0);
        //预存储行灰度值
//        line_white_dot_sum[front_text1]=0;line_black_dot_sum[front_text1]=0;
        line_white_dot_sum[front_text2-4]=0;line_black_dot_sum[front_text2-4]=0;
        line_white_dot_sum[front_text2  ]=0;line_black_dot_sum[front_text2  ]=0;
        line_white_dot_sum[front_text2+2]=0;line_black_dot_sum[front_text2+2]=0;
        for(int j=0;j<188;j++)
        {
            //用于观察斑马线行或者搜线行灰度值情况（运行后可关闭）
            practical_image[ZebraCrossingLine1][j]=mt9v03x_image[ZebraCrossingLine1][j];
            //用于判断十字(记录)
//            if(mt9v03x_image[front_text1][j]>=hop)line_white_dot_sum[front_text1]++;
//            else line_black_dot_sum[front_text1]++;
            if(mt9v03x_image[front_text2-4][j]>=hop)line_white_dot_sum[front_text2-4]++;
            else line_black_dot_sum[front_text2-4]++;
            if(mt9v03x_image[front_text2  ][j]>=hop)line_white_dot_sum[front_text2  ]++;
            else line_black_dot_sum[front_text2  ]++;
            if(mt9v03x_image[front_text2+2][j]>=hop)line_white_dot_sum[front_text2+2]++;
            else line_black_dot_sum[front_text2+2]++;
        }
        //分析各行信息并存储）（需要测时）
        width_text(0);width_text(1);width_text(2);width_text(3);width_text(4);
        width_text(5);width_text(6);width_text(7);width_text(8);width_text(9);
        width_text(10);width_text(11);width_text(12);width_text(13);width_text(14);
        width_text(15);width_text(16);width_text(17);width_text(18);width_text(19);
        width_text(20);width_text(21);width_text(22);width_text(23);width_text(24);
        width_text(25);width_text(26);width_text(27);width_text(28);width_text(29);
        width_text(30);width_text(31);width_text(32);width_text(33);width_text(34);
        width_text(35);width_text(36);width_text(37);width_text(38);width_text(39);
        width_text(40);width_text(41);width_text(42);width_text(43);width_text(44);
        width_text(45);width_text(46);width_text(47);width_text(48);width_text(49);
        width_text(50);
        //补线特用（需要观察其情况）*********************************************************************************
        width_text(front_text4);width_text(front_text5);width_text(front_text6);
        //斑马线跳变点情况
        Zebra_Crossing_Search_One(ZebraCrossingLine1-6);
        Zebra_Crossing_Search_One(ZebraCrossingLine1-5);
        Zebra_Crossing_Search_One(ZebraCrossingLine1-4);
        Zebra_Crossing_Search_One(ZebraCrossingLine1-3);
        Zebra_Crossing_Search_One(ZebraCrossingLine1-2);
        Zebra_Crossing_Search_One(ZebraCrossingLine1-1);
        Zebra_Crossing_Search_One(ZebraCrossingLine1  );
        Zebra_Crossing_Search_One(ZebraCrossingLine1+1);
        //获取各行斜率并统计不同种类斜率个数（1——最大有效行）
        Border_Slope_Calculate();
        //开启下一次采集图像
        mt9v03x_finish_flag = 0;


//        runtime = systick_getval_us(STM1);         //读取STM1计时时间(开始计时到现在的时间)
//        if(runtime>runtimemax)runtimemax=runtime;  //记录最大时间
    }
}
/***************************************************************************************
函 数 名 :void Zebra_Crossing_Search(int line_function)
功    能 :搜索斑马线
传    入 :所选取行的序号
返 回 值 :无
整定参数 :无
说明：左出库即左入库，右出库即右入库；左入库则由左往右搜索，右入库亦然；
***************************************************************************************/
void Zebra_Crossing_Search_One(int line_function)
{
    int i=0;
    //计数清零
    ZebraBtoW_count[line_function]=0;
    ZebraWtoB_count[line_function]=0;
    for(i=0;i<15;i++)
    {
        ZebraWtoB_column[line_function][i]=0;
        ZebraBtoW_column[line_function][i]=0;
    }
    if(OutgarageDirection == 0) //左出库
    {
        for(i=2;i<=185;i++)
        {
            if( mt9v03x_image[line_function][i+2]<hop &&
                mt9v03x_image[line_function][i+1]<hop &&
                mt9v03x_image[line_function][i  ]>hop &&//限制当前判断点
                mt9v03x_image[line_function][i-1]>hop &&
                mt9v03x_image[line_function][i-2]>hop  )
                {ZebraWtoB_column[line_function][ZebraWtoB_count[line_function]]=i;ZebraWtoB_count[line_function]+=1;}//搜索白变黑
            if( mt9v03x_image[line_function][i+2]>hop &&
                mt9v03x_image[line_function][i+1]>hop &&
                mt9v03x_image[line_function][i  ]<hop &&//限制当前判断点
                mt9v03x_image[line_function][i-1]<hop &&
                mt9v03x_image[line_function][i-2]<hop  )
                {ZebraBtoW_column[line_function][ZebraBtoW_count[line_function]]=i;ZebraBtoW_count[line_function]+=1; }//搜索黑变白
        }
    }
    if(OutgarageDirection == 1) //右出库
    {
        for(i=185;i>=2;i--)//右入库，从右往左搜索
        {
           if( mt9v03x_image[line_function][i-2]<hop &&
               mt9v03x_image[line_function][i-1]<hop &&
               mt9v03x_image[line_function][i  ]>hop &&//限制当前判断点
               mt9v03x_image[line_function][i+1]>hop &&
               mt9v03x_image[line_function][i+2]>hop )
               {ZebraWtoB_column[line_function][ZebraWtoB_count[line_function]]=i;ZebraWtoB_count[line_function]+=1;}//搜索白变黑
           if( mt9v03x_image[line_function][i-2]>hop &&
               mt9v03x_image[line_function][i-1]>hop &&
               mt9v03x_image[line_function][i  ]<hop &&//限制当前判断点
               mt9v03x_image[line_function][i+1]<hop &&
               mt9v03x_image[line_function][i+2]<hop )
               {ZebraBtoW_column[line_function][ZebraBtoW_count[line_function]]=i;ZebraBtoW_count[line_function]+=1; }//搜索黑变白
        }
    }

}
/***************************************************************************************
函 数 名 :int width_text(int line_function)
功    能 :左右边界（第几行、是否有、边界阈值是多少）、宽度、中线查看
传    入 :所选取行的序号
返 回 值 :无
整定参数 :无
***************************************************************************************/
void width_text(int line_function)
{
    int i=0;
//重置左右边界标志位
    left_flag[line_function]=0;
    right_flag[line_function]=0;
//搜索左边界
    leftline[line_function]=4;
    for(i=line_search_start[line_function];i>=4;i--)
    {
       if( mt9v03x_image[line_function][i-3]<hop &&
           mt9v03x_image[line_function][i-2]<hop &&
           mt9v03x_image[line_function][i+2]>hop &&
           mt9v03x_image[line_function][i+4]>hop &&
           mt9v03x_image[line_function][i+7]>hop )
       {
           leftline[line_function]=i;
           left_flag[line_function]=1;
           break;
       }
    }
//搜索右边界
    rightline[line_function]=183;
    for(i=line_search_start[line_function];i<=183;i++)
    {
        if( mt9v03x_image[line_function][i+3]<hop &&
            mt9v03x_image[line_function][i+2]<hop &&
            mt9v03x_image[line_function][i-2]>hop &&
            mt9v03x_image[line_function][i-4]>hop &&
            mt9v03x_image[line_function][i-7]>hop )
       {
           rightline[line_function]=i;
           right_flag[line_function]=1;
           break;
       }
    }
    //计算赛道宽度
    width_out[line_function]=rightline[line_function]-leftline[line_function];
    //求取赛道中线
    midline[line_function]=(rightline[line_function]+leftline[line_function])/2;
//获得最后修正的中线结果
    //左边界消失（借助有边界的一边进行补线）
    if(left_flag[line_function]==0 && right_flag[line_function]==1)
    { result_midline[line_function]=rightline[line_function]-width_stand_out[line_function]/2; }
    //右边界消失（借助有边界的一边进行补线）
    else if(left_flag[line_function]==1 && right_flag[line_function]==0)
    { result_midline[line_function]=leftline[line_function]+width_stand_out[line_function]/2; }
    //左右边界都在正常搜线结果作为最终中线
    else result_midline[line_function]=midline[line_function];
//获取偏差
    midline_error[line_function]=midline_stand-result_midline[line_function];
//根据上一次的中点情况更新 左右搜索的起点(如此处理出十字会有晃动)(若十字搜先行其实列居中或多行搜索中线)
    if(left_flag[line_function]==0 && right_flag[line_function]==0)
    {   }
    else
    {
        line_search_start[line_function]=result_midline[line_function];
        line_search_start[line_function]=line_search_start[line_function]>left_limit?(line_search_start[line_function]<right_limit?line_search_start[line_function]:right_limit):left_limit;//限幅
    }
//入环时将搜索起点定为边界（左入环从左往右，右入环从右往左）
    if(Annulu_Flag==2 )
    {
        if(Annulu_Direction==1)line_search_start[line_function]=20;
        if(Annulu_Direction==2)line_search_start[line_function]=170;
    }
}
/***************************************************************************************
函 数 名 :void Border_Slope_Calculate(void)
功     能 :求解各行左右边界斜率
传     入 :无
返 回 值 :无
整定参数 :无
说明：列数最多为180，则求解斜率范围应在【-180——180】
***************************************************************************************/
void Border_Slope_Calculate(void)
{
    //统计清零
    for(int i=0;i<=4;i++)
    {
        for(int j=0;j<=4;j++)
        {
            LeftSlopeSortCount[i][j]=0;
            RrightSlopeSortCount[i][j]=0;
        }
    }
    for(int i=1;i<=practical_maxline-1;i++)
    {
    //左边界斜率计算与处理
        //左边界斜率计算
        if(left_flag[i-1]==1  && left_flag[i]==1  && left_flag[i+1]==1  ) LeftLineSlope[i]=(leftline[i+1]-leftline[i-1]);
        else LeftLineSlope[i]=-200;
        //根据计算所得斜率进行统计
        if(i<=Line_Demarcation[0])
        {
            if     ( LeftLineSlope[i]>=LeftSlopeLimit[0])LeftSlopeSortCount[0][0]++;//右斜
            else if( LeftLineSlope[i]>=LeftSlopeLimit[1])LeftSlopeSortCount[0][1]++;//正常
            else if( LeftLineSlope[i]>=LeftSlopeLimit[2])LeftSlopeSortCount[0][2]++;//微左斜（环岛）
            else if( LeftLineSlope[i]>=LeftSlopeLimit[3])LeftSlopeSortCount[0][3]++;//严重左斜
            else if( LeftLineSlope[i]==-200)             LeftSlopeSortCount[0][4]++;//未满足求斜率要求
        }
        else if(i<=Line_Demarcation[1])
        {
            if     ( LeftLineSlope[i]>=LeftSlopeLimit[0])LeftSlopeSortCount[1][0]++;//右斜
            else if( LeftLineSlope[i]>=LeftSlopeLimit[1])LeftSlopeSortCount[1][1]++;//正常
            else if( LeftLineSlope[i]>=LeftSlopeLimit[2])LeftSlopeSortCount[1][2]++;//微左斜（环岛）
            else if( LeftLineSlope[i]>=LeftSlopeLimit[3])LeftSlopeSortCount[1][3]++;//严重左斜
            else if( LeftLineSlope[i]==-200)             LeftSlopeSortCount[1][4]++;//未满足求斜率要求
        }
        else if(i<=Line_Demarcation[2])
        {
            if     ( LeftLineSlope[i]>=LeftSlopeLimit[0])LeftSlopeSortCount[2][0]++;//右斜
            else if( LeftLineSlope[i]>=LeftSlopeLimit[1])LeftSlopeSortCount[2][1]++;//正常
            else if( LeftLineSlope[i]>=LeftSlopeLimit[2])LeftSlopeSortCount[2][2]++;//微左斜（环岛）
            else if( LeftLineSlope[i]>=LeftSlopeLimit[3])LeftSlopeSortCount[2][3]++;//严重左斜
            else if( LeftLineSlope[i]==-200)             LeftSlopeSortCount[2][4]++;//未满足求斜率要求
        }
        else
        {
            if     ( LeftLineSlope[i]>=LeftSlopeLimit[0])LeftSlopeSortCount[3][0]++;//右斜
            else if( LeftLineSlope[i]>=LeftSlopeLimit[1])LeftSlopeSortCount[3][1]++;//正常
            else if( LeftLineSlope[i]>=LeftSlopeLimit[2])LeftSlopeSortCount[3][2]++;//微左斜（环岛）
            else if( LeftLineSlope[i]>=LeftSlopeLimit[3])LeftSlopeSortCount[3][3]++;//严重左斜
            else if( LeftLineSlope[i]==-200)             LeftSlopeSortCount[3][4]++;//未满足求斜率要求
        }
    //右边界斜率计算与处理
        //右边界斜率计算
        if(right_flag[i-1]==1 && right_flag[i]==1 && right_flag[i+1]==1 )RightLineSlope[i]=(rightline[i+1]-rightline[i-1]);
        else RightLineSlope[i]=200;
        //根据计算所得斜率进行统计
        if(i<=Line_Demarcation[0])
        {
            if     ( RightLineSlope[i]<=RrightSlopeLimit[0])RrightSlopeSortCount[0][0]++;//左斜
            else if( RightLineSlope[i]<=RrightSlopeLimit[1])RrightSlopeSortCount[0][1]++;//正常
            else if( RightLineSlope[i]<=RrightSlopeLimit[2])RrightSlopeSortCount[0][2]++;//微右斜（环岛）
            else if( RightLineSlope[i]<=RrightSlopeLimit[3])RrightSlopeSortCount[0][3]++;//严重右斜
            else if( RightLineSlope[i]==200)                RrightSlopeSortCount[0][4]++;//未满足求斜率要求
        }
        else if(i<=Line_Demarcation[1])
        {
            if     ( RightLineSlope[i]<=RrightSlopeLimit[0])RrightSlopeSortCount[1][0]++;//左斜
            else if( RightLineSlope[i]<=RrightSlopeLimit[1])RrightSlopeSortCount[1][1]++;//正常
            else if( RightLineSlope[i]<=RrightSlopeLimit[2])RrightSlopeSortCount[1][2]++;//微右斜（环岛）
            else if( RightLineSlope[i]<=RrightSlopeLimit[3])RrightSlopeSortCount[1][3]++;//严重右斜
            else if( RightLineSlope[i]==200)                RrightSlopeSortCount[1][4]++;//未满足求斜率要求
        }
        else if(i<=Line_Demarcation[2])
        {
            if     ( RightLineSlope[i]<=RrightSlopeLimit[0])RrightSlopeSortCount[2][0]++;//左斜
            else if( RightLineSlope[i]<=RrightSlopeLimit[1])RrightSlopeSortCount[2][1]++;//正常
            else if( RightLineSlope[i]<=RrightSlopeLimit[2])RrightSlopeSortCount[2][2]++;//微右斜（环岛）
            else if( RightLineSlope[i]<=RrightSlopeLimit[3])RrightSlopeSortCount[2][3]++;//严重右斜
            else if( RightLineSlope[i]==200)                RrightSlopeSortCount[2][4]++;//未满足求斜率要求
        }
        else
        {
            if     ( RightLineSlope[i]<=RrightSlopeLimit[0])RrightSlopeSortCount[3][0]++;//左斜
            else if( RightLineSlope[i]<=RrightSlopeLimit[1])RrightSlopeSortCount[3][1]++;//正常
            else if( RightLineSlope[i]<=RrightSlopeLimit[2])RrightSlopeSortCount[3][2]++;//微右斜（环岛）
            else if( RightLineSlope[i]<=RrightSlopeLimit[3])RrightSlopeSortCount[3][3]++;//严重右斜
            else if( RightLineSlope[i]==200)                RrightSlopeSortCount[3][4]++;//未满足求斜率要求
        }
    }
    //各行合计
    for(int i=0;i<=4;i++)
    {
        LeftSlopeSortCount[4][i]=LeftSlopeSortCount[3][i]+LeftSlopeSortCount[2][i]+LeftSlopeSortCount[1][i]+LeftSlopeSortCount[0][i];
        RrightSlopeSortCount[4][i]=RrightSlopeSortCount[3][i]+RrightSlopeSortCount[2][i]+RrightSlopeSortCount[1][i]+RrightSlopeSortCount[0][i];
    }
}
/***************************************************************************************
函 数 名 :void Line_Jump_Search(void)
功    能 :行边界跳变检测
传    入 :无
返 回 值 :无
整定参数 :无
***************************************************************************************/
void Line_Jump_Search(void)
{
    int i=0;
    for(i=practical_maxline-2;i>=2;i--)
    {
        if(left_flag[i]==0)continue;
//        if(left_flag[i]==1)
    }
}
/***************************************************************************************
函 数 名 :void LineHop_Average(int line_function)
功    能 :存储行灰度均值
传    入 :所选取行的序号
返 回 值 :无
整定参数 :无
改进：   后续增加传入数量，减少反复进入for循环，做到一次for循环同时处理多行
***************************************************************************************/
void LineHop_Average(int line_function)
{
    int i=0;
    image_average[line_function]=0;
    for(i=4;i<=184;i++)image_average[line_function]+=mt9v03x_image[line_function][i];
    image_average[line_function]=image_average[line_function]/181;
}
/***************************************************************************************
函 数 名 :int Image_Ternary_Segmentation(int line_function)
功     能  :对所选列分成左中右三部分，对比三部分的加和值大小
传     入  :所选取列的序号
返 回  值 :返回 1 为中间大于左右两边（十字），返回 2 为两边大于中间（三叉），返回0为其他
整定参数 :无
***************************************************************************************/
int Image_Ternary_Segmentation(int line_function)
{
    int i=0;
    int comparative_result=0;
    left_add=0;
    middle_add=0;
    right_add=0;
    for(i=4  ;i<=63 ;i++)left_add  +=mt9v03x_image[line_function][i];
    for(i=64 ;i<=123;i++)middle_add+=mt9v03x_image[line_function][i];
    for(i=123;i<=183;i++)right_add +=mt9v03x_image[line_function][i];
    left_add=left_add/60;
    middle_add=middle_add/60;
    right_add=right_add/60;
    if(middle_add>(left_add*1.1) && middle_add>(right_add*1.1) )comparative_result=1;
    else if( (middle_add*1.1)<left_add && (middle_add*1.1)<right_add)comparative_result=2;
    else comparative_result=0;
    return comparative_result;
}
/***************************************************************************************
函 数 名 :void Annulus_Judge(void)
功      能 :判断圆环
传      入 :无
返 回 值 :无
整定参数 :无
***************************************************************************************/
void Annulus_Judge(void)
{
///////////////////////////////////////////////////////////
//判断
///////////////////////////////////////////////////////////
    //判断准备入环（不在环内，应当再加其他正常运行条件，边界斜率符合相应条件）
         //左入环
         if(  Annulu_Flag==0 &&
              LeftSlopeSortCount[1][2]>=SlopePASSMargin[0][0] && RrightSlopeSortCount[1][1]>=5 && //10——20左微斜右正常
              LeftSlopeSortCount[2][4]>=SlopePASSMargin[1][1] && RrightSlopeSortCount[2][4]<=7  ) //21--35左空白右有边界
         {
              Buzzing_start_flag=1;//蜂鸣器响
              Annulu_Direction=1;  //确定为左圆环
              Annulu_Flag=1;       //确定准备入环
         }
         //右入环
         if(  Annulu_Flag==0 &&
              RrightSlopeSortCount[1][2]>=SlopePASSMargin[0][0] && LeftSlopeSortCount[1][1]>=5 && //10——20右微斜右正常
              RrightSlopeSortCount[2][4]>=SlopePASSMargin[1][1] && LeftSlopeSortCount[2][4]<=7  ) //21--35右空白左有边界
         {
             Buzzing_start_flag=1;//蜂鸣器响
             Annulu_Direction=2;  //确定为右圆环
             Annulu_Flag=1;       //确定准备入环
         }
    //判断开始入环（处在准备入环阶段，边界斜率符合相应条件）
         if(Annulu_Flag==1 )
         {
             if(Annulu_Direction==1)//左圆环
             {
                 if(Annulu_Flag_supplement==0)//准备入环（第一阶段）（判断行出现无斜率——判断行有斜率）
                 {
                     if(LeftLineSlope[13]!=-200 && LeftLineSlope[14]!=-200 && LeftLineSlope[15]!=-200)
                     {Buzzing_start_flag=1;Annulu_Flag_supplement=1;}
                 }
                 if(Annulu_Flag_supplement==1)//准备入环（第二阶段）（判断行出现有斜率——判断行重新无斜率）
                 {
                     if(LeftLineSlope[14]==-200 && LeftLineSlope[15]==-200)Annulu_Flag_supplement=2;
                 }
                 //最终判断
                 if(Annulu_Flag_supplement==2)
                 {Buzzing_start_flag=1;Annulu_Flag=2;Annulu_Flag_supplement=0;}
             }
             if(Annulu_Direction==2)//右圆环
             {
                 if(Annulu_Flag_supplement==0)//准备入环（第一阶段）（判断行出现无斜率——判断行有斜率）
                 {
                     if(RightLineSlope[13]!=200 && RightLineSlope[14]!=200 && RightLineSlope[15]!=200)
                     {Buzzing_start_flag=1;Annulu_Flag_supplement=1;}
                 }
                 if(Annulu_Flag_supplement==1)//准备入环（第二阶段）（判断行出现有斜率——判断行重新无斜率）
                 {
                     if(RightLineSlope[14]==200 && RightLineSlope[15]==200)Annulu_Flag_supplement=2;
                 }
                 //最终判断
                 if(Annulu_Flag_supplement==2)
                 {Buzzing_start_flag=1;Annulu_Flag=2;Annulu_Flag_supplement=0;}
             }
         }
   //判断正在环中
         //
         if(Annulu_Flag==2 )
         {
             if(Annulu_Direction==1) //左环
             {
                 if(Annulu_Flag_supplement==0)//开始入环（第一阶段）（判断行从搜索到边界——搜索不到边界）
                 {
                     if(right_flag[front_text2]==0 && right_flag[front_text2+1]==0 && right_flag[front_text2+2]==0)
                     {Buzzing_start_flag=1;Annulu_Flag_supplement=1;}
                 }
                 if(Annulu_Flag_supplement==1)//开始入环（第二阶段）（判断行从搜索不到边界——重新搜索到边界）
                 {
                     if(right_flag[front_text2]==1 && right_flag[front_text2+1]==1 && right_flag[front_text2+2]==1)Annulu_Flag_supplement=2;
                 }
                 //最终判断
                 if(AnnuluAngle<AnnuluAngleLimit_Left[0] || Annulu_Flag_supplement==2)
                 {Buzzing_start_flag=1;Annulu_Flag=3;Annulu_Flag_supplement=0;}
             }
             if(Annulu_Direction==2) //右环
             {
                 if(Annulu_Flag_supplement==0)//开始入环（第一阶段）（判断行从搜索到边界——搜索不到边界）
                 {
                     if(left_flag[front_text2]==0 && left_flag[front_text2+1]==0 && left_flag[front_text2+2]==0)
                     {Buzzing_start_flag=1;Annulu_Flag_supplement=1;}
                 }
                 if(Annulu_Flag_supplement==1)//开始入环（第二阶段））（判断行从搜索不到边界——重新搜索到边界）
                 {
                     if(left_flag[front_text2]==1 && left_flag[front_text2+1]==1 && left_flag[front_text2+2]==1)Annulu_Flag_supplement=2;
                 }
                 //最终判断
                 if(AnnuluAngle>AnnuluAngleLimit_Right[0] || Annulu_Flag_supplement==2)
                 {Buzzing_start_flag=1;Annulu_Flag=3;Annulu_Flag_supplement=0;}
             }
         }
   //判断准备出环
         if(Annulu_Flag==3)
         {
             if(Annulu_Direction==1)if(AnnuluAngle<AnnuluAngleLimit_Left[1] ){Buzzing_start_flag=1;Annulu_Flag=4;}
             if(Annulu_Direction==2)if(AnnuluAngle>AnnuluAngleLimit_Right[1] ){Buzzing_start_flag=1;Annulu_Flag=4;}
         }
   //判断正在出环（第一阶段）
         if(Annulu_Flag==4)
         {
             if(Annulu_Direction==1)if(right_flag[front_text2-4]==0 && right_flag[front_text2-3]==0){Buzzing_start_flag=1;Annulu_Flag=5;}
             if(Annulu_Direction==2)if(left_flag[front_text2-4]==0 && left_flag[front_text2-3]==0){Buzzing_start_flag=1;Annulu_Flag=5;}
         }
   //判断正在出环（第二阶段）
         if(Annulu_Flag==5)
         {
             if(Annulu_Direction==1)
             {
                 if(right_flag[front_text2-4]==1 && right_flag[front_text2-3]==1 &&
                    right_flag[front_text2-2]==1 && right_flag[front_text2-1]==1 &&
                    right_flag[front_text2  ]==1 && right_flag[front_text2+1]==1)
                 {Buzzing_start_flag=1;Annulu_Flag=6;}
             }
             if(Annulu_Direction==2)
             {
                 if(left_flag[front_text2-4]==1 && left_flag[front_text2-3]==1 &&
                    left_flag[front_text2-2]==1 && left_flag[front_text2-1]==1 &&
                    left_flag[front_text2  ]==1 && left_flag[front_text2+1]==1)
                 {Buzzing_start_flag=1;Annulu_Flag=6;}
             }
         }
   //判断结束出环
         if(Annulu_Flag==6)
         {
             if(LeftSlopeSortCount[1][4]<=SlopePASSMargin[1][0] && RrightSlopeSortCount[1][4]<=SlopePASSMargin[1][0])
             {Buzzing_start_flag=1;Annulu_Flag=0;Annulu_Direction=0;}
         }
///////////////////////////////////////////////////////////
//处理
///////////////////////////////////////////////////////////
    //准备入环
        if(Annulu_Flag==1)
        {
            //清零
            Annulu_one_count=0;
            Annulu_one_error_sum=0;
            Annulu_one_error=0;
            if(Annulu_Direction==1)//左圆环
            {
                if(right_flag[front_text2-2]==1)
                { Annulu_one_error_sum+=(midline_stand-(rightline[front_text2-2]-width_stand_out[front_text2-2]/2));Annulu_one_count++; }
                if(right_flag[front_text2-1]==1)
                { Annulu_one_error_sum+=(midline_stand-(rightline[front_text2-1]-width_stand_out[front_text2-1]/2));Annulu_one_count++; }
                if(right_flag[front_text2+1]==1)
                { Annulu_one_error_sum+=(midline_stand-(rightline[front_text2+1]-width_stand_out[front_text2+1]/2));Annulu_one_count++; }
                if(right_flag[front_text2+2]==1)
                { Annulu_one_error_sum+=(midline_stand-(rightline[front_text2+2]-width_stand_out[front_text2+2]/2));Annulu_one_count++; }
            }
            if(Annulu_Direction==2)//右圆环
            {
                if(left_flag[front_text2-2]==1)
                { Annulu_one_error_sum+=(midline_stand-(leftline[front_text2-2]+width_stand_out[front_text2-2]/2));Annulu_one_count++; }
                if(left_flag[front_text2-1]==1)
                { Annulu_one_error_sum+=(midline_stand-(leftline[front_text2-1]+width_stand_out[front_text2-1]/2));Annulu_one_count++; }
                if(left_flag[front_text2+1]==1)
                { Annulu_one_error_sum+=(midline_stand-(leftline[front_text2+1]+width_stand_out[front_text2+1]/2));Annulu_one_count++; }
                if(left_flag[front_text2+2]==1)
                { Annulu_one_error_sum+=(midline_stand-(leftline[front_text2+2]+width_stand_out[front_text2+2]/2));Annulu_one_count++; }
            }
            Annulu_one_error=Annulu_one_error_sum/Annulu_one_count;
            ServoFuzzyErr=(float)(Annulu_one_error);
        }
    //开始入环//
        if(Annulu_Flag==2)
        {
//            //蜂鸣器响
//            Buzzing_start_flag=1;
//            //清零
//            Annulu_Two_count=0;
//            Annulu_Two_error_sum=0;
//            Annulu_Two_error=0;
//            Annulu_Two_error_last=0;
//            if(Annulu_Direction==1)//左圆环
//            {
//                if(left_flag[front_text2-2]==1)
//                { Annulu_Two_error_sum+=(midline_stand-(leftline[front_text2-2]+width_stand_out[front_text2-2]/2));Annulu_Two_count++; }
//                if(left_flag[front_text2-1]==1)
//                { Annulu_Two_error_sum+=(midline_stand-(leftline[front_text2-1]+width_stand_out[front_text2-1]/2));Annulu_Two_count++; }
//                if(left_flag[front_text2+1]==1)
//                { Annulu_Two_error_sum+=(midline_stand-(leftline[front_text2+1]+width_stand_out[front_text2+1]/2));Annulu_Two_count++; }
//                if(left_flag[front_text2+2]==1)
//                { Annulu_Two_error_sum+=(midline_stand-(leftline[front_text2+2]+width_stand_out[front_text2+2]/2));Annulu_Two_count++; }
//            }
//            if(Annulu_Direction==2)//右圆环
//            {
//                if(right_flag[front_text2-2]==1)
//                { Annulu_Two_error_sum+=(midline_stand-(rightline[front_text2-2]-width_stand_out[front_text2-2]/2));Annulu_Two_count++; }
//                if(right_flag[front_text2-1]==1)
//                { Annulu_Two_error_sum+=(midline_stand-(rightline[front_text2-1]-width_stand_out[front_text2-1]/2));Annulu_Two_count++; }
//                if(right_flag[front_text2+1]==1)
//                { Annulu_Two_error_sum+=(midline_stand-(rightline[front_text2+1]-width_stand_out[front_text2+1]/2));Annulu_Two_count++; }
//                if(right_flag[front_text2+2]==1)
//                { Annulu_Two_error_sum+=(midline_stand-(rightline[front_text2+2]-width_stand_out[front_text2+2]/2));Annulu_Two_count++; }
//            }
//            Annulu_Two_error_last=Annulu_Two_error;
//            if(Annulu_Two_count==0) ServoFuzzyErr=(float)(Annulu_Two_error_last);
//            else
//            {
//                Annulu_Two_error=Annulu_Two_error_sum/Annulu_Two_count;
//                ServoFuzzyErr=(float)(Annulu_Two_error);
//            }
            if(Annulu_Direction==1)ServoFuzzyErr=(float)(Annulu_Two_LeftTurn[Annulu_Radius]);
            if(Annulu_Direction==2)ServoFuzzyErr=(float)(Annulu_Two_RightTurn[Annulu_Radius]);
        }
    //正在环中及准备出环（自然搜线  但更改pid）PID_Option=4;width_stand_out[front_text2]=92;}else{width_stand_out[front_text2]=92;
        if(Annulu_Flag==3 || Annulu_Flag==4){}
    //正在出环（第一阶段）
        if(Annulu_Flag==5)
        {
            if(Annulu_Direction==1)ServoFuzzyErr=(float)(Annulu_Two_LeftTurn[Annulu_Radius]);
            if(Annulu_Direction==2)ServoFuzzyErr=(float)(Annulu_Two_RightTurn[Annulu_Radius]);
        }
    //正在出环（第二阶段）(自然搜线 不做处理)
        if(Annulu_Flag==6)
        { }
}
/***************************************************************************************
函 数 名 :void Servocontrol()
功    能 :舵机控制
传    入 :无
返 回 值 :无
整定参数 :无
***************************************************************************************/
float LeftlineDerivative_wach=0;
float RightlineDerivative_wach=0;

int uphill_Angle = -10;
int Downhill_Angle = 5;
int EndRamp_Angle =  5;
void Servocontrol()
{
///////////////////////////////////////////////////////////
//出库（出库打固定舵）
///////////////////////////////////////////////////////////
    if     (OutgarageFlag == 0)ServoFuzzyErr = 0;   //等待出库
    else if(OutgarageFlag == 1)                     //正在出库
    {
        if(OutgarageDirection == 0) ServoFuzzyErr = (float)OutgarageServoDuty[0];  //左出库
        if(OutgarageDirection == 1) ServoFuzzyErr = (float)OutgarageServoDuty[1];  //右出库
        //确定pid
        PID_Option=3;
        if(OutgarageAngle>OutgarageAngleLimit || OutgarageAngle<-OutgarageAngleLimit)OutgarageFlag=2;
    }
    else //出库完成后执行其他事项
    {
        //    turn_flag = 0;
//        if()
//        {
//            NormalOperationflag
//        }
        //pid默认设置（摄像头分段pid）
        PID_Option=0;//默认摄像头
        PID_Flag=0;  //默认分段
///////////////////////////////////////////////////////////
//丢线处理（需要进一步处理，判断条件更改 变成数黑色点）
///////////////////////////////////////////////////////////
            //第二行丢线
            if(left_flag[front_text2]==0 && right_flag[front_text2]==0 )//&& image_average[front_text2]<image_average_limit_bottom
            {
                //蜂鸣器响起
                //Buzzing_start_flag=1;
                if(left_flag[front_text3]==0 && right_flag[front_text3]==0 )
                {
                    if(left_flag[front_text4]==0 && right_flag[front_text4]==0 )
                    {
                        if(left_flag[front_text5]==0 && right_flag[front_text5]==0 )
                        {
                            ServoFuzzyErr=(float)(midline_error[front_text6]);     //第六行补线
                        }
                        else ServoFuzzyErr=(float)(midline_error[front_text5]);    //第五行补线
                    }
                    else ServoFuzzyErr=(float)(midline_error[front_text4]);   //第四行补线
                }
                else ServoFuzzyErr=(float)(midline_error[front_text3]);  //第三行补线
            }
            else ServoFuzzyErr=(float)(midline_error[front_text2]);  //第二行搜线
///////////////////////////////////////////////////////////
//十字(搜先行前后两行空白点较大 不在坡道  不入库)
///////////////////////////////////////////////////////////
            //左右边界消失 （切换成电感搜线）（暂未考虑岔路）
            if((line_white_dot_sum[front_text2-4]>160 || line_white_dot_sum[front_text2+2]>160)&&
                ramp_flag==0 && ZebraCrossingFlag==0)// (left_flag[front_text1]==0 && right_flag[front_text1]==0) || (left_flag[front_text2]==0 || right_flag[front_text2]==0) )
            {
                //蜂鸣器响
//                Buzzing_start_flag=1;
                //切换电磁
                ServoFuzzyErr=(float)(InductanceUseRecult);
                //确定pid
                PID_Option=1;
            }
///////////////////////////////////////////////////////////
//圆环
///////////////////////////////////////////////////////////
            Annulus_Judge();
///////////////////////////////////////////////////////////
//坡道识别(距离近 不在坡道上 不入库)
///////////////////////////////////////////////////////////
            if( tfmini_return>4000 && ramp_flag==0 && ZebraCrossingFlag==0)   ramp_flag=1;
            //在坡道的全程
            if ( ramp_flag!=0 )
            {
                //蜂鸣器响
                Buzzing_start_flag=1;
                //陀螺仪使用
                angle_re_use=angle_re;
                //切换电磁
                ServoFuzzyErr=(float)(InductanceUseRecult);
                //确定pid
                PID_Option=2;
                //判断坡道各阶段
                if ( ramp_flag==1 )     {if( angle_re_use<uphill_Angle  )  ramp_flag=2;} // 入坡道  ——正在上坡
                else if ( ramp_flag==2) {if( angle_re_use>Downhill_Angle)  ramp_flag=3;} //正在上坡——正在下坡
                else if ( ramp_flag==3) {if( left_flag[front_text2]==1 || right_flag[front_text2]==1 ||
                                             left_flag[front_text3]==1 || right_flag[front_text3]==1 ||
                                             left_flag[front_text4]==1 || right_flag[front_text4]==1 ) ramp_flag=4;}//正在下坡——下坡后图像中有有效行
                else if ( ramp_flag==4) {if( tfmini_return>4000 )          ramp_flag=5;}//下坡后图像中有有效行——触地
                else if ( ramp_flag==5) {if( tfmini_return<4000 )          ramp_flag=6;}//下坡后图像中有有效行——触地
                else if ( ramp_flag==6) {if( angle_re_use<EndRamp_Angle ){ ramp_flag=0;angle_re_use=0;}}//触地—平地结束坡道
            }
///////////////////////////////////////////////////////////
//识别斑马线(黑白，白黑跳变点多  不在入库中  不在坡道上  )
///////////////////////////////////////////////////////////
            if( ( (ZebraWtoB_count[ZebraCrossingLine1-1]>5 &&  ZebraBtoW_count[ZebraCrossingLine1-1]>5)  ||
                  (ZebraWtoB_count[ZebraCrossingLine1  ]>5 &&  ZebraBtoW_count[ZebraCrossingLine1  ]>5)  ||
                  (ZebraWtoB_count[ZebraCrossingLine1+1]>5 &&  ZebraBtoW_count[ZebraCrossingLine1+1]>5) )&&
                   ZebraCrossingFlag==0 && ramp_flag==0  )
            {
                //蜂鸣器响
//                Buzzing_start_flag=1;
                //有效行清零
                ZebraErrorCount=0;
                ZebraErrorSum=0;
                for(int i=1;i<=6;i++)
                {
                    if(ZebraWtoB_count[ZebraCrossingLine1-i]>5 &&  ZebraBtoW_count[ZebraCrossingLine1-i]>5)
                    {
                        ZebraErrorSum+=(midline_stand-ZebraWtoB_column[ZebraCrossingLine1-i][0]);
                        ZebraErrorCount++;
                    }
                }
                ZebraError=ZebraErrorSum/ZebraErrorCount;
                ZebraCrossingFlag=1;
            }
            //入库控制
             if(ZebraCrossingFlag==1)
            {
                //蜂鸣器响
//                Buzzing_start_flag=1;
                ServoFuzzyErr=(float)(ZebraError);
                if(ZebraDistance_Real>ZebraDistance_Limit )ZebraCrossingFlag=2;
            }
            if(ZebraCrossingFlag==2)
            {
                //蜂鸣器响
//                Buzzing_start_flag=1;
//                //清零
//                ZebraErrorCount=0;//有效行清零
//                ZebraErrorSum=0;  //误差和清零
//                for(int i=0;i<=3;i++)
//                {
//                    if(ZebraWtoB_count[ZebraCrossingLine1-i]>=1 )
//                    {
//                        if(OutgarageDirection == 0) //左出库则左入库
//                        {
//                            ZebraErrorSum+=(midline_stand-(ZebraWtoB_column[ZebraCrossingLine1-i][0]-width_out2/2));
//                        }
//                        if(OutgarageDirection == 1) //右出库则右入库
//                        {
//                            ZebraErrorSum+=(midline_stand-(ZebraWtoB_column[ZebraCrossingLine1-i][0]+width_out2/2));
//                        }
//                        ZebraErrorCount++;
//                    }
//                }
                if(OutgarageDirection == 0)ZebraError=EndLeftturn; //左出库则左入库
                if(OutgarageDirection == 1)ZebraError=EndRightturn; //右出库则右入库
                ServoFuzzyErr=(float)(ZebraError);
                PID_Option=3;
                if(ZebraAngle>ZebraAngleLimit || ZebraAngle<-ZebraAngleLimit)ZebraCrossingFlag=3;
            }
            if(ZebraCrossingFlag==3)
            {
                ServoFuzzyErr = 0;
            }
    }//出库函数结尾
///////////////////////////////////////////////////////////
//pid选取
///////////////////////////////////////////////////////////
    if(PID_Option==0)//摄像头pid
    {
        if(PID_Flag==0)//摄像头分段pid
        {
            if(ServoFuzzyErr<=Sub_PID[2] && ServoFuzzyErr>=Sub_PID[1])
            {
                ServoKp=ServoKp_image1;
                ServoKd=ServoKd_image1;
            }
            if( (ServoFuzzyErr>Sub_PID[2] && ServoFuzzyErr<=Sub_PID[3]) || (ServoFuzzyErr<Sub_PID[1] && ServoFuzzyErr>=Sub_PID[0] ) )
            {
                ServoKp=ServoKp_image2;
                ServoKd=ServoKd_image2;
            }
            if( ServoFuzzyErr>Sub_PID[3] || ServoFuzzyErr<Sub_PID[0] )
            {
                ServoKp=ServoKp_image3;
                ServoKd=ServoKd_image3;
            }
        }
        if(PID_Flag==1)//摄像头单pid
        {
            ServoKp=ServoKp_all;
            ServoKd=ServoKd_all;
        }
    }
    else  if(PID_Option==1) //电磁正常pid
    {
        ServoKp=ServoKp_induc[1];
        ServoKd=ServoKd_induc[1];
    }
    else  if(PID_Option==2)//电磁坡道pid
    {
        ServoKp=ServoKp_induc[0];
        ServoKd=ServoKd_induc[0];
    }
    else  if(PID_Option==3)//出库入库pid
    {
        ServoKp=1;
        ServoKd=0;
    }
    else  if(PID_Option==4)//环中pid
    {
        ServoKp=ServoKp_Annulu[Annulu_Radius];
        ServoKd=ServoKd_Annulu[Annulu_Radius];
    }
///////////////////////////////////////////////////////////
//计算偏差并给到舵机
///////////////////////////////////////////////////////////
    //存储偏差
    for(int i = 0; i <= LSDepth-2; i++)ServoFuzzyErrStore[i] = ServoFuzzyErrStore[i+1];
    ServoFuzzyErrStore[9] = ServoFuzzyErr;
    /*偏差变化率（最小二乘法）*/
    ServoFuzzyErrDerivative = Slope_Calculate(0, 8, ServoFuzzyErrStore);
    //依据不同情况输出不同舵机偏差值
//    if(InductanceSAVE[0]<OutTrack && InductanceSAVE[1]<OutTrack) ServoFuzzyOutput = 0;//出賽道保护
//    else
        ServoFuzzyOutput = ServoKp*ServoFuzzyErr + ServoKd*ServoFuzzyErrDerivative;  //舵机PID控制
    //最终输出舵机值
//    if(turn_flag == 0)
    ServoDuty = (int)(-ServoFuzzyOutput) + SERVOMIDDLE;
//    if(turn_flag == 1)ServoDuty = SERVORIGHT;
//    if(turn_flag == 2)ServoDuty = SERVOLEFT ;
    //舵机限幅
    ServoDuty = ServoDuty<SERVORIGHT?(ServoDuty>SERVOLEFT?ServoDuty:SERVOLEFT):SERVORIGHT;
    pwm_duty(S_MOTOR_PIN,ServoDuty);
    //pwm_duty(S_MOTOR_PIN,Duty); //测试左右极限时用
}
/***************************************************************************************
函 数 名 :Motor_Control()
功    能 :电机控制
传    入 :无
返 回 值 :无
整定参数 :无
***************************************************************************************/
void Motor_Control()
{
//电机测试用
//    pwm_duty(ATOM0_CH3_P21_5, duty_front); //电机正转
//    pwm_duty(ATOM0_CH0_P21_2, duty_behind);//电机反转
//编码器
    encoder_speed = gpt12_get(GPT12_T5);
    gpt12_clear(GPT12_T5);
//总路程记步
    //car_stop_real_distance+=encoder_speed;
//坡道记步
//    if (ramp_flag==1)ramp_flag_real_distance+=encoder_speed;
//    else ramp_flag_real_distance=0;
//电机控制
    //获得pid实际速度
    MotorNowSpeed = (float)(encoder_speed*10);

    //出库速度设定
    if      (OutgarageFlag == 0)  SpeedSet = 0;                            //等待出库（速度为零）
    else  if(OutgarageFlag == 1)  SpeedSet  = (float)OutgarageSpeed;       //正在出库(出库速度)
    else
    {
        //正常行驶速度设置
        SpeedSet=SpeedSet_Nnomal;

        //开始入库
        if     (ZebraCrossingFlag==0)ZebraDistance_Real=0;
        else if(ZebraCrossingFlag==1){SpeedSet=(float)ZebraSpeed;ZebraDistance_Real+=encoder_speed;}
        else SpeedSet=0;
        //坡道变速
        if       ( ramp_flag==1 )SpeedSet=SpeedSet_ramp[0];
        else  if ( ramp_flag==2 )SpeedSet=SpeedSet_ramp[1];
        else  if ( ramp_flag==3 )SpeedSet=SpeedSet_ramp[2];
        else  if ( ramp_flag==4 )SpeedSet=SpeedSet_ramp[3];
        else  if ( ramp_flag==5 )SpeedSet=SpeedSet_ramp[4];
        else  if ( ramp_flag==6 )SpeedSet=SpeedSet_ramp[5];
        //环岛速度控制
        if       ( Annulu_Flag==1 )SpeedSet=SpeedSet_Annulu[0]; //准备入环
        else if  ( Annulu_Flag==2 )SpeedSet=SpeedSet_Annulu[1]; //开始入环
        else if  ( Annulu_Flag==3 )SpeedSet=SpeedSet_Annulu[2]; //正在环中
        else if  ( Annulu_Flag==4 || Annulu_Flag==5 || Annulu_Flag==6)SpeedSet=SpeedSet_Annulu[3]; //准备出环//正在出环（第一阶段）//正在出环（第二阶段）
        //出赛道电机保护
        if(InductanceSAVE[2]<OutTrack && InductanceSAVE[1]<OutTrack)SpeedSet=0;
    }
    //行驶速度设定（期望速度设定）
    MotorExpSpeed = SpeedSet*10;


    //总路程记步停车
    if(car_stop_real_distance>car_stop_limit_distance){MotorExpSpeed=0;car_stop_reset=1;}
//    if(LeftlineDerivative<LeftlineDerivative_limit && RightlineDerivative>RightlineDerivative_limit)  SpeedSet=0;
    //速度偏差存储/求斜率
    for(int i = 0; i <= LSDepth-2; i++)MotorErrArray[i] = MotorErrArray[i+1];
    MotorErrArray[9] = MotorNowSpeed - MotorExpSpeed;
    MotorErrDerivativeLast=MotorErrDerivative;
    MotorErrDerivative = Slope_Calculate(0, 8, MotorErrArray);
    //运用pid得输出值
    //存储
    MotorOutLast = MotorOut;
    //积分限幅
    MotorOutLast = MotorOutLast>6000?6000:MotorOutLast;
    MotorOutLast = MotorOutLast<-6000?-6000:MotorOutLast;
    //左增量pid计算
    MotorOut = MotorOutLast-(int)(MotorI*MotorErrArray[9] + MotorK*(MotorErrArray[9]- MotorErrArray[8])
               - MotorD*(MotorErrDerivative-MotorErrDerivativeLast));
    //控制电机
    if(MotorOut>=0)//左电机
    {
        MotorOut = MotorOut>10000?10000:MotorOut;//分母为10*10000（FTM.H）
        pwm_duty(ATOM0_CH3_P21_5, (int)MotorOut); //电机正转
        pwm_duty(ATOM0_CH0_P21_2, 0);//电机反转
    }
    else if(MotorOut<0)
    {
        MotorOut = MotorOut<-10000?-10000:MotorOut;//分母为10*10000（FTM.H）
        pwm_duty(ATOM0_CH3_P21_5, 0); //电机正转
        pwm_duty(ATOM0_CH0_P21_2, (int)(-MotorOut));//电机反转
    }
}
/***************************************************************************************
函 数 名 :void Signal_Init()
功    能 :电感初始化，按下中间按键初始化完成
传    入 :无
返 回 值 :无
整定参数 :无
***************************************************************************************/
void Signal_Init()
{
    int i=0;
    //延时200ms
    systick_delay_ms(STM0, 200);
    //初始化电感内存
    for(i=0;i<4;i++)InductanceGetlookMax[i]=0;
    //电感值调试（动态调试）
    while(1)
    {
        //电感采集(采集5次然后返回平均值)
        InductanceGetlook[0]=adc_mean_filter(ADC_0, Diangan0, ADC_12BIT, 5);
        InductanceGetlook[1]=adc_mean_filter(ADC_0, Diangan1, ADC_12BIT, 5);
        InductanceGetlook[2]=adc_mean_filter(ADC_0, Diangan2, ADC_12BIT, 5);
        InductanceGetlook[3]=adc_mean_filter(ADC_0, Diangan3, ADC_12BIT, 5);
        //显示电感值
        sprintf(ch1," 0:    %d    ",InductanceGetlook[0]);
        oled_p6x8str(1,0,ch1);
        sprintf(ch1," 1:    %d    ",InductanceGetlook[1]);
        oled_p6x8str(1,1,ch1);
        sprintf(ch1," 2:    %d    ",InductanceGetlook[2]);
        oled_p6x8str(1,2,ch1);
        sprintf(ch1," 3:    %d    ",InductanceGetlook[3]);
        oled_p6x8str(1,3,ch1);
        //电感清零
        InductanceGetlook[0] = 0;
        InductanceGetlook[1] = 0;
        InductanceGetlook[2] = 0;
        InductanceGetlook[3] = 0;
        //切出电感初始化模式
        if(gpio_get(KeyTip)==0)
        {
            systick_delay_ms(STM0, 200);
            oled_fill(0x00);
            break;
        }
    }
    while(1)
    {
        int16 mpu_gyroZ=0;
        int   compensate=5;                        //消除零飘

        //电感采集(采集2次然后返回平均值)
        InductanceGetlook[0]=adc_mean_filter(ADC_0, Diangan0, ADC_12BIT, 2);
        InductanceGetlook[1]=adc_mean_filter(ADC_0, Diangan1, ADC_12BIT, 2);
        InductanceGetlook[2]=adc_mean_filter(ADC_0, Diangan2, ADC_12BIT, 2);
        InductanceGetlook[3]=adc_mean_filter(ADC_0, Diangan3, ADC_12BIT, 2);
        //获得电感最大值
        if(InductanceGetlook[0] > InductanceGetlookMax[0])
          InductanceGetlookMax[0] = InductanceGetlook[0];
        if(InductanceGetlook[1] > InductanceGetlookMax[1])
          InductanceGetlookMax[1] = InductanceGetlook[1];
        if(InductanceGetlook[2] > InductanceGetlookMax[2])
          InductanceGetlookMax[2] = InductanceGetlook[2];
        if(InductanceGetlook[3] > InductanceGetlookMax[3])
          InductanceGetlookMax[3] = InductanceGetlook[3];
        //显示电感最大值
        sprintf(ch1," 0:        %d    ",InductanceGetlookMax[0]);
        oled_p6x8str(1,0,ch1);
        sprintf(ch1," 1:        %d    ",InductanceGetlookMax[1]);
        oled_p6x8str(1,1,ch1);
        sprintf(ch1," 2:        %d    ",InductanceGetlookMax[2]);
        oled_p6x8str(1,2,ch1);
        sprintf(ch1," 3:        %d    ",InductanceGetlookMax[3]);
        oled_p6x8str(1,3,ch1);
        //MPU6050
        get_accdata();
        get_gyro();
        mpu_gyroZ=mpu_gyro_z+compensate;
        sprintf(ch1,"gyro_z:    %d    ",mpu_gyro_z);
        oled_p6x8str(1,4,ch1);
        sprintf(ch1,"compensate:%d   *",compensate);
        oled_p6x8str(1,5,ch1);
        sprintf(ch1,"gyroZ:     %d    ",mpu_gyroZ);
        oled_p6x8str(1,6,ch1);

        if(gpio_get(KeyUp)==0)
        {
            count_oled++;
            if(count_oled==1)compensate++;
            else if(count_oled>count_oled_time) count_oled=0;
        }
        if(gpio_get(KeyDown)==0)
        {
            count_oled++;
            if(count_oled==1)compensate--;
            else if(count_oled>count_oled_time)count_oled=0;
        }
        //切出电感最大值观测模式
        if(gpio_get(KeyTip)==0)
        {
            systick_delay_ms(STM0, 200);
            oled_fill(0x00);
            count_oled=0;
            break;
        }
        //按键延迟计数参数归零
        if(gpio_get(KeyTip)==1 && gpio_get(KeyUp)==1 && gpio_get(KeyDown)==1 && gpio_get(KeyRight)==1 &&  gpio_get(KeyLeft)==1)
        {
           count_oled=0;
        }
    }

}
/***************************************************************************************
函 数 名 :void Inductance_GetDeal()
功    能 :电感信号采集、处理
传    入 :无
返 回 值 :无
整定参数 :无
***************************************************************************************/
void Inductance_GetDeal()
{
    InductanceGetlookMax[0]=4095;
    InductanceGetlookMax[1]=4095;
    InductanceGetlookMax[2]=4095;
    InductanceGetlookMax[3]=4095;
/*****************电感信息处理***************/
    //电感采集(采集5次然后返回平均值)
    InductanceGetlook[0]=adc_mean_filter(ADC_0, Diangan0, ADC_12BIT, 5);
    InductanceGetlook[1]=adc_mean_filter(ADC_0, Diangan1, ADC_12BIT, 5);
    InductanceGetlook[2]=adc_mean_filter(ADC_0, Diangan2, ADC_12BIT, 5);
    InductanceGetlook[3]=adc_mean_filter(ADC_0, Diangan3, ADC_12BIT, 5);
    //存储
    InductanceSAVE[0] = InductanceGetlook[0];
    InductanceSAVE[1] = InductanceGetlook[1];
    InductanceSAVE[2] = InductanceGetlook[2];
    InductanceSAVE[3] = InductanceGetlook[3];
    //清零
    InductanceGetlook[0] = 0;
    InductanceGetlook[1] = 0;
    InductanceGetlook[2] = 0;
    InductanceGetlook[3] = 0;
    //归一化
    InductanceGet[0] = (int)((1.0 * InductanceSAVE[0] / InductanceGetlookMax[0]) * 1000);
    InductanceGet[1] = (int)((1.0 * InductanceSAVE[1] / InductanceGetlookMax[1]) * 1000);
    InductanceGet[2] = (int)((1.0 * InductanceSAVE[2] / InductanceGetlookMax[2]) * 1000);
    InductanceGet[3] = (int)((1.0 * InductanceSAVE[3] / InductanceGetlookMax[3]) * 1000);
    //限幅
    if(InductanceGet[0] > 1000)InductanceGet[0] = 1000;
    if(InductanceGet[1] > 1000)InductanceGet[1] = 1000;
    if(InductanceGet[2] > 1000)InductanceGet[2] = 1000;
    if(InductanceGet[3] > 1000)InductanceGet[3] = 1000;
    //加一防止出现除法运算分子为零的情况
    InductanceGet[0] += 1;
    InductanceGet[1] += 1;
    InductanceGet[2] += 1;
    InductanceGet[3] += 1;
/*****************求变化率***************/
    //左侧
    //左侧电感数据更新
    for(int i = 0; i <= LSDepth-2; i++)InductanceGetStoreLeft[i] = InductanceGetStoreLeft[i+1];
    InductanceGetStoreLeft[9] = InductanceSAVE[0];
    //左端电感变化率（最小二乘法）
    InductanceLeftDerivative = Slope_Calculate(0, 8, InductanceGetStoreLeft);
    //右侧
    //右侧电感数据更新
    for(int i = 0; i <= LSDepth-2; i++)InductanceGetStoreRight[i] = InductanceGetStoreRight[i+1];
    InductanceGetStoreRight[9] = InductanceSAVE[1];
    //右端电感变化率（最小二乘法）
    InductanceRightDerivative = Slope_Calculate(0, 8, InductanceGetStoreRight);
    //近端
    //近端电感数据更新
    for(int i = 0; i <= LSDepth-2; i++)InductanceGetStoreNear[i] = InductanceGetStoreNear[i+1];
    InductanceGetStoreNear[9] = InductanceSAVE[2];
    //近端电感变化率（最小二乘法）
    InductanceNearDerivative = Slope_Calculate(0, 8, InductanceGetStoreNear);
    //远端
    //远端电感数据更新
    for(int i = 0; i <= LSDepth-2; i++)InductanceGetStoreFar[i] = InductanceGetStoreFar[i+1];
    InductanceGetStoreFar[9] = InductanceSAVE[3];
    //远端电感变化率（最小二乘法）
    InductanceFarDerivative = Slope_Calculate(0, 8, InductanceGetStoreFar);
/*****************得电磁偏差***************/
    //差比积算得偏差结果
    InductanceDeal[0] = (1.0*(InductanceGet[2] - InductanceGet[1])) / (1.0*(InductanceGet[2] * InductanceGet[1])) * 5000;
    //消抖
    if(InductanceDeal[0] <= 0.1 && InductanceDeal[0] >= -0.1) InductanceUse[0] = 0;
    else InductanceUse[0] = InductanceDeal[0];
    //限幅
    if(InductanceUse[0] > 35)InductanceUse[0] = 35;
    if(InductanceUse[0] < -35)InductanceUse[0] = -35;

    //存储上一次最终偏差
    InductanceUseLast=InductanceUseRecult;
    //得此次最终偏差
    InductanceUseRecult = InductanceUse[0];
    //一阶滞后滤波
    InductanceUseRecult = InductanceUseWi*InductanceUseRecult + (1-InductanceUseWi)*InductanceUseLast;
}
/***************************************************************************************
函 数 名 :float Slope_Calculate(uint8 begin,uint8 end,float *p)
//功    能 :最小二乘法拟合斜率
传    入 :无
返 回 值 :无
整定参数 :无
范例:       //for(int i = 0; i <= LSDepth-2; i++)
        //{
        //  InductanceGetStoreNear[i] = InductanceGetStoreNear[i+1];
        //}
        //InductanceGetStoreNear[9] = Inductanceleft_two;
        //InductanceNearDerivative = Slope_Calculate(0, 8, InductanceGetStoreNear);
***************************************************************************************/
float Slope_Calculate(uint8 begin,uint8 end,float *p)
{
    float xsum=0,ysum=0,xysum=0,x2sum=0;
    uint8 i=0;
    float result=0;
    static float resultlast;
    p=p+begin;
    for(i=begin;i<end;i++)
    {
       xsum+=i;
       ysum+=*p;
       xysum+=i*(*p);
       x2sum+=i*i;
       p=p+1;
    }
    if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零
    {
    result=((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum);
    resultlast=result;
    }
    else
    {
    result=resultlast;
    }
    return result;
}
/***************************************************************************************
函 数 名 :void Buzzing_Remind(void)
功    能 :蜂鸣器警示
传    入 :无
返 回 值 :无
整定参数 :无
***************************************************************************************/
void Buzzing_Remind()
{
    if(Buzzing_start_flag==1)
    {
        Buzzing_count++;
        if(Buzzing_count<Buzzing_count_limit)
        {
            gpio_set(P21_6, 1);
        }
        else
        {
            gpio_set(P21_6, 0);
            Buzzing_start_flag=0;
            Buzzing_count=0;
        }
    }
}
#pragma section all restore
