#include "Cpu0_Main.h"
#include "headfile.h"
#include "stdlib.h"
#pragma section all "cpu0_dsram"

/////////////////////////////////////�������б��//////////////////////////////////////////
int NormalOperationflag=0;      //�������б�� 0Ϊ�������У�1Ϊ��Ԫ����
/////////////////////////////////////ͼ��ɼ�//////////////////////////////////////////
int practical_maxline = 50;            //����ͼ����Ϣ�����������
int practical_image[120][188]={{0}};   //ͼ��ɼ��󽫲ɼ��������Ԥ�棨ת�棩
uint8 mt9v03x_image_out[188]={0};
int mt9v03x_image_Resolution[188]={0};
int leftline [120]={0};  //ʵʱ��߽�
int rightline[120]={0};  //ʵʱ�ұ߽�
int width_out[120]={0};  //����ж�Ӧʵʱ�������
int width_stand_out[120] ={0};  //���б�׼��� ��Ϻ���+����ƫ��
int midline  [120]={0};  //����ж�Ӧʵʱ����
int result_midline[120]={0}; //�����������pid�е�����ֵ
int midline_stand=91;        //��׼����
int midline_error[120]={0};  //����ж�Ӧʵʱƫ��***
int image_average[120]={0};  //����е����ص��ֵ***************************
int image_average_bottom =180; //ǰ���Ƿ�Ϊȫ�ڵ���ֵ
int turn_flag=0;             //ת��������־λ����ת������ת
int line_white_dot_sum[120]={0}; //һ���а׵����
int line_black_dot_sum[120]={0}; //һ���кڵ����

int left_limit = 10;
int right_limit = 180;
int line_search_start[120]={91};
int left_flag[120]={0};
int right_flag[120]={0};
int left_hop[120]={0};
int right_hop[120]={0};
int hop=115;             //����Ҷ�ֵ                                              //*
int hop_bottom = 10 ;//*//

int front_text0=10;     //��Ч��0 (�����б�ǰ����Ϣ)                              //*//
int front_text1=15;     //��Ч��1                                                 //*//
int front_text2=20;     //��Ч��2                                                 //*
int front_text3=35;     //��Ч��3                                                 //*//
int front_text4=55;     //��Ч��4                                                 //*//
int front_text5=80;     //��Ч��5                                                 //*//
int front_text6=100;    //��Ч��6                                                 //*//
//���ұ߽��ȡ
float LeftlineArray[10]={0.0};
float RightlineArray[10]={0.0};
float LeftlineDerivativeLast = 0.0; //��߽���һ��б��
float RightlineDerivativeLast = 0.0;//�ұ߽���һ��б��
float LeftlineDerivative = 0.0;     //��߽�б��
float RightlineDerivative = 0.0;    //�ұ߽�б��

float LeftlineDerivative_limit  = -1.5;
float RightlineDerivative_limit =  1.5;
//ͼ����Ϣ���
int three_row_white_flag = 0; //���а�ɫ�������ȫ�ף�
int image_average_limit_bottom=100;  //�����ؾ�ֵ���ޣ�С����ȫ�ڣ�
//�߽�б�ʽ���
int  LeftLineSlope[120]={-200};    //��¼������߽�б��
int  RightLineSlope[120]={200};   //��¼�����ұ߽�б��
int LeftSlopeLimit[4]=  {-1,-7,-20,-50}; //��߽�б�ʷ���߽�    �ɵ�
int RrightSlopeLimit[4]={ 1, 7, 20, 50}; //�ұ߽�б�ʷ���߽�    �ɵ�
int Line_Demarcation[3]={10,20,35};      //�зֽ���
int LeftSlopeSortCount[5][5]={{0}};      //��߽�б�ʷ���ͳ�ƣ�[��������][��������]ƫ˳��ͳ�ƣ�    ����
int RrightSlopeSortCount[5][5]={{0}};    //��߽�б�ʷ���ͳ�ƣ�[��������][��������]ƫ˳��ͳ�ƣ�    ����
/////////////////////////////////////���//////////////////////////////////////////////
int16 encoder_speed=0; //������
//���������
int duty_front = 1500; //��ת
int duty_behind = 0;   //��ת
//�ٶ�ֵ
float SpeedSet = 0;                         //�ٶ�����***********************
float SpeedSet_Nnomal=75;                   //�����ٶ�
float SpeedSet_ramp[6]={75,50,50,75,75,75}; //�µ��ٶ�
float SpeedSet_Annulu[4]={60,65,85,60};     //Բ���ٶ�
int   OutgarageSpeed = 75;                  //�����ٶ�
int   ZebraSpeed = 60;                      //����ٶ�
float MotorNowSpeed = 0; //ʵʱ�ٶ�
float MotorExpSpeed = 0; //�����ٶ�
//�ٶȱջ�
float MotorErrArray[10] = {0};//�ٶ�ƫ��洢
float MotorErrDerivativeLast = 0;//��һ���ٶ�ƫ��仯��
float MotorErrDerivative = 0;//�ٶ�ƫ��仯��

float MotorK =  25.0 ; //�ٶȱջ�kp�����Ƹ���仯ʱ�Ĺ���ƫ�
float MotorD =  0.0 ; //�ٶȱջ�kd�����ƾ�̬������
float MotorI =  1.6 ; //�ٶȱջ�ki��Ӳ�ȣ�
//�ٶ����
float MotorOutLast = 0;
float MotorOut = 0;
/////////////////////////////////////���//////////////////////////////////////////////
//�����ƫ��
int SERVOLEFT   = 560; //(-135)
int SERVOMIDDLE = 695;
int SERVORIGHT  = 830; //(+135)
//FuzzyPID

float ServoFuzzyErr = 0.0,ServoFuzzyErrStore[10] = {0.0},ServoFuzzyErrDerivative = 0.0;
float ServoFuzzyOutput = 0.0;
float ServoKp = 0.0;                                              //����ʹ��pid
float ServoKd = 0.0;
float ServoKp_induc[2] = {1.50,5.0};                                       //*
float ServoKd_induc[2] = {0.0,0.0};                                        //*
int PID_Flag=0;
int PID_Option=0; //pidѡ��0Ϊ�ֶ�ʽpid��1Ϊ��������ܶ�pid��2Ϊ����µ�pid��3Ϊ����pid��Ĭ��Ϊ0
float ServoKp_image1 = 1.20;                                      //*1.6  1.4
float ServoKd_image1 = 15.0;                                      //*25   2.0
float ServoKp_image2 = 2.20;                                      //*2.6  2.6
float ServoKd_image2 = 30.0;                                      //*30   30
float ServoKp_image3 = 2.55;                                      //*1.6  1.7
float ServoKd_image3 = 40.0;                                      //*30   30
float ServoKp_all = 1.80;                                         //*
float ServoKd_all = 25.0;                                         //*
int Sub_PID[4]={-30,-8,8,30};
//������
int   ServoDuty = 0;//���ʵ����
int   Duty = 695;   //���������
/////////////////////////////////////�����Ϣ//////////////////////////////////////////
uint16 InductanceGetlook[4] = {0};                         //��ʼ���ֵ
uint16 InductanceGetlookMax[4] = {0};                      //������ֵ
uint16 InductanceGet[4] = {0};                             //��һ�����
float InductanceSAVE[4]= {0};                              //����ʵ������ʱ�洢��вɼ�ֵ
float InductanceDeal[1] = {0.0};                           //��Ȼ����(ƫ��)
float InductanceUse[1] = {0.0};                            //ƫ���������
float InductanceUseRecult = 0.0;                           //�����������ߵ�����
float InductanceUseLast=0.0;                               //��һ���������ߵ�����
float InductanceUseWi=0.95;                                //һ���ͺ��˲�ϵ��(ϵ��*���+(1-ϵ��)*�ϴ�)
int   OutTrack =300;                                       //���������ҵ��
/////////////////////////////////////��С���˷�////////////////////////////////////////
//��б����ֵ����
int LSDepth = 10;
//��˵�б仯��
float InductanceGetStoreLeft[10] = {0.0};
float InductanceLeftDerivative= 0.0;
float InductanceLeftDerivative_last= 0.0;
float InductanceLeftDerivative_k= 0.0;
//�Ҷ˵�б仯��
float InductanceGetStoreRight[10] = {0.0};
float InductanceRightDerivative= 0.0;
float InductanceRightDerivative_last= 0.0;
float InductanceRightDerivative_k= 0.0;
//���˵�б仯��(��)
float InductanceGetStoreNear[10] = {0.0};
float InductanceNearDerivative= 0.0;
float InductanceNearDerivative_last= 0.0;
float InductanceNearDerivative_k= 0.0;
//Զ�˵�б仯�ʣ�����
float InductanceGetStoreFar[10] = {0.0};
float InductanceFarDerivative= 0.0;
float InductanceFarDerivative_last= 0.0;
float InductanceFarDerivative_k= 0.0;
/////////////////////////////////////����////////////////////////////////////////////
/*�̶�����*/
int OutgarageFlag = 1;       //�����Ƿ���⣬0׼������1����2����
/*���ڲ���*/
int OutgarageDirection=1;    //���ⷽ��0Ϊ����⣬1Ϊ�ҳ��⣬Ĭ��Ϊ�ҳ��⣨1��
int OutgarageServoDuty[2] = {110,-110};  //����ʱ����趨ֵ
float OutgarageAngle=0;        //����Ƕ�
float OutgarageAngleLimit=70;  //����Ƕ�����
//////////////////////////////////ʶ�������/////////////////////////////////////////
/*�̶�����*/
int ZebraWtoB_count[120]={0}; //���аױ�ڸ���
int ZebraBtoW_count[120]={0}; //���кڱ�׸���
int ZebraWtoB_column[120][15]={{0}};//�ױ��������
int ZebraBtoW_column[120][15]={{0}};//�ڱ��������
int ZebraErrorSum=0;         //������ǰ����ƫ���
int ZebraErrorCount=0;       //������ǰ��Ч����
int ZebraError=0;            //������ǰ����ƫ��
int ZebraCrossingFlag=0;     //�����߱��
/*���ڲ���*/
int ZebraCrossingLine1 =20;  //������������
float ZebraAngle=0.0;        //���ǽǶ�
int ZebraAngleLimit=70;      //�������Ƕ�
int ZebraDistance_Real=0;    //�����ʻ����
int ZebraDistance_Limit=2500;//����һ�׶���ʻ��������
int EndLeftturn=155;         //�����������
int EndRightturn=-120;       //�����������
/////////////////////////////////////Բ��////////////////////////////////////////
int Annulu_Direction=0;    //Բ�������ǣ�0Ϊ������1Ϊ��Բ����2Ϊ��Բ��
int Annulu_Flag=0;         //Բ����ǣ�0Ϊ������
int Annulu_Flag_supplement=0;   //Բ����ǻ����е�С����
int SlopePASSMargin[2][2]={{5,8},{2,13}};  //�߽�б������Ч�ж����������ڴ˲�����Ч��(��Ϊ�ϸ�Ҫ�󼰷��ϸ�Ҫ��)
float AnnuluAngle=0;         //Բ�������ǽǶ�
float AnnuluAngleLimit_Left[3]={-45,-230,-270};   //Բ�������ǽǶ����ƣ��󣩣��ж��ڻ��У������������ѳ�����
float AnnuluAngleLimit_Right[3]={ 45, 230, 270};   //Բ�������ǽǶ����ƣ��ң����ж��ڻ��У������������ѳ�����
//׼���뻷
int Annulu_one_count=0;     //׼���뻷����һ���б߽�������
int Annulu_one_error_sum=0; //׼���뻷����Ч�е�ƫ���
int Annulu_one_error=0;     //׼���뻷����Ч�е�ƫ��
//��ʼ�뻷
//int Annulu_Two_count=0;     //��ʼ�뻷����һ���б߽�������
//int Annulu_Two_error_sum=0; //��ʼ�뻷����Ч�е�ƫ���
//int Annulu_Two_error=0;     //��ʼ�뻷����Ч�е�ƫ��
//int Annulu_Two_error_last=0;//��ʼ�뻷����Ч�е���һ��ƫ��
int Annulu_Radius=1;       //Բ���뾶ѡ��0Ϊ50��1Ϊ60��2Ϊ70��3Ϊ80 (Ĭ��Ϊ1)
int Annulu_Two_LeftTurn[4]  = { 50, 35, 35, 30};//40Ϊ������ϣ��������
int Annulu_Two_RightTurn[4] = {-50,-30,-35,-30};   //��תpwm��ΧС����ü����仯��С;��תpwm��Χ�󣬴�û�,�仯����pid��������ת���ɿ����޸�pidҲ�ɿ��ǹ�һ������תpwm����
float ServoKp_Annulu[4]= {1.8,1.8,1.8,1.8};//��ͬ�뾶����Ӧ��ͬ���KP
float ServoKd_Annulu[4]= { 25, 25, 25, 25};//��ͬ�뾶����Ӧ��ͬ���KD

/////////////////////////////////////�µ�ʶ��//////////////////////////////////////////
int ramp_flag=0;              //�µ���־
int ramp_flag_real_distance=0;       //�µ���ʻʵʱ����
int ramp_flag_limit_distance=20000;  //�µ���ʻ���ƾ���
int ramp_discern_top=12;      //�µ�ʶ���ϱ߽�
int ramp_discern_bottom=1;    //�µ�ʶ���±߽�
/////////////////////////////////////FLASH/////////////////////////////////////////////

/////////////////////////////////////��������ʾ////////////////////////////////////////
int Buzzing_count=0;        //��������ʱ��
int Buzzing_count_limit=2000; //��������ʱ����ʱʱ���趨
int Buzzing_start_flag=0;   //������������־
/////////////////////////////////////��������//////////////////////////////////////////
int runtime=0;                             //��ʱ����������ʱ��
int runtimemax=0;
int i=0;
int j=0;
/////////////////////////////////////ͣ��//////////////////////////////////////////
long int car_stop_real_distance=0;          //������ʻʵ�ʾ���
long int car_stop_limit_distance=23000*12;  //������ʻ���ƾ���12.5
int car_stop_reset=0;                       //��ʻ��������
////////////////////////////////////////TFmini/////////////////////////////////////////
uint8  TFminidate[9]={0};      //TFminidate[4]Ϊ����
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
    /********************----------------������ʼ��������һ�Σ�----------------********************/
    for(int i=0;i<=120;i++)
    {
        width_stand_out[i]=(int)(2.8895*i + 27.174+2);    //�б�׼��� ��Ϻ���+����ƫ��                     //*//
        if(width_stand_out[i]>188)width_stand_out[i]=188;
    }
    width_stand_out[front_text2]=92;
    /********************---------------uart��ʼ��-----------------********************/
    //����1Ĭ�����ӵ�����ͷ���ô���
    /********************----------------FTM��ʼ��-----------------********************/
    //��������ʼ��
    gpt12_init(GPT12_T5, GPT12_T5INB_P10_3, GPT12_T5EUDB_P10_1);
    /********************----------------PWM��ʼ��-----------------********************/
    //�������
    gtm_pwm_init(S_MOTOR_PIN, 50, SERVOMIDDLE);  //��750 ��520��-145����980��+145��
    //�����ʼ��
    gtm_pwm_init(ATOM0_CH3_P21_5, 13000, 0);     //�����ϣ�
    gtm_pwm_init(ATOM0_CH0_P21_2, 13000, 0);     //�����£�
    /********************----------------ADC ��ʼ��-----------------********************/
    adc_init(ADC_0, ADC0_CH0_A0);                //��ʼ��ADC0 ͨ��0 ʹ��A0����
    adc_init(ADC_0, ADC0_CH1_A1);
    adc_init(ADC_0, ADC0_CH2_A2);
    adc_init(ADC_0, ADC0_CH3_A3);

    adc_init(ADC_0, ADC0_CH6_A6);
    /********************---------------gpio��ʼ��-----------------********************/
    //LED��
    gpio_init(P20_8, GPO, 0, PUSHPULL);          //����P20_8Ϊ��� Ĭ������͵�ƽ  PUSHPULL���������
    gpio_init(P20_9, GPO, 0, PUSHPULL);
    //������
    gpio_init(P21_6, GPO, 0, NO_PULL);           //��Դ������
    //gtm_pwm_init(ATOM1_CH7_P02_7, 4000, 9000); //��Դ������
    //���������ʼ��GPIOΪ����
    //������Ҫ�����Ž��г�ʼ�������Ұ������ó����롢�ߵ�ƽ���ߵ͵�ƽʵ������ν�����������Ӳ�������������
    gpio_init(KeyTip,  GPI, 1, PULLUP);
    gpio_init(KeyUp,   GPI, 1, PULLUP);
    gpio_init(KeyDown, GPI, 1, PULLUP);
    gpio_init(KeyLeft, GPI, 1, PULLUP);
    gpio_init(KeyRight,GPI, 1, PULLUP);
    /********************-----------�������Ż�����ʼ��-----------********************/
    //��ʼ������ͷ
    mt9v03x_init();
    /********************----------------�жϳ�ʼ��----------------********************/
    //ʹ��CCU6_0ģ���ͨ��1 ����һ��3ms�������ж�
    pit_interrupt_ms(CCU6_0, PIT_CH1, 2);   //���Ƴ���  //ÿ�θı��ж�����ʱ����ע���޸��趨�ٶȣ�������ֵ�����ı䣩
    /********************-------�ֳ�����������ʼ����Ϣ�ɼ�---------********************/
//    Signal_Init();  //��г�ʼ��
}
/***************************************************************************************
�� �� �� :int core0_main(void)
��    �� :������
��    �� :��
�� �� ֵ :��
�������� :��
***************************************************************************************/
int core0_main(void)
{
    get_clk();//��ȡʱ��Ƶ��  ��ر���
    //�û��ڴ˴����ø��ֳ�ʼ��������
    Initialization(); //�����ʼ��

    //�ȴ����к��ĳ�ʼ�����
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    enableInterrupts();
    while (TRUE)
    {
        //TFmini�ɼ�����(����25cmΪ�ߵ�ƽ������Ϊ�͵�ƽ)
        tfmini_return=adc_mean_filter(ADC_0, ADC0_CH6_A6, ADC_12BIT, 1);
        //ͼ����
        Picture_Processing();
        //��д���
        Inductance_GetDeal();
        //��������ʾ
        Buzzing_Remind();
    }
}
/***************************************************************************************
�� �� �� :void Picture_Processing(void)
��     ��  :ͼ����(ͼ����Ϣ��������)
��     ��  :��
�� ��  ֵ :��
�������� :��
˵������Ŀǰ���2.16ms ʱ��仯ԭ������������ֹ������
    //ͼ����(�۲�ͼ��)
//        if(mt9v03x_finish_flag)
//        {
//          seekfree_sendimg_03x(UART_0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//          mt9v03x_finish_flag = 0;//��ͼ��ʹ����Ϻ�  ��������־λ�����򲻻Ὺʼ�ɼ���һ��ͼ��
//          //ע�⣺һ��Ҫ��ͼ��ʹ����Ϻ�������˱�־λ
//        }
    //ͼ�񴮿����(���ڹ۲�ͼ����Ч���±߽�)(���ߵ���)
//        if(mt9v03x_finish_flag)
//        {
//          for(int j=0;j<MT9V03X_W;j++)mt9v03x_image_out[j]=mt9v03x_image[front_text2][j];
//          mt9v03x_finish_flag = 0;//��ͼ��ʹ����Ϻ�  ��������־λ�����򲻻Ὺʼ�ɼ���һ��ͼ��
//          //ע�⣺һ��Ҫ��ͼ��ʹ����Ϻ�������˱�־λ
//        }
***************************************************************************************/
void Picture_Processing(void)
{
    if(mt9v03x_finish_flag)
    {
//       systick_start(STM1);//ʹ��STM1 ���м�ʱ

       //seekfree_sendimg_03x(UART_0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);

        //������������
        //comparative_result0=Image_Ternary_Segmentation(front_text0);
        //Ԥ�洢�лҶ�ֵ
//        line_white_dot_sum[front_text1]=0;line_black_dot_sum[front_text1]=0;
        line_white_dot_sum[front_text2-4]=0;line_black_dot_sum[front_text2-4]=0;
        line_white_dot_sum[front_text2  ]=0;line_black_dot_sum[front_text2  ]=0;
        line_white_dot_sum[front_text2+2]=0;line_black_dot_sum[front_text2+2]=0;
        for(int j=0;j<188;j++)
        {
            //���ڹ۲�������л��������лҶ�ֵ��������к�ɹرգ�
            practical_image[ZebraCrossingLine1][j]=mt9v03x_image[ZebraCrossingLine1][j];
            //�����ж�ʮ��(��¼)
//            if(mt9v03x_image[front_text1][j]>=hop)line_white_dot_sum[front_text1]++;
//            else line_black_dot_sum[front_text1]++;
            if(mt9v03x_image[front_text2-4][j]>=hop)line_white_dot_sum[front_text2-4]++;
            else line_black_dot_sum[front_text2-4]++;
            if(mt9v03x_image[front_text2  ][j]>=hop)line_white_dot_sum[front_text2  ]++;
            else line_black_dot_sum[front_text2  ]++;
            if(mt9v03x_image[front_text2+2][j]>=hop)line_white_dot_sum[front_text2+2]++;
            else line_black_dot_sum[front_text2+2]++;
        }
        //����������Ϣ���洢������Ҫ��ʱ��
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
        //�������ã���Ҫ�۲��������*********************************************************************************
        width_text(front_text4);width_text(front_text5);width_text(front_text6);
        //��������������
        Zebra_Crossing_Search_One(ZebraCrossingLine1-6);
        Zebra_Crossing_Search_One(ZebraCrossingLine1-5);
        Zebra_Crossing_Search_One(ZebraCrossingLine1-4);
        Zebra_Crossing_Search_One(ZebraCrossingLine1-3);
        Zebra_Crossing_Search_One(ZebraCrossingLine1-2);
        Zebra_Crossing_Search_One(ZebraCrossingLine1-1);
        Zebra_Crossing_Search_One(ZebraCrossingLine1  );
        Zebra_Crossing_Search_One(ZebraCrossingLine1+1);
        //��ȡ����б�ʲ�ͳ�Ʋ�ͬ����б�ʸ�����1���������Ч�У�
        Border_Slope_Calculate();
        //������һ�βɼ�ͼ��
        mt9v03x_finish_flag = 0;


//        runtime = systick_getval_us(STM1);         //��ȡSTM1��ʱʱ��(��ʼ��ʱ�����ڵ�ʱ��)
//        if(runtime>runtimemax)runtimemax=runtime;  //��¼���ʱ��
    }
}
/***************************************************************************************
�� �� �� :void Zebra_Crossing_Search(int line_function)
��    �� :����������
��    �� :��ѡȡ�е����
�� �� ֵ :��
�������� :��
˵��������⼴����⣬�ҳ��⼴����⣻����������������������������Ȼ��
***************************************************************************************/
void Zebra_Crossing_Search_One(int line_function)
{
    int i=0;
    //��������
    ZebraBtoW_count[line_function]=0;
    ZebraWtoB_count[line_function]=0;
    for(i=0;i<15;i++)
    {
        ZebraWtoB_column[line_function][i]=0;
        ZebraBtoW_column[line_function][i]=0;
    }
    if(OutgarageDirection == 0) //�����
    {
        for(i=2;i<=185;i++)
        {
            if( mt9v03x_image[line_function][i+2]<hop &&
                mt9v03x_image[line_function][i+1]<hop &&
                mt9v03x_image[line_function][i  ]>hop &&//���Ƶ�ǰ�жϵ�
                mt9v03x_image[line_function][i-1]>hop &&
                mt9v03x_image[line_function][i-2]>hop  )
                {ZebraWtoB_column[line_function][ZebraWtoB_count[line_function]]=i;ZebraWtoB_count[line_function]+=1;}//�����ױ��
            if( mt9v03x_image[line_function][i+2]>hop &&
                mt9v03x_image[line_function][i+1]>hop &&
                mt9v03x_image[line_function][i  ]<hop &&//���Ƶ�ǰ�жϵ�
                mt9v03x_image[line_function][i-1]<hop &&
                mt9v03x_image[line_function][i-2]<hop  )
                {ZebraBtoW_column[line_function][ZebraBtoW_count[line_function]]=i;ZebraBtoW_count[line_function]+=1; }//�����ڱ��
        }
    }
    if(OutgarageDirection == 1) //�ҳ���
    {
        for(i=185;i>=2;i--)//����⣬������������
        {
           if( mt9v03x_image[line_function][i-2]<hop &&
               mt9v03x_image[line_function][i-1]<hop &&
               mt9v03x_image[line_function][i  ]>hop &&//���Ƶ�ǰ�жϵ�
               mt9v03x_image[line_function][i+1]>hop &&
               mt9v03x_image[line_function][i+2]>hop )
               {ZebraWtoB_column[line_function][ZebraWtoB_count[line_function]]=i;ZebraWtoB_count[line_function]+=1;}//�����ױ��
           if( mt9v03x_image[line_function][i-2]>hop &&
               mt9v03x_image[line_function][i-1]>hop &&
               mt9v03x_image[line_function][i  ]<hop &&//���Ƶ�ǰ�жϵ�
               mt9v03x_image[line_function][i+1]<hop &&
               mt9v03x_image[line_function][i+2]<hop )
               {ZebraBtoW_column[line_function][ZebraBtoW_count[line_function]]=i;ZebraBtoW_count[line_function]+=1; }//�����ڱ��
        }
    }

}
/***************************************************************************************
�� �� �� :int width_text(int line_function)
��    �� :���ұ߽磨�ڼ��С��Ƿ��С��߽���ֵ�Ƕ��٣�����ȡ����߲鿴
��    �� :��ѡȡ�е����
�� �� ֵ :��
�������� :��
***************************************************************************************/
void width_text(int line_function)
{
    int i=0;
//�������ұ߽��־λ
    left_flag[line_function]=0;
    right_flag[line_function]=0;
//������߽�
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
//�����ұ߽�
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
    //�����������
    width_out[line_function]=rightline[line_function]-leftline[line_function];
    //��ȡ��������
    midline[line_function]=(rightline[line_function]+leftline[line_function])/2;
//���������������߽��
    //��߽���ʧ�������б߽��һ�߽��в��ߣ�
    if(left_flag[line_function]==0 && right_flag[line_function]==1)
    { result_midline[line_function]=rightline[line_function]-width_stand_out[line_function]/2; }
    //�ұ߽���ʧ�������б߽��һ�߽��в��ߣ�
    else if(left_flag[line_function]==1 && right_flag[line_function]==0)
    { result_midline[line_function]=leftline[line_function]+width_stand_out[line_function]/2; }
    //���ұ߽綼���������߽����Ϊ��������
    else result_midline[line_function]=midline[line_function];
//��ȡƫ��
    midline_error[line_function]=midline_stand-result_midline[line_function];
//������һ�ε��е�������� �������������(��˴����ʮ�ֻ��лζ�)(��ʮ����������ʵ�о��л������������)
    if(left_flag[line_function]==0 && right_flag[line_function]==0)
    {   }
    else
    {
        line_search_start[line_function]=result_midline[line_function];
        line_search_start[line_function]=line_search_start[line_function]>left_limit?(line_search_start[line_function]<right_limit?line_search_start[line_function]:right_limit):left_limit;//�޷�
    }
//�뻷ʱ��������㶨Ϊ�߽磨���뻷�������ң����뻷��������
    if(Annulu_Flag==2 )
    {
        if(Annulu_Direction==1)line_search_start[line_function]=20;
        if(Annulu_Direction==2)line_search_start[line_function]=170;
    }
}
/***************************************************************************************
�� �� �� :void Border_Slope_Calculate(void)
��     �� :���������ұ߽�б��
��     �� :��
�� �� ֵ :��
�������� :��
˵�����������Ϊ180�������б�ʷ�ΧӦ�ڡ�-180����180��
***************************************************************************************/
void Border_Slope_Calculate(void)
{
    //ͳ������
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
    //��߽�б�ʼ����봦��
        //��߽�б�ʼ���
        if(left_flag[i-1]==1  && left_flag[i]==1  && left_flag[i+1]==1  ) LeftLineSlope[i]=(leftline[i+1]-leftline[i-1]);
        else LeftLineSlope[i]=-200;
        //���ݼ�������б�ʽ���ͳ��
        if(i<=Line_Demarcation[0])
        {
            if     ( LeftLineSlope[i]>=LeftSlopeLimit[0])LeftSlopeSortCount[0][0]++;//��б
            else if( LeftLineSlope[i]>=LeftSlopeLimit[1])LeftSlopeSortCount[0][1]++;//����
            else if( LeftLineSlope[i]>=LeftSlopeLimit[2])LeftSlopeSortCount[0][2]++;//΢��б��������
            else if( LeftLineSlope[i]>=LeftSlopeLimit[3])LeftSlopeSortCount[0][3]++;//������б
            else if( LeftLineSlope[i]==-200)             LeftSlopeSortCount[0][4]++;//δ������б��Ҫ��
        }
        else if(i<=Line_Demarcation[1])
        {
            if     ( LeftLineSlope[i]>=LeftSlopeLimit[0])LeftSlopeSortCount[1][0]++;//��б
            else if( LeftLineSlope[i]>=LeftSlopeLimit[1])LeftSlopeSortCount[1][1]++;//����
            else if( LeftLineSlope[i]>=LeftSlopeLimit[2])LeftSlopeSortCount[1][2]++;//΢��б��������
            else if( LeftLineSlope[i]>=LeftSlopeLimit[3])LeftSlopeSortCount[1][3]++;//������б
            else if( LeftLineSlope[i]==-200)             LeftSlopeSortCount[1][4]++;//δ������б��Ҫ��
        }
        else if(i<=Line_Demarcation[2])
        {
            if     ( LeftLineSlope[i]>=LeftSlopeLimit[0])LeftSlopeSortCount[2][0]++;//��б
            else if( LeftLineSlope[i]>=LeftSlopeLimit[1])LeftSlopeSortCount[2][1]++;//����
            else if( LeftLineSlope[i]>=LeftSlopeLimit[2])LeftSlopeSortCount[2][2]++;//΢��б��������
            else if( LeftLineSlope[i]>=LeftSlopeLimit[3])LeftSlopeSortCount[2][3]++;//������б
            else if( LeftLineSlope[i]==-200)             LeftSlopeSortCount[2][4]++;//δ������б��Ҫ��
        }
        else
        {
            if     ( LeftLineSlope[i]>=LeftSlopeLimit[0])LeftSlopeSortCount[3][0]++;//��б
            else if( LeftLineSlope[i]>=LeftSlopeLimit[1])LeftSlopeSortCount[3][1]++;//����
            else if( LeftLineSlope[i]>=LeftSlopeLimit[2])LeftSlopeSortCount[3][2]++;//΢��б��������
            else if( LeftLineSlope[i]>=LeftSlopeLimit[3])LeftSlopeSortCount[3][3]++;//������б
            else if( LeftLineSlope[i]==-200)             LeftSlopeSortCount[3][4]++;//δ������б��Ҫ��
        }
    //�ұ߽�б�ʼ����봦��
        //�ұ߽�б�ʼ���
        if(right_flag[i-1]==1 && right_flag[i]==1 && right_flag[i+1]==1 )RightLineSlope[i]=(rightline[i+1]-rightline[i-1]);
        else RightLineSlope[i]=200;
        //���ݼ�������б�ʽ���ͳ��
        if(i<=Line_Demarcation[0])
        {
            if     ( RightLineSlope[i]<=RrightSlopeLimit[0])RrightSlopeSortCount[0][0]++;//��б
            else if( RightLineSlope[i]<=RrightSlopeLimit[1])RrightSlopeSortCount[0][1]++;//����
            else if( RightLineSlope[i]<=RrightSlopeLimit[2])RrightSlopeSortCount[0][2]++;//΢��б��������
            else if( RightLineSlope[i]<=RrightSlopeLimit[3])RrightSlopeSortCount[0][3]++;//������б
            else if( RightLineSlope[i]==200)                RrightSlopeSortCount[0][4]++;//δ������б��Ҫ��
        }
        else if(i<=Line_Demarcation[1])
        {
            if     ( RightLineSlope[i]<=RrightSlopeLimit[0])RrightSlopeSortCount[1][0]++;//��б
            else if( RightLineSlope[i]<=RrightSlopeLimit[1])RrightSlopeSortCount[1][1]++;//����
            else if( RightLineSlope[i]<=RrightSlopeLimit[2])RrightSlopeSortCount[1][2]++;//΢��б��������
            else if( RightLineSlope[i]<=RrightSlopeLimit[3])RrightSlopeSortCount[1][3]++;//������б
            else if( RightLineSlope[i]==200)                RrightSlopeSortCount[1][4]++;//δ������б��Ҫ��
        }
        else if(i<=Line_Demarcation[2])
        {
            if     ( RightLineSlope[i]<=RrightSlopeLimit[0])RrightSlopeSortCount[2][0]++;//��б
            else if( RightLineSlope[i]<=RrightSlopeLimit[1])RrightSlopeSortCount[2][1]++;//����
            else if( RightLineSlope[i]<=RrightSlopeLimit[2])RrightSlopeSortCount[2][2]++;//΢��б��������
            else if( RightLineSlope[i]<=RrightSlopeLimit[3])RrightSlopeSortCount[2][3]++;//������б
            else if( RightLineSlope[i]==200)                RrightSlopeSortCount[2][4]++;//δ������б��Ҫ��
        }
        else
        {
            if     ( RightLineSlope[i]<=RrightSlopeLimit[0])RrightSlopeSortCount[3][0]++;//��б
            else if( RightLineSlope[i]<=RrightSlopeLimit[1])RrightSlopeSortCount[3][1]++;//����
            else if( RightLineSlope[i]<=RrightSlopeLimit[2])RrightSlopeSortCount[3][2]++;//΢��б��������
            else if( RightLineSlope[i]<=RrightSlopeLimit[3])RrightSlopeSortCount[3][3]++;//������б
            else if( RightLineSlope[i]==200)                RrightSlopeSortCount[3][4]++;//δ������б��Ҫ��
        }
    }
    //���кϼ�
    for(int i=0;i<=4;i++)
    {
        LeftSlopeSortCount[4][i]=LeftSlopeSortCount[3][i]+LeftSlopeSortCount[2][i]+LeftSlopeSortCount[1][i]+LeftSlopeSortCount[0][i];
        RrightSlopeSortCount[4][i]=RrightSlopeSortCount[3][i]+RrightSlopeSortCount[2][i]+RrightSlopeSortCount[1][i]+RrightSlopeSortCount[0][i];
    }
}
/***************************************************************************************
�� �� �� :void Line_Jump_Search(void)
��    �� :�б߽�������
��    �� :��
�� �� ֵ :��
�������� :��
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
�� �� �� :void LineHop_Average(int line_function)
��    �� :�洢�лҶȾ�ֵ
��    �� :��ѡȡ�е����
�� �� ֵ :��
�������� :��
�Ľ���   �������Ӵ������������ٷ�������forѭ��������һ��forѭ��ͬʱ�������
***************************************************************************************/
void LineHop_Average(int line_function)
{
    int i=0;
    image_average[line_function]=0;
    for(i=4;i<=184;i++)image_average[line_function]+=mt9v03x_image[line_function][i];
    image_average[line_function]=image_average[line_function]/181;
}
/***************************************************************************************
�� �� �� :int Image_Ternary_Segmentation(int line_function)
��     ��  :����ѡ�зֳ������������֣��Ա������ֵļӺ�ֵ��С
��     ��  :��ѡȡ�е����
�� ��  ֵ :���� 1 Ϊ�м�����������ߣ�ʮ�֣������� 2 Ϊ���ߴ����м䣨���棩������0Ϊ����
�������� :��
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
�� �� �� :void Annulus_Judge(void)
��      �� :�ж�Բ��
��      �� :��
�� �� ֵ :��
�������� :��
***************************************************************************************/
void Annulus_Judge(void)
{
///////////////////////////////////////////////////////////
//�ж�
///////////////////////////////////////////////////////////
    //�ж�׼���뻷�����ڻ��ڣ�Ӧ���ټ��������������������߽�б�ʷ�����Ӧ������
         //���뻷
         if(  Annulu_Flag==0 &&
              LeftSlopeSortCount[1][2]>=SlopePASSMargin[0][0] && RrightSlopeSortCount[1][1]>=5 && //10����20��΢б������
              LeftSlopeSortCount[2][4]>=SlopePASSMargin[1][1] && RrightSlopeSortCount[2][4]<=7  ) //21--35��հ����б߽�
         {
              Buzzing_start_flag=1;//��������
              Annulu_Direction=1;  //ȷ��Ϊ��Բ��
              Annulu_Flag=1;       //ȷ��׼���뻷
         }
         //���뻷
         if(  Annulu_Flag==0 &&
              RrightSlopeSortCount[1][2]>=SlopePASSMargin[0][0] && LeftSlopeSortCount[1][1]>=5 && //10����20��΢б������
              RrightSlopeSortCount[2][4]>=SlopePASSMargin[1][1] && LeftSlopeSortCount[2][4]<=7  ) //21--35�ҿհ����б߽�
         {
             Buzzing_start_flag=1;//��������
             Annulu_Direction=2;  //ȷ��Ϊ��Բ��
             Annulu_Flag=1;       //ȷ��׼���뻷
         }
    //�жϿ�ʼ�뻷������׼���뻷�׶Σ��߽�б�ʷ�����Ӧ������
         if(Annulu_Flag==1 )
         {
             if(Annulu_Direction==1)//��Բ��
             {
                 if(Annulu_Flag_supplement==0)//׼���뻷����һ�׶Σ����ж��г�����б�ʡ����ж�����б�ʣ�
                 {
                     if(LeftLineSlope[13]!=-200 && LeftLineSlope[14]!=-200 && LeftLineSlope[15]!=-200)
                     {Buzzing_start_flag=1;Annulu_Flag_supplement=1;}
                 }
                 if(Annulu_Flag_supplement==1)//׼���뻷���ڶ��׶Σ����ж��г�����б�ʡ����ж���������б�ʣ�
                 {
                     if(LeftLineSlope[14]==-200 && LeftLineSlope[15]==-200)Annulu_Flag_supplement=2;
                 }
                 //�����ж�
                 if(Annulu_Flag_supplement==2)
                 {Buzzing_start_flag=1;Annulu_Flag=2;Annulu_Flag_supplement=0;}
             }
             if(Annulu_Direction==2)//��Բ��
             {
                 if(Annulu_Flag_supplement==0)//׼���뻷����һ�׶Σ����ж��г�����б�ʡ����ж�����б�ʣ�
                 {
                     if(RightLineSlope[13]!=200 && RightLineSlope[14]!=200 && RightLineSlope[15]!=200)
                     {Buzzing_start_flag=1;Annulu_Flag_supplement=1;}
                 }
                 if(Annulu_Flag_supplement==1)//׼���뻷���ڶ��׶Σ����ж��г�����б�ʡ����ж���������б�ʣ�
                 {
                     if(RightLineSlope[14]==200 && RightLineSlope[15]==200)Annulu_Flag_supplement=2;
                 }
                 //�����ж�
                 if(Annulu_Flag_supplement==2)
                 {Buzzing_start_flag=1;Annulu_Flag=2;Annulu_Flag_supplement=0;}
             }
         }
   //�ж����ڻ���
         //
         if(Annulu_Flag==2 )
         {
             if(Annulu_Direction==1) //��
             {
                 if(Annulu_Flag_supplement==0)//��ʼ�뻷����һ�׶Σ����ж��д��������߽硪�����������߽磩
                 {
                     if(right_flag[front_text2]==0 && right_flag[front_text2+1]==0 && right_flag[front_text2+2]==0)
                     {Buzzing_start_flag=1;Annulu_Flag_supplement=1;}
                 }
                 if(Annulu_Flag_supplement==1)//��ʼ�뻷���ڶ��׶Σ����ж��д����������߽硪�������������߽磩
                 {
                     if(right_flag[front_text2]==1 && right_flag[front_text2+1]==1 && right_flag[front_text2+2]==1)Annulu_Flag_supplement=2;
                 }
                 //�����ж�
                 if(AnnuluAngle<AnnuluAngleLimit_Left[0] || Annulu_Flag_supplement==2)
                 {Buzzing_start_flag=1;Annulu_Flag=3;Annulu_Flag_supplement=0;}
             }
             if(Annulu_Direction==2) //�һ�
             {
                 if(Annulu_Flag_supplement==0)//��ʼ�뻷����һ�׶Σ����ж��д��������߽硪�����������߽磩
                 {
                     if(left_flag[front_text2]==0 && left_flag[front_text2+1]==0 && left_flag[front_text2+2]==0)
                     {Buzzing_start_flag=1;Annulu_Flag_supplement=1;}
                 }
                 if(Annulu_Flag_supplement==1)//��ʼ�뻷���ڶ��׶Σ������ж��д����������߽硪�������������߽磩
                 {
                     if(left_flag[front_text2]==1 && left_flag[front_text2+1]==1 && left_flag[front_text2+2]==1)Annulu_Flag_supplement=2;
                 }
                 //�����ж�
                 if(AnnuluAngle>AnnuluAngleLimit_Right[0] || Annulu_Flag_supplement==2)
                 {Buzzing_start_flag=1;Annulu_Flag=3;Annulu_Flag_supplement=0;}
             }
         }
   //�ж�׼������
         if(Annulu_Flag==3)
         {
             if(Annulu_Direction==1)if(AnnuluAngle<AnnuluAngleLimit_Left[1] ){Buzzing_start_flag=1;Annulu_Flag=4;}
             if(Annulu_Direction==2)if(AnnuluAngle>AnnuluAngleLimit_Right[1] ){Buzzing_start_flag=1;Annulu_Flag=4;}
         }
   //�ж����ڳ�������һ�׶Σ�
         if(Annulu_Flag==4)
         {
             if(Annulu_Direction==1)if(right_flag[front_text2-4]==0 && right_flag[front_text2-3]==0){Buzzing_start_flag=1;Annulu_Flag=5;}
             if(Annulu_Direction==2)if(left_flag[front_text2-4]==0 && left_flag[front_text2-3]==0){Buzzing_start_flag=1;Annulu_Flag=5;}
         }
   //�ж����ڳ������ڶ��׶Σ�
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
   //�жϽ�������
         if(Annulu_Flag==6)
         {
             if(LeftSlopeSortCount[1][4]<=SlopePASSMargin[1][0] && RrightSlopeSortCount[1][4]<=SlopePASSMargin[1][0])
             {Buzzing_start_flag=1;Annulu_Flag=0;Annulu_Direction=0;}
         }
///////////////////////////////////////////////////////////
//����
///////////////////////////////////////////////////////////
    //׼���뻷
        if(Annulu_Flag==1)
        {
            //����
            Annulu_one_count=0;
            Annulu_one_error_sum=0;
            Annulu_one_error=0;
            if(Annulu_Direction==1)//��Բ��
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
            if(Annulu_Direction==2)//��Բ��
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
    //��ʼ�뻷//
        if(Annulu_Flag==2)
        {
//            //��������
//            Buzzing_start_flag=1;
//            //����
//            Annulu_Two_count=0;
//            Annulu_Two_error_sum=0;
//            Annulu_Two_error=0;
//            Annulu_Two_error_last=0;
//            if(Annulu_Direction==1)//��Բ��
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
//            if(Annulu_Direction==2)//��Բ��
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
    //���ڻ��м�׼����������Ȼ����  ������pid��PID_Option=4;width_stand_out[front_text2]=92;}else{width_stand_out[front_text2]=92;
        if(Annulu_Flag==3 || Annulu_Flag==4){}
    //���ڳ�������һ�׶Σ�
        if(Annulu_Flag==5)
        {
            if(Annulu_Direction==1)ServoFuzzyErr=(float)(Annulu_Two_LeftTurn[Annulu_Radius]);
            if(Annulu_Direction==2)ServoFuzzyErr=(float)(Annulu_Two_RightTurn[Annulu_Radius]);
        }
    //���ڳ������ڶ��׶Σ�(��Ȼ���� ��������)
        if(Annulu_Flag==6)
        { }
}
/***************************************************************************************
�� �� �� :void Servocontrol()
��    �� :�������
��    �� :��
�� �� ֵ :��
�������� :��
***************************************************************************************/
float LeftlineDerivative_wach=0;
float RightlineDerivative_wach=0;

int uphill_Angle = -10;
int Downhill_Angle = 5;
int EndRamp_Angle =  5;
void Servocontrol()
{
///////////////////////////////////////////////////////////
//���⣨�����̶��棩
///////////////////////////////////////////////////////////
    if     (OutgarageFlag == 0)ServoFuzzyErr = 0;   //�ȴ�����
    else if(OutgarageFlag == 1)                     //���ڳ���
    {
        if(OutgarageDirection == 0) ServoFuzzyErr = (float)OutgarageServoDuty[0];  //�����
        if(OutgarageDirection == 1) ServoFuzzyErr = (float)OutgarageServoDuty[1];  //�ҳ���
        //ȷ��pid
        PID_Option=3;
        if(OutgarageAngle>OutgarageAngleLimit || OutgarageAngle<-OutgarageAngleLimit)OutgarageFlag=2;
    }
    else //������ɺ�ִ����������
    {
        //    turn_flag = 0;
//        if()
//        {
//            NormalOperationflag
//        }
        //pidĬ�����ã�����ͷ�ֶ�pid��
        PID_Option=0;//Ĭ������ͷ
        PID_Flag=0;  //Ĭ�Ϸֶ�
///////////////////////////////////////////////////////////
//���ߴ�����Ҫ��һ�������ж��������� �������ɫ�㣩
///////////////////////////////////////////////////////////
            //�ڶ��ж���
            if(left_flag[front_text2]==0 && right_flag[front_text2]==0 )//&& image_average[front_text2]<image_average_limit_bottom
            {
                //����������
                //Buzzing_start_flag=1;
                if(left_flag[front_text3]==0 && right_flag[front_text3]==0 )
                {
                    if(left_flag[front_text4]==0 && right_flag[front_text4]==0 )
                    {
                        if(left_flag[front_text5]==0 && right_flag[front_text5]==0 )
                        {
                            ServoFuzzyErr=(float)(midline_error[front_text6]);     //�����в���
                        }
                        else ServoFuzzyErr=(float)(midline_error[front_text5]);    //�����в���
                    }
                    else ServoFuzzyErr=(float)(midline_error[front_text4]);   //�����в���
                }
                else ServoFuzzyErr=(float)(midline_error[front_text3]);  //�����в���
            }
            else ServoFuzzyErr=(float)(midline_error[front_text2]);  //�ڶ�������
///////////////////////////////////////////////////////////
//ʮ��(������ǰ�����пհ׵�ϴ� �����µ�  �����)
///////////////////////////////////////////////////////////
            //���ұ߽���ʧ ���л��ɵ�����ߣ�����δ���ǲ�·��
            if((line_white_dot_sum[front_text2-4]>160 || line_white_dot_sum[front_text2+2]>160)&&
                ramp_flag==0 && ZebraCrossingFlag==0)// (left_flag[front_text1]==0 && right_flag[front_text1]==0) || (left_flag[front_text2]==0 || right_flag[front_text2]==0) )
            {
                //��������
//                Buzzing_start_flag=1;
                //�л����
                ServoFuzzyErr=(float)(InductanceUseRecult);
                //ȷ��pid
                PID_Option=1;
            }
///////////////////////////////////////////////////////////
//Բ��
///////////////////////////////////////////////////////////
            Annulus_Judge();
///////////////////////////////////////////////////////////
//�µ�ʶ��(����� �����µ��� �����)
///////////////////////////////////////////////////////////
            if( tfmini_return>4000 && ramp_flag==0 && ZebraCrossingFlag==0)   ramp_flag=1;
            //���µ���ȫ��
            if ( ramp_flag!=0 )
            {
                //��������
                Buzzing_start_flag=1;
                //������ʹ��
                angle_re_use=angle_re;
                //�л����
                ServoFuzzyErr=(float)(InductanceUseRecult);
                //ȷ��pid
                PID_Option=2;
                //�ж��µ����׶�
                if ( ramp_flag==1 )     {if( angle_re_use<uphill_Angle  )  ramp_flag=2;} // ���µ�  ������������
                else if ( ramp_flag==2) {if( angle_re_use>Downhill_Angle)  ramp_flag=3;} //�������¡�����������
                else if ( ramp_flag==3) {if( left_flag[front_text2]==1 || right_flag[front_text2]==1 ||
                                             left_flag[front_text3]==1 || right_flag[front_text3]==1 ||
                                             left_flag[front_text4]==1 || right_flag[front_text4]==1 ) ramp_flag=4;}//�������¡������º�ͼ��������Ч��
                else if ( ramp_flag==4) {if( tfmini_return>4000 )          ramp_flag=5;}//���º�ͼ��������Ч�С�������
                else if ( ramp_flag==5) {if( tfmini_return<4000 )          ramp_flag=6;}//���º�ͼ��������Ч�С�������
                else if ( ramp_flag==6) {if( angle_re_use<EndRamp_Angle ){ ramp_flag=0;angle_re_use=0;}}//���ء�ƽ�ؽ����µ�
            }
///////////////////////////////////////////////////////////
//ʶ�������(�ڰף��׺�������  ���������  �����µ���  )
///////////////////////////////////////////////////////////
            if( ( (ZebraWtoB_count[ZebraCrossingLine1-1]>5 &&  ZebraBtoW_count[ZebraCrossingLine1-1]>5)  ||
                  (ZebraWtoB_count[ZebraCrossingLine1  ]>5 &&  ZebraBtoW_count[ZebraCrossingLine1  ]>5)  ||
                  (ZebraWtoB_count[ZebraCrossingLine1+1]>5 &&  ZebraBtoW_count[ZebraCrossingLine1+1]>5) )&&
                   ZebraCrossingFlag==0 && ramp_flag==0  )
            {
                //��������
//                Buzzing_start_flag=1;
                //��Ч������
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
            //������
             if(ZebraCrossingFlag==1)
            {
                //��������
//                Buzzing_start_flag=1;
                ServoFuzzyErr=(float)(ZebraError);
                if(ZebraDistance_Real>ZebraDistance_Limit )ZebraCrossingFlag=2;
            }
            if(ZebraCrossingFlag==2)
            {
                //��������
//                Buzzing_start_flag=1;
//                //����
//                ZebraErrorCount=0;//��Ч������
//                ZebraErrorSum=0;  //��������
//                for(int i=0;i<=3;i++)
//                {
//                    if(ZebraWtoB_count[ZebraCrossingLine1-i]>=1 )
//                    {
//                        if(OutgarageDirection == 0) //������������
//                        {
//                            ZebraErrorSum+=(midline_stand-(ZebraWtoB_column[ZebraCrossingLine1-i][0]-width_out2/2));
//                        }
//                        if(OutgarageDirection == 1) //�ҳ����������
//                        {
//                            ZebraErrorSum+=(midline_stand-(ZebraWtoB_column[ZebraCrossingLine1-i][0]+width_out2/2));
//                        }
//                        ZebraErrorCount++;
//                    }
//                }
                if(OutgarageDirection == 0)ZebraError=EndLeftturn; //������������
                if(OutgarageDirection == 1)ZebraError=EndRightturn; //�ҳ����������
                ServoFuzzyErr=(float)(ZebraError);
                PID_Option=3;
                if(ZebraAngle>ZebraAngleLimit || ZebraAngle<-ZebraAngleLimit)ZebraCrossingFlag=3;
            }
            if(ZebraCrossingFlag==3)
            {
                ServoFuzzyErr = 0;
            }
    }//���⺯����β
///////////////////////////////////////////////////////////
//pidѡȡ
///////////////////////////////////////////////////////////
    if(PID_Option==0)//����ͷpid
    {
        if(PID_Flag==0)//����ͷ�ֶ�pid
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
        if(PID_Flag==1)//����ͷ��pid
        {
            ServoKp=ServoKp_all;
            ServoKd=ServoKd_all;
        }
    }
    else  if(PID_Option==1) //�������pid
    {
        ServoKp=ServoKp_induc[1];
        ServoKd=ServoKd_induc[1];
    }
    else  if(PID_Option==2)//����µ�pid
    {
        ServoKp=ServoKp_induc[0];
        ServoKd=ServoKd_induc[0];
    }
    else  if(PID_Option==3)//�������pid
    {
        ServoKp=1;
        ServoKd=0;
    }
    else  if(PID_Option==4)//����pid
    {
        ServoKp=ServoKp_Annulu[Annulu_Radius];
        ServoKd=ServoKd_Annulu[Annulu_Radius];
    }
///////////////////////////////////////////////////////////
//����ƫ��������
///////////////////////////////////////////////////////////
    //�洢ƫ��
    for(int i = 0; i <= LSDepth-2; i++)ServoFuzzyErrStore[i] = ServoFuzzyErrStore[i+1];
    ServoFuzzyErrStore[9] = ServoFuzzyErr;
    /*ƫ��仯�ʣ���С���˷���*/
    ServoFuzzyErrDerivative = Slope_Calculate(0, 8, ServoFuzzyErrStore);
    //���ݲ�ͬ��������ͬ���ƫ��ֵ
//    if(InductanceSAVE[0]<OutTrack && InductanceSAVE[1]<OutTrack) ServoFuzzyOutput = 0;//��ِ������
//    else
        ServoFuzzyOutput = ServoKp*ServoFuzzyErr + ServoKd*ServoFuzzyErrDerivative;  //���PID����
    //����������ֵ
//    if(turn_flag == 0)
    ServoDuty = (int)(-ServoFuzzyOutput) + SERVOMIDDLE;
//    if(turn_flag == 1)ServoDuty = SERVORIGHT;
//    if(turn_flag == 2)ServoDuty = SERVOLEFT ;
    //����޷�
    ServoDuty = ServoDuty<SERVORIGHT?(ServoDuty>SERVOLEFT?ServoDuty:SERVOLEFT):SERVORIGHT;
    pwm_duty(S_MOTOR_PIN,ServoDuty);
    //pwm_duty(S_MOTOR_PIN,Duty); //�������Ҽ���ʱ��
}
/***************************************************************************************
�� �� �� :Motor_Control()
��    �� :�������
��    �� :��
�� �� ֵ :��
�������� :��
***************************************************************************************/
void Motor_Control()
{
//���������
//    pwm_duty(ATOM0_CH3_P21_5, duty_front); //�����ת
//    pwm_duty(ATOM0_CH0_P21_2, duty_behind);//�����ת
//������
    encoder_speed = gpt12_get(GPT12_T5);
    gpt12_clear(GPT12_T5);
//��·�̼ǲ�
    //car_stop_real_distance+=encoder_speed;
//�µ��ǲ�
//    if (ramp_flag==1)ramp_flag_real_distance+=encoder_speed;
//    else ramp_flag_real_distance=0;
//�������
    //���pidʵ���ٶ�
    MotorNowSpeed = (float)(encoder_speed*10);

    //�����ٶ��趨
    if      (OutgarageFlag == 0)  SpeedSet = 0;                            //�ȴ����⣨�ٶ�Ϊ�㣩
    else  if(OutgarageFlag == 1)  SpeedSet  = (float)OutgarageSpeed;       //���ڳ���(�����ٶ�)
    else
    {
        //������ʻ�ٶ�����
        SpeedSet=SpeedSet_Nnomal;

        //��ʼ���
        if     (ZebraCrossingFlag==0)ZebraDistance_Real=0;
        else if(ZebraCrossingFlag==1){SpeedSet=(float)ZebraSpeed;ZebraDistance_Real+=encoder_speed;}
        else SpeedSet=0;
        //�µ�����
        if       ( ramp_flag==1 )SpeedSet=SpeedSet_ramp[0];
        else  if ( ramp_flag==2 )SpeedSet=SpeedSet_ramp[1];
        else  if ( ramp_flag==3 )SpeedSet=SpeedSet_ramp[2];
        else  if ( ramp_flag==4 )SpeedSet=SpeedSet_ramp[3];
        else  if ( ramp_flag==5 )SpeedSet=SpeedSet_ramp[4];
        else  if ( ramp_flag==6 )SpeedSet=SpeedSet_ramp[5];
        //�����ٶȿ���
        if       ( Annulu_Flag==1 )SpeedSet=SpeedSet_Annulu[0]; //׼���뻷
        else if  ( Annulu_Flag==2 )SpeedSet=SpeedSet_Annulu[1]; //��ʼ�뻷
        else if  ( Annulu_Flag==3 )SpeedSet=SpeedSet_Annulu[2]; //���ڻ���
        else if  ( Annulu_Flag==4 || Annulu_Flag==5 || Annulu_Flag==6)SpeedSet=SpeedSet_Annulu[3]; //׼������//���ڳ�������һ�׶Σ�//���ڳ������ڶ��׶Σ�
        //�������������
        if(InductanceSAVE[2]<OutTrack && InductanceSAVE[1]<OutTrack)SpeedSet=0;
    }
    //��ʻ�ٶ��趨�������ٶ��趨��
    MotorExpSpeed = SpeedSet*10;


    //��·�̼ǲ�ͣ��
    if(car_stop_real_distance>car_stop_limit_distance){MotorExpSpeed=0;car_stop_reset=1;}
//    if(LeftlineDerivative<LeftlineDerivative_limit && RightlineDerivative>RightlineDerivative_limit)  SpeedSet=0;
    //�ٶ�ƫ��洢/��б��
    for(int i = 0; i <= LSDepth-2; i++)MotorErrArray[i] = MotorErrArray[i+1];
    MotorErrArray[9] = MotorNowSpeed - MotorExpSpeed;
    MotorErrDerivativeLast=MotorErrDerivative;
    MotorErrDerivative = Slope_Calculate(0, 8, MotorErrArray);
    //����pid�����ֵ
    //�洢
    MotorOutLast = MotorOut;
    //�����޷�
    MotorOutLast = MotorOutLast>6000?6000:MotorOutLast;
    MotorOutLast = MotorOutLast<-6000?-6000:MotorOutLast;
    //������pid����
    MotorOut = MotorOutLast-(int)(MotorI*MotorErrArray[9] + MotorK*(MotorErrArray[9]- MotorErrArray[8])
               - MotorD*(MotorErrDerivative-MotorErrDerivativeLast));
    //���Ƶ��
    if(MotorOut>=0)//����
    {
        MotorOut = MotorOut>10000?10000:MotorOut;//��ĸΪ10*10000��FTM.H��
        pwm_duty(ATOM0_CH3_P21_5, (int)MotorOut); //�����ת
        pwm_duty(ATOM0_CH0_P21_2, 0);//�����ת
    }
    else if(MotorOut<0)
    {
        MotorOut = MotorOut<-10000?-10000:MotorOut;//��ĸΪ10*10000��FTM.H��
        pwm_duty(ATOM0_CH3_P21_5, 0); //�����ת
        pwm_duty(ATOM0_CH0_P21_2, (int)(-MotorOut));//�����ת
    }
}
/***************************************************************************************
�� �� �� :void Signal_Init()
��    �� :��г�ʼ���������м䰴����ʼ�����
��    �� :��
�� �� ֵ :��
�������� :��
***************************************************************************************/
void Signal_Init()
{
    int i=0;
    //��ʱ200ms
    systick_delay_ms(STM0, 200);
    //��ʼ������ڴ�
    for(i=0;i<4;i++)InductanceGetlookMax[i]=0;
    //���ֵ���ԣ���̬���ԣ�
    while(1)
    {
        //��вɼ�(�ɼ�5��Ȼ�󷵻�ƽ��ֵ)
        InductanceGetlook[0]=adc_mean_filter(ADC_0, Diangan0, ADC_12BIT, 5);
        InductanceGetlook[1]=adc_mean_filter(ADC_0, Diangan1, ADC_12BIT, 5);
        InductanceGetlook[2]=adc_mean_filter(ADC_0, Diangan2, ADC_12BIT, 5);
        InductanceGetlook[3]=adc_mean_filter(ADC_0, Diangan3, ADC_12BIT, 5);
        //��ʾ���ֵ
        sprintf(ch1," 0:    %d    ",InductanceGetlook[0]);
        oled_p6x8str(1,0,ch1);
        sprintf(ch1," 1:    %d    ",InductanceGetlook[1]);
        oled_p6x8str(1,1,ch1);
        sprintf(ch1," 2:    %d    ",InductanceGetlook[2]);
        oled_p6x8str(1,2,ch1);
        sprintf(ch1," 3:    %d    ",InductanceGetlook[3]);
        oled_p6x8str(1,3,ch1);
        //�������
        InductanceGetlook[0] = 0;
        InductanceGetlook[1] = 0;
        InductanceGetlook[2] = 0;
        InductanceGetlook[3] = 0;
        //�г���г�ʼ��ģʽ
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
        int   compensate=5;                        //������Ʈ

        //��вɼ�(�ɼ�2��Ȼ�󷵻�ƽ��ֵ)
        InductanceGetlook[0]=adc_mean_filter(ADC_0, Diangan0, ADC_12BIT, 2);
        InductanceGetlook[1]=adc_mean_filter(ADC_0, Diangan1, ADC_12BIT, 2);
        InductanceGetlook[2]=adc_mean_filter(ADC_0, Diangan2, ADC_12BIT, 2);
        InductanceGetlook[3]=adc_mean_filter(ADC_0, Diangan3, ADC_12BIT, 2);
        //��õ�����ֵ
        if(InductanceGetlook[0] > InductanceGetlookMax[0])
          InductanceGetlookMax[0] = InductanceGetlook[0];
        if(InductanceGetlook[1] > InductanceGetlookMax[1])
          InductanceGetlookMax[1] = InductanceGetlook[1];
        if(InductanceGetlook[2] > InductanceGetlookMax[2])
          InductanceGetlookMax[2] = InductanceGetlook[2];
        if(InductanceGetlook[3] > InductanceGetlookMax[3])
          InductanceGetlookMax[3] = InductanceGetlook[3];
        //��ʾ������ֵ
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
        //�г�������ֵ�۲�ģʽ
        if(gpio_get(KeyTip)==0)
        {
            systick_delay_ms(STM0, 200);
            oled_fill(0x00);
            count_oled=0;
            break;
        }
        //�����ӳټ�����������
        if(gpio_get(KeyTip)==1 && gpio_get(KeyUp)==1 && gpio_get(KeyDown)==1 && gpio_get(KeyRight)==1 &&  gpio_get(KeyLeft)==1)
        {
           count_oled=0;
        }
    }

}
/***************************************************************************************
�� �� �� :void Inductance_GetDeal()
��    �� :����źŲɼ�������
��    �� :��
�� �� ֵ :��
�������� :��
***************************************************************************************/
void Inductance_GetDeal()
{
    InductanceGetlookMax[0]=4095;
    InductanceGetlookMax[1]=4095;
    InductanceGetlookMax[2]=4095;
    InductanceGetlookMax[3]=4095;
/*****************�����Ϣ����***************/
    //��вɼ�(�ɼ�5��Ȼ�󷵻�ƽ��ֵ)
    InductanceGetlook[0]=adc_mean_filter(ADC_0, Diangan0, ADC_12BIT, 5);
    InductanceGetlook[1]=adc_mean_filter(ADC_0, Diangan1, ADC_12BIT, 5);
    InductanceGetlook[2]=adc_mean_filter(ADC_0, Diangan2, ADC_12BIT, 5);
    InductanceGetlook[3]=adc_mean_filter(ADC_0, Diangan3, ADC_12BIT, 5);
    //�洢
    InductanceSAVE[0] = InductanceGetlook[0];
    InductanceSAVE[1] = InductanceGetlook[1];
    InductanceSAVE[2] = InductanceGetlook[2];
    InductanceSAVE[3] = InductanceGetlook[3];
    //����
    InductanceGetlook[0] = 0;
    InductanceGetlook[1] = 0;
    InductanceGetlook[2] = 0;
    InductanceGetlook[3] = 0;
    //��һ��
    InductanceGet[0] = (int)((1.0 * InductanceSAVE[0] / InductanceGetlookMax[0]) * 1000);
    InductanceGet[1] = (int)((1.0 * InductanceSAVE[1] / InductanceGetlookMax[1]) * 1000);
    InductanceGet[2] = (int)((1.0 * InductanceSAVE[2] / InductanceGetlookMax[2]) * 1000);
    InductanceGet[3] = (int)((1.0 * InductanceSAVE[3] / InductanceGetlookMax[3]) * 1000);
    //�޷�
    if(InductanceGet[0] > 1000)InductanceGet[0] = 1000;
    if(InductanceGet[1] > 1000)InductanceGet[1] = 1000;
    if(InductanceGet[2] > 1000)InductanceGet[2] = 1000;
    if(InductanceGet[3] > 1000)InductanceGet[3] = 1000;
    //��һ��ֹ���ֳ����������Ϊ������
    InductanceGet[0] += 1;
    InductanceGet[1] += 1;
    InductanceGet[2] += 1;
    InductanceGet[3] += 1;
/*****************��仯��***************/
    //���
    //��������ݸ���
    for(int i = 0; i <= LSDepth-2; i++)InductanceGetStoreLeft[i] = InductanceGetStoreLeft[i+1];
    InductanceGetStoreLeft[9] = InductanceSAVE[0];
    //��˵�б仯�ʣ���С���˷���
    InductanceLeftDerivative = Slope_Calculate(0, 8, InductanceGetStoreLeft);
    //�Ҳ�
    //�Ҳ������ݸ���
    for(int i = 0; i <= LSDepth-2; i++)InductanceGetStoreRight[i] = InductanceGetStoreRight[i+1];
    InductanceGetStoreRight[9] = InductanceSAVE[1];
    //�Ҷ˵�б仯�ʣ���С���˷���
    InductanceRightDerivative = Slope_Calculate(0, 8, InductanceGetStoreRight);
    //����
    //���˵�����ݸ���
    for(int i = 0; i <= LSDepth-2; i++)InductanceGetStoreNear[i] = InductanceGetStoreNear[i+1];
    InductanceGetStoreNear[9] = InductanceSAVE[2];
    //���˵�б仯�ʣ���С���˷���
    InductanceNearDerivative = Slope_Calculate(0, 8, InductanceGetStoreNear);
    //Զ��
    //Զ�˵�����ݸ���
    for(int i = 0; i <= LSDepth-2; i++)InductanceGetStoreFar[i] = InductanceGetStoreFar[i+1];
    InductanceGetStoreFar[9] = InductanceSAVE[3];
    //Զ�˵�б仯�ʣ���С���˷���
    InductanceFarDerivative = Slope_Calculate(0, 8, InductanceGetStoreFar);
/*****************�õ��ƫ��***************/
    //��Ȼ����ƫ����
    InductanceDeal[0] = (1.0*(InductanceGet[2] - InductanceGet[1])) / (1.0*(InductanceGet[2] * InductanceGet[1])) * 5000;
    //����
    if(InductanceDeal[0] <= 0.1 && InductanceDeal[0] >= -0.1) InductanceUse[0] = 0;
    else InductanceUse[0] = InductanceDeal[0];
    //�޷�
    if(InductanceUse[0] > 35)InductanceUse[0] = 35;
    if(InductanceUse[0] < -35)InductanceUse[0] = -35;

    //�洢��һ������ƫ��
    InductanceUseLast=InductanceUseRecult;
    //�ô˴�����ƫ��
    InductanceUseRecult = InductanceUse[0];
    //һ���ͺ��˲�
    InductanceUseRecult = InductanceUseWi*InductanceUseRecult + (1-InductanceUseWi)*InductanceUseLast;
}
/***************************************************************************************
�� �� �� :float Slope_Calculate(uint8 begin,uint8 end,float *p)
//��    �� :��С���˷����б��
��    �� :��
�� �� ֵ :��
�������� :��
����:       //for(int i = 0; i <= LSDepth-2; i++)
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
    if((end-begin)*x2sum-xsum*xsum) //�жϳ����Ƿ�Ϊ��
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
�� �� �� :void Buzzing_Remind(void)
��    �� :��������ʾ
��    �� :��
�� �� ֵ :��
�������� :��
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
