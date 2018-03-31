/******************��Ⱥ� ���˫�� �����㷨********************/
#include "common.h"
#include "include.h"
//v=14,p=21,d=90

#define SECTOR_NUM  (FLASH_SECTOR_NUM-1)

#define Go              !gpio_get (PTA17) //������ȡ
#define Change_page     !gpio_get (PTA7)
#define Add_line        !gpio_get (PTA14)
#define Sub_line        !gpio_get (PTA15)
#define Add             !gpio_get (PTA16)
#define Sub             !gpio_get (PTA13)

#define singel_car       gpio_get (PTA12) //������־             0Ϊ˫�� 1Ϊ����
#define S_HD_L_R         gpio_get (PTE27) //ǰ��С������ת����ת 0Ϊ��ת 1Ϊ��ת
#define B_HD_L_R         gpio_get (PTE28) //ǰ���󻷵���ת����ת 0Ϊ��ת 1Ϊ��ת


#define     Step_Middle      5100//�����ֵ
#define     Step_Right       5900//����ֵ
#define     Step_Left        4200//����ֵ
/****************��������********************/
//uint16 SpeedSet=20;//pwm=20->v=3.3152
//uint8 Speed_P=6,Speed_I=10;
//#define total 12820
//int16 vn;
//int16 vm;
//int16 motor_pwm;
//float v;
//float SpeedData[4]={0};
//float g_fCarSpeed,g_fSpeedControlOutOld,g_fSpeedControlOutNew;
//float g_fSpeedControlIntegral,g_fSpeedControlOut;
//float fDelta,fP;
//float Speedlimit=200;//�ٶȱջ�����

int mode1;
int mode2;
int mode3;
int mode4;

int stop_direction=0;
int DirectionControlCount;
float sduty=0;
int32 steel_PWM=0;
//float ADLeft[8];
//float ADRight[8];
float DirectionADLeft[4];
float DirectionADRight[4];
float DirectionADLeftH[4];
float DirectionADRightH[4];
float DirectionADMiddle_L[4];
float DirectionADMiddle_R[4];
float Direction_Error[20];
float DirectionData[6]={0};
float SubAddData[6]={0};
//float SpeedData[4]={0};
float   SubAdd;
float   Left,Right;
float   Middle_L,Middle_L_1,Middle_R,Middle_R_1;
float   LeftH,RightH,LeftH_1,RightH_1;
float   nLeft,nRight,nLeft_1,nRight_1;
float   g_fDirection_Error;
float   g_fDirectionControlOutOld,g_fDirectionControlOutNew,g_fDirectionControlOut;
int     start_car=0;//������־
int     Clk_flag=0;//������flag
int clk=0;
int     s_flag=0;//ֱ��flag
int16   s_count=0;//ֱ��count
int     yh_cnt=0;//Բ����ǳ�������
int16   cd_cnt=0;//����count
int     cd_flag=0;//����flag
uint16  speed;//�ٶ�
int     zhi_to_wan_flag=0;//ֱ����flag
int     zhi_to_wan_cnt=0;//ֱ����count
int     stop=0;//ͣ����־
int     stop_cnt=0;

bool HD_straight_flag=0;//����ֱ����־
int  HD_straight_cnt=0;//����ֱ������

int    B_HD_flag=0;//�󻷵���־
int16  B_HD_cnt=0;//�󻷵�����
int    S_HD_end=0;//С����������־

int Dre_B=0;//�󻷵�����
int Dre_S=0;//С��������

int HD_num=0;//Բ������

float batter_power=0.0;
/****************��������********************/

uint16 Direction_P_AS=20,Direction_D_AS=50;
uint16 Direction_P_AS1=0;
float P,D;
uint16 Step_Middle1=5100;

/***************˫���źŷ���*******************/
uint16 Car;//0Ϊǰ����1Ϊ��

#define B_GO            0xF0//�󳵷�������     240
#define go_overtake_ask 0xF8//���ܳ����ʴ�     248
#define HD_OVERTAKE_ASK 0xF2//���������ʴ�     242
#define sz_overtake_ask 0xF4//ʮ�ֳ����ʴ�     244
#define go_over         0xFB//���ܳ�������     250
#define HD_OVER         0xF6//������������     246
#define sz_over         0xF9//ʮ�ֳ�������     249
#define STOP            0xFC//ͣ����־         252
#define END             0xFF//������־         255

bool go=0;//����
bool end=0;//ͣ��
bool front_or_back;//0Ϊǰ����1Ϊ��

bool HD_overtake=0;//����������־
bool HD_overtake_f=0;//����������ɱ�־

bool go_overtake=0;//���ܳ�����־
bool go_overtake_f=0;//���ܳ���������־

bool SZ_overtake=0;//��ʮ�ֳ�����־
bool SZ_overtake_f=0;//��ʮ�ֳ���������־

bool B_HD_can=0;//�󻷵�������
bool S_HD_can=0;//С����������

bool PD_flag;//�µ���־
bool Last_PD_flag;
int PD_num;

float PD_distance;

float HD_distance;//��������
float distance;//�ܾ���
float flag_distance;//С���������¼
float distance_error;//����ƫ��
float B_HD_distance=0;//�󻷵������־


uint32 ultrasonic_time=0;//������ʱ��
uint32 ABDistance;//�����ķ��ͽ���ģ��ľ���,��λ����
uint32 re_ABDistance;//��һ�εĳ��������
/****************����ֵ����********************/
float ABS_FLOAT(float data)
{
float temp;
if(data<0) temp=-data;
else temp=data;
return temp;
}

/******************************���������ʼ��***********************/
void HD_Direction_init(void)
{
  if(S_HD_L_R)Dre_S=1;
  else Dre_S=-1;
  
  if(B_HD_L_R)Dre_B=1;
  else Dre_B=-1;
}

/****************AD����********************/
void AD_init()
{
  adc_init(ADC1_SE8);//PTB0
  adc_init(ADC1_SE9);//PTB1
  adc_init(ADC1_SE10);//PTB4
  adc_init(ADC1_SE11);//PTB5
  adc_init(ADC1_SE12);//PTB6
  adc_init(ADC1_SE13);//PTB7
  adc_init(ADC1_SE14);//PTB10
}

uint16 adc_ave(ADCn_Ch_e adcn_ch, ADC_nbit bit,uint16 N)//��β�����ƽ��
{
  uint16 temp=0;
  uint8 i;
  for(i=0;i<N;i++)
  {
    temp+=adc_once(adcn_ch,bit);
  }
  temp=temp/N;
  return temp;
}


/********************�ջ����********************/
uint16 Speed;
uint16 v_speed=0;
uint16 SpeedSet=10;//pwm=20->v=3.3152
uint16 Speed_wan_Set=10;
int g_fSpeedControlIntegral_limit=200;
int16 Speed_P=65,Speed_I=20;

#define total 7100//1m,0.0001408m/pulse
int16 vn;
int16 vm;
float v;
float SpeedData[4]={0};
float g_fCarSpeed,g_fSpeedControlOutOld,g_fSpeedControlOutNew;
float g_fSpeedControlIntegral,g_fSpeedControlOut;
float fDelta,fP;

float Speedlimit=700;

void SpeedControl()
{
  int i;
  vn=ftm_quad_get(FTM2);
  distance+=(vn/7.1)/1000;
  v=(float)((vn*100.0)/total)*10;
  ftm_quad_clean(FTM2);
  g_fCarSpeed=vn;
    
  for(i=0;i<3;i++)
  {
    SpeedData[i]=SpeedData[i+1];
  }
  SpeedData[3]=g_fCarSpeed;
  g_fCarSpeed=0;
  for(i=0;i<4;i++)
  {
    g_fCarSpeed+=SpeedData[i]*(i+1)/10;
  }
  
  fDelta=Speed-g_fCarSpeed;
  
  fP=fDelta*Speed_P;
  
  g_fSpeedControlIntegral+=fDelta*(Speed_I/10.0);//���ַ���
  if(g_fSpeedControlIntegral<=-g_fSpeedControlIntegral_limit) g_fSpeedControlIntegral=-g_fSpeedControlIntegral_limit;//�����޷�
  if(g_fSpeedControlIntegral>=g_fSpeedControlIntegral_limit)  g_fSpeedControlIntegral=g_fSpeedControlIntegral_limit;

  g_fSpeedControlOutOld=g_fSpeedControlOutNew;
  g_fSpeedControlOutNew=fP+g_fSpeedControlIntegral;

  if(g_fSpeedControlOutNew>=Speedlimit)  g_fSpeedControlOutNew=Speedlimit;//������޷�
  if(g_fSpeedControlOutNew<=-Speedlimit)  g_fSpeedControlOutNew=-Speedlimit;
}

float motor_data[4];
int16 motor_pwm;
void Motor_Ctrl()
{
  if(start_car==1)//����
  {
    //DELAY_MS(1500);
//    gpio_set(PTD15,0);
    float temp;
        
    temp=(int16)g_fSpeedControlOutNew;
    motor_pwm=temp/10;
    if(motor_pwm>=0)
    {
      ftm_pwm_duty(FTM1, FTM_CH0,0);
      ftm_pwm_duty(FTM1, FTM_CH1,motor_pwm);
    }
    else
    {
      ftm_pwm_duty(FTM1, FTM_CH0,abs(motor_pwm));
      ftm_pwm_duty(FTM1, FTM_CH1,0);
    }

  }
}

/*************************������***********************/
void fengming()
{
  if(Clk_flag||clk)
  {
    gpio_set(PTC11,1);
  }
  else
    gpio_set(PTC11,0);
}

/****************OLED��ʼ��*****************/
void oled_init()
{
   gpio_init(PTE0,GPO,1);
   gpio_init(PTE1,GPO,1);
   gpio_init(PTE2,GPO,1);
   gpio_init(PTE3,GPO,1);
//   gpio_init(PTC16,GPO,1);
//   gpio_init(PTC17,GPO,1);
//   gpio_init(PTC18,GPO,1);
//   gpio_init(PTC19,GPO,1);
   LCD_Init();
}

/*****************���*���PWM��ʼ��*******************/
void motorPWM_init()
{
   ftm_pwm_init(FTM1, FTM_CH0, 10*1500,0);//PTA8
   ftm_pwm_init(FTM1, FTM_CH1, 10*1500,0);//PTA9
}
void steelPWM_init()
{
  ftm_pwm_init(FTM0,FTM_CH7,300,Step_Middle1);
}

/*****************�����������ʼ��***************************/
void key_init1()
{ 
     port_init (PTA13, ALT1 | PULLUP );//��
     port_init (PTA14, ALT1 | PULLUP );//��
     port_init (PTA15, ALT1 | PULLUP );//��
     port_init (PTA16, ALT1 | PULLUP );//��
     port_init (PTA7,  ALT1 | PULLUP ); //����
     port_init (PTA17, ALT1 | PULLUP ); //����  
//     gpio_init(PTA13,GPI,1);  
//     gpio_init(PTA14,GPI,1);
//     gpio_init(PTA15,GPI,1);
//     gpio_init(PTA16,GPI,1);
//     gpio_init(PTA7, GPI,1);
//     gpio_init(PTA17,GPI,1);  
}

void boma()
{
  port_init (PTE26, ALT1 | PULLUP );//4
  port_init (PTE28, ALT1 | PULLUP );//3
  port_init (PTE27, ALT1 | PULLUP );//2
  port_init (PTA12, ALT1 | PULLUP );//1
//  gpio_init(PTE26,GPI,1);
//  gpio_init(PTE28,GPI,1);
//  gpio_init(PTE27,GPI,1);
//  gpio_init(PTA12,GPI,1);
}


/*****************I/O�ڳ�ʼ��***********************************/
void io_init()
{
  gpio_init(PTD15,GPO,1);//LED��ʼ��
  gpio_init(PTC0,GPO,1);
  gpio_init(PTC11,GPO,0);//�������˿ڳ�ʼ��
  
}

/************************�������*******************************/
float left11,right11;
int stop_flag=0;
void DirectionAverageCalculate()
{
  int i;
  int j=0;
  uint32 Data_value=0;  
    
  for(i=0;i<3;i++){DirectionADLeft[i]=DirectionADLeft[i+1];}         
  for(j=0;j<5;j++)
  {
    Data_value+=adc_once(ADC1_SE8, ADC_12bit);//��һ//PTB0
  }   
  DirectionADLeft[3]=Data_value/5;
  Data_value=0;

  for(i=0;i<3;i++){DirectionADRight[i]=DirectionADRight[i+1];}
  for(j=0;j<5;j++)
  {
    Data_value+=adc_once(ADC1_SE9, ADC_12bit);//��һ//PTB1
  } 
  DirectionADRight[3]=Data_value/5;
  Data_value=0;
  
  for(i=0;i<3;i++){DirectionADLeftH[i]=DirectionADLeftH[i+1];}
  for(j=0;j<5;j++)
  {
    Data_value+=adc_once(ADC1_SE10, ADC_12bit);//���//PTB7
  } 
  DirectionADLeftH[3]=Data_value/5;
  Data_value=0;
  
  for(i=0;i<3;i++){DirectionADRightH[i]=DirectionADRightH[i+1];}
  for(j=0;j<5;j++)
  {
    Data_value+=adc_once(ADC1_SE11, ADC_12bit);//�Ҷ�//PTB10
  } 
  DirectionADRightH[3]=Data_value/5;
  Data_value=0;

  for(i=0;i<3;i++){DirectionADMiddle_L[i]=DirectionADMiddle_L[i+1];}
  for(j=0;j<5;j++)
  {
    Data_value+=adc_once(ADC1_SE12, ADC_12bit);//��һ//PTB4
  } 
  DirectionADMiddle_L[3]=Data_value/5;
  Data_value=0;  
  
}

float SAsqrt;
float SubAdd_Q,SubAdd_H,SubAdd1,Last_SubAdd_Q,Last_SubAdd_H;
float Last_Error=0;
void DirectionControl()
{
  float Error,temp_error;
  int i;
  
  for(i=0;i<4;i++){nLeft+=DirectionADLeft[i]*(i+1)/10;}//Ȩ��0.1��0.8
  for(i=0;i<4;i++){nRight+=DirectionADRight[i]*(i+1)/10;}
  
  for(i=0;i<4;i++){LeftH+=DirectionADLeftH[i]*(i+1)/10;}
  for(i=0;i<4;i++){RightH+=DirectionADRightH[i]*(i+1)/10;}
  
  for(i=0;i<4;i++){Middle_L+=DirectionADMiddle_L[i]*(i+1)/10;}

  nLeft_1=nLeft; //��
  nRight_1=nRight;//��
  
  LeftH_1=LeftH;//����
  RightH_1=RightH;
  
  Middle_L_1=Middle_L;
  
  if(Middle_L_1>=2900&&ABS_FLOAT(LeftH_1-RightH_1)<800&&ABS_FLOAT(nLeft_1-nRight_1)<1000&&RightH_1>1500&&LeftH_1>1500)
  {
    PD_flag=1;
    PD_distance=distance;
  }
  Last_PD_flag=PD_flag;
  if(distance-PD_distance>3&&PD_distance!=0)PD_flag=0;
  
  if(Last_PD_flag==1&&PD_flag==0)
  {
    PD_num++;
    if(singel_car==0&&PD_num==1)//��ʮ��3m ���� 
    {
      go_overtake=1;//ֱ��������־��λ 
    }
  }
    
  if(PD_flag==1)clk=1;
  else clk=0;
   
  if(PD_flag==0)
  {
    if(singel_car==0)//˫��  
    {
      if(front_or_back==0)//ǰ��  
      {
        if(S_HD_end==0)
        {
          if(Middle_L_1<1100&&ABS_FLOAT(LeftH_1-RightH_1)<1100&&RightH_1>1400&&LeftH_1>1400
           &&ABS_FLOAT(nLeft_1-nRight_1)<600)                                 //СԲ���ж�
          {
            Clk_flag=1;
            S_HD_can=1;//СԲ����������־
          }
          else
          {
            Clk_flag=0;
          }
        }
        else
        {
          if(Middle_L_1<1000&&ABS_FLOAT(LeftH_1-RightH_1)<1000&&RightH_1>1300&&LeftH_1>1300
           &&ABS_FLOAT(nLeft_1-nRight_1)<600)                                 //��Բ���ж�
          {
            B_HD_flag=1;//������־
            B_HD_can=1;//��Բ����������־  �����СԲ����־λ��ͻ
            S_HD_can=0;//СԲ����������־����
          }
          else
            B_HD_flag=0;
        }
      }
      else//houche
      {
        if(S_HD_end==0)
        {
          if(Middle_L_1<1100&&ABS_FLOAT(LeftH_1-RightH_1)<1100&&RightH_1>1400&&LeftH_1>1400
           &&ABS_FLOAT(nLeft_1-nRight_1)<600)                                 //СԲ���ж�
          {
            Clk_flag=1;
          }
          else
          {
            Clk_flag=0;
          }
        }
        else
        {
          if(Middle_L_1<1000&&ABS_FLOAT(LeftH_1-RightH_1)<1000&&RightH_1>1300&&LeftH_1>1300
           &&ABS_FLOAT(nLeft_1-nRight_1)<600)                                 //��Բ���ж�
          {
            B_HD_flag=1;//������־
          }
          else
            B_HD_flag=0;
        }
      }
    }
    else//danche
    {
      if(S_HD_end==0)
      {
        if(Middle_L_1<1100&&ABS_FLOAT(LeftH_1-RightH_1)<1100&&RightH_1>1400&&LeftH_1>1400
           &&ABS_FLOAT(nLeft_1-nRight_1)<600)                                 //СԲ���ж�
        {
          Clk_flag=1;
        }
        else
        {
          Clk_flag=0;
        }
      }
      else
      {
        if(Middle_L_1<1000&&ABS_FLOAT(LeftH_1-RightH_1)<1000&&RightH_1>1200&&LeftH_1>1200
           &&ABS_FLOAT(nLeft_1-nRight_1)<600)                                 //��Բ���ж�
        {
          B_HD_flag=1;//������־
        }
        else
          B_HD_flag=0;
      }
    } 
  }

  SubAdd_Q=(nRight_1-nLeft_1)/(nLeft_1+nRight_1);//��Ⱥ�
  SubAdd_H=(RightH_1-LeftH_1)/(LeftH_1+RightH_1);
  Last_SubAdd_Q=SubAdd_Q;
  Last_SubAdd_H=SubAdd_H;
  
  SubAdd=(((SubAdd_Q*8)/10)+((SubAdd_H*2)/10))*40;
//  SubAdd=SubAdd_Q*40;
//  SubAdd=SubAdd_H*40;
  
  if(singel_car==0)//ֱ��������Ⱥʹ���
  {
    if(go_overtake==1)
    {
      if(front_or_back==0)
      {
        SubAdd=SubAdd-28;//�����������
      }
      else
      {
        SubAdd=SubAdd+28;//�����ұ�����
      }
    }
  }
  
  
//  temp_error=ABS(SubAdd);
         
//         if(temp_error<=15&&temp_error>=0)
//           P=-0.0009172*(temp_error*temp_error*temp_error*temp_error)
//             +0.02462*(temp_error*temp_error*temp_error)// error=5,p=4,10
//                  -0.1647*(temp_error*temp_error)
//                  +0.3771*(temp_error)
//                   +12.752;
//         else P=20;
         
       
  for(i=0;i<5;i++)//������ʷֵ
  {
    SubAddData[i]=SubAddData[i+1];
  }
  SubAddData[5]=SubAdd;
  
  
//  if(nLeft_1>2000&&nRight_1>2000&&LeftH_1>2500&&RightH_1>2500)//ֱ�����
//  {
//    HD_straight_cnt;
//    if(HD_straight_cnt>=2)
//    {
//      HD_straight_cnt=2;
//      HD_straight_flag=1;
//      //clk=1;
//    }
//    else HD_straight_flag=0;
//  }
//  else HD_straight_cnt=0;
  
  if(ABS_FLOAT(SubAdd_H)*40<5&&ABS_FLOAT(SubAdd_Q)*40>20)//ֱ������
  {
    zhi_to_wan_flag=1;
  }
  else
  {
    zhi_to_wan_flag=0;
  }

  if(nLeft_1<25&&nRight_1<25&&LeftH_1<25&&RightH_1<25)//�������ͣ����־
  {
    stop_cnt+=1;
    if(stop_cnt>=5)
    {
      stop_cnt=5;
      stop=1;
    }
  }
  
  Error=SubAddData[5]-SubAddData[0];                   //����ƫ��
  Last_Error=Error;
  
  nLeft=0;
  nRight=0;
  LeftH=0;
  RightH=0;
  Middle_L=0;
  Middle_R=0;
             
}

/**************************�������*****************************/
void Steel_Contorl()
{
//  if(ABS(SubAdd)<=10)
//  {
//     Direction_P_AS1=Direction_P_AS+0.4*ABS(SubAdd);
//     sduty=Direction_P_AS1*SubAdd+Direction_D_AS*Last_Error;
//  }
//  else 
//  {
//    Direction_P_AS1=Direction_P_AS+0.4*10;
//    sduty=Direction_P_AS1*SubAdd+Direction_D_AS*Last_Error;
//  }
  
  P=Direction_P_AS+(ABS_FLOAT(SubAdd)/8.0);//��̬P
  
//  if(ABS_FLOAT(SubAdd)<5)  P=5;  //�ֶ�P
//  else if(ABS_FLOAT(SubAdd)<20)  P=16;
//  else if(ABS_FLOAT(SubAdd)<30)  P=18;
//  else P=20;
  
//  P=Direction_P_AS;
  
  if(ABS_FLOAT(SubAdd_Q)*40<5&&ABS_FLOAT(SubAdd_H)*40<5&&LeftH_1>2000&&RightH_1>2000)//ֱ��СP D
  {
    P=3;
    Direction_D_AS=50;
  }
  
//  if(PD_num<=1&&PD_flag==1&&distance-PD_distance<3&&distance-PD_distance>0.7)//ʮ�ִ���
//  {
//    P=25;
//    D=250;
//  }
  
  //sduty=Direction_P_AS*SubAdd+Direction_D_AS*Last_Error;
  
if(singel_car==0)//˫��Բ������
{
  if(front_or_back==0)//ǰ��
  {
    if(Clk_flag==1)//СԲ������
    {
      flag_distance=distance;//СԲ�������¼
      Clk_flag=0;//Բ����־����   
    }
    if(flag_distance!=0&&distance-flag_distance<0.1)//���10cm
    {
      sduty=Dre_S*800;
    }
    else sduty=P*SubAdd+Direction_D_AS*Last_Error;
  
    if(flag_distance!=0&&distance-flag_distance<0.3)//����30cm
    {
      Speed=5;
    }
    else Speed=SpeedSet;
    if(flag_distance!=0&&distance-flag_distance>1)//1m��СԲ��������־
    {
      S_HD_end=1;
    }
  
    if(S_HD_end==1)//СԲ������
    {
      if(B_HD_flag==1)//�󻷵�����
      {
        B_HD_distance=distance;//��Բ�������¼
      }
      if(B_HD_distance!=0&&distance-B_HD_distance<0.1)//���10cm
      {
        sduty=Dre_B*800;
      }
      else sduty=P*SubAdd+Direction_D_AS*Last_Error;
  
      if(B_HD_distance!=0&&distance-B_HD_distance<0.3)//����30cm
      {
        Speed=5;
      }
      else Speed=SpeedSet;
    }
  }
  else//��
  {
    if(Clk_flag==1)//СԲ������
    {
      flag_distance=distance;//СԲ�������¼
      Clk_flag=0;//Բ����־����   
    }
    if(flag_distance!=0&&distance-flag_distance<0.1)//���10cm
    {
      sduty=Dre_S*(-800);
    }
    else sduty=P*SubAdd+Direction_D_AS*Last_Error;
  
    if(flag_distance!=0&&distance-flag_distance<0.3)//����50cm
    {
      Speed=5;
    }
    else Speed=SpeedSet;
    if(flag_distance!=0&&distance-flag_distance>1)//1m��СԲ��������־
    {
      S_HD_end=1;
    }
  
    if(S_HD_end==1)//СԲ������
    {
      if(B_HD_flag==1)//�󻷵�����
      {
        B_HD_distance=distance;//��Բ�������¼
      }
      if(B_HD_distance!=0&&distance-B_HD_distance<0.05)//���10cm
      {
        sduty=Dre_B*(-800);
      }
      else sduty=P*SubAdd+Direction_D_AS*Last_Error;
  
      if(B_HD_distance!=0&&distance-B_HD_distance<0.3)//����80cm
      {
        Speed=5;
      }
      else Speed=SpeedSet;
    }    
  }
}
else//����Բ������
{
  if(Clk_flag==1)//СԲ������
  {
    flag_distance=distance;//СԲ�������¼
    Clk_flag=0;//Բ����־����    
  }
  if(flag_distance!=0&&distance-flag_distance<0.1)//���10cm
  {
    sduty=Dre_S*800;
  }
  else sduty=P*SubAdd+Direction_D_AS*Last_Error;
  
  if(flag_distance!=0&&distance-flag_distance<0.3)//����50cm
  {
    Speed=5;
  }
  else Speed=SpeedSet;
  if(flag_distance!=0&&distance-flag_distance>1)//1m��СԲ��������־
  {
    S_HD_end=1;
  }
  
  if(S_HD_end==1)//СԲ������
  {
    if(B_HD_flag==1)//�󻷵�����
    {
      B_HD_distance=distance;//��Բ�������¼
    }
    if(B_HD_distance!=0&&distance-B_HD_distance<0.1)//���10cm
    {
      sduty=Dre_B*800;
    }
    else sduty=P*SubAdd+Direction_D_AS*Last_Error;
  
    if(B_HD_distance!=0&&distance-B_HD_distance<0.3)//����80cm
    {
      Speed=4;
    }
    else if(B_HD_distance!=0&&distance-B_HD_distance<0.5)
    {
      Speed=8;
    }
    else Speed=SpeedSet;
  }
}

  if(PD_num<1&&PD_flag==1&&distance-PD_distance<3)//ʮ�ִ���
    Speed=12;

  if(PD_num<2&&PD_flag==1&&distance-PD_distance<1)
    Speed=18;

  if(zhi_to_wan_flag==1)Speed=Speed_wan_Set;            //�������
  
  if(sduty>=2000)   sduty=2000;                           //�޷�
  if(sduty<=-2000)  sduty=-2000;
  steel_PWM=(int)sduty+Step_Middle1;
  ftm_pwm_duty(FTM0,FTM_CH7,steel_PWM);

}

/*************************************�������***********************/
void motor_control()
{
  if(singel_car==0)//������ ���ٶȿ���
  {
    if(front_or_back==1)
    {
//      if((ABDistance!=re_ABDistance)&&(ABDistance!=288))//����⵽�������ź�ʱ���ٶȿ���
//      {
        if(ABDistance>60)
        {
          Speed=Speed+1;
        }
        else if(ABDistance<20&&ABDistance!=0)
        {
          Speed=0;
        }
        else if(ABDistance<30&&ABDistance!=0)
        {
          Speed=Speed-11;
        }
        else if(ABDistance<35&&ABDistance!=0)
        {
          Speed=Speed-9;
        }
        else if(ABDistance<40&&ABDistance!=0)
        {
          Speed=Speed-6;
        }
        else if(ABDistance<45&&ABDistance!=0)
        {
          Speed=Speed-3;
        }
        else if(ABDistance<50&&ABDistance!=0)
        {
          Speed=Speed-1;
        }
        
        else if(ABDistance>65)
        {
          Speed=Speed+3;
        }
        else if(ABDistance>75)
        {
          Speed=Speed+5;
        }
        else if(ABDistance>85)
        {
          Speed=Speed+7;
        }
//      }
        
        if(go_overtake==1)//���ܳ���ʱ���ٶȿ���
        {
          Speed=10;
        }
      
    }
    else//ǰ���ٶȿ���
    {
        if(go_overtake==1&&distance>=0.3&&distance<=2)//���ܳ����ٶȿ���  30cmͣ��
        {
          Speed=0;
        }
        
        if(go_overtake==1&&(distance-PD_distance>0.8)&&PD_distance!=0)//��ʮ�ֳ����ٶȿ���  50cmͣ��
        {
          Speed=0;
        }
            
//      if(S_HD_can)
//      {
        if(HD_overtake==1&&(distance-flag_distance>1.5)&&flag_distance!=0)  //С�����ٶȿ���
        {
          Speed=4;
        }
        
        if(HD_overtake==1&&(distance-flag_distance>0.5)&&(distance-flag_distance<1.3)&&flag_distance!=0)
        {
          Speed=18;
        }
//      }
      
//      if(B_HD_can)
//      {
        
        if(HD_overtake==1&&(distance-B_HD_distance>3)&&B_HD_distance!=0)//�󻷵��ٶȿ���
        {
          Speed=4;
        }
        
        if(HD_overtake==1&&(distance-B_HD_distance>0.5)&&(distance-B_HD_distance<2.8)&&B_HD_distance!=0)
        {
          Speed=18;
        }
        
//      }
    }
    
  }
  
  
  if(Speed>20)Speed=20;//�ٶ��޷�
  if(Speed<0) Speed=0;
  
  if(stop==1)
  {
    end=1;//ͣ������
    //Clk_flag=1;
  }
}

/************************************������־����***********************************/
void overtake()
{
  if(front_or_back==0)//ǰ��������־����
  {
    if(go_overtake==1&&(ABDistance!=re_ABDistance)&&(ABDistance!=288)&&(ABDistance<300))
    {
      go_overtake_f=1;//���ܳ������
      go_overtake=0;//������־����
      //clk=1;
    }
    
    if(B_HD_can||S_HD_can)
    {
      B_HD_can=0;
      S_HD_can=0;
      HD_overtake=1;//����������־
      HD_overtake_f=0;//������ɱ�־����
      //clk=1;
    }
    //else clk=0;    
    
    if(HD_overtake==1&&(ABDistance!=re_ABDistance)&&(ABDistance!=288)&&(ABDistance<300))
    {
      HD_overtake_f=1;//�������
      HD_overtake=0;//����������־����
      S_HD_can=0;
      //clk=1;
    }
    

    
    re_ABDistance=ABDistance;//��һ�γ��������ڱ��γ�����
    
  }
 // else//�󳵳�����־����
//  {
//    if(HD_overtake==1&&HD_straight_flag==1&&(((distance-B_HD_distance)>0.1&&B_HD_distance!=0)||((distance-flag_distance)>0.1&&flag_distance!=0)))
//    {
//      HD_overtake_f=1;//�������
//      clk=1;
//      
//    }

//  }
  
}


/***********************************�������*****************************************/
//void Motor_Ctrl()
//{
////  float temp;
////  temp=(int16)g_fSpeedControlOutNew;
////  motor_pwm=temp/4;
//  if(start_car==1)
//  {
//    gpio_set(PTD15,0);
//    ftm_pwm_duty(FTM1, FTM_CH0,0);
//    ftm_pwm_duty(FTM1, FTM_CH1,speed);
//    
//    if(Clk_flag==1)
//    {
//      ftm_pwm_duty(FTM1, FTM_CH0,0);
//      ftm_pwm_duty(FTM1, FTM_CH1,0);
//    }
////    if(motor_pwm>=0)
////    {
////      ftm_pwm_duty(FTM1, FTM_CH0,motor_pwm);
////      ftm_pwm_duty(FTM1, FTM_CH1,0);
////    }
////    else
////    {
////      ftm_pwm_duty(FTM1, FTM_CH0,0);
////      ftm_pwm_duty(FTM1, FTM_CH1,abs(motor_pwm));
////    }
//  }
//}

/*************************************ǰ�󳵳�ʼ��************************************/
void AB_Init()
{
  go=0;
  if(Car==0)
  {
  front_or_back=0;
  }
  if(Car==1)
  {
  front_or_back=1;
  }
}

/**************************************��������**************************************/
void parameter_write();
void pre_show1();
void pre_show2();
void pre_show3();
int line=0;
int page=1;

void KeyScan1(void)
{
  if(Go==1)//����
  {
    DELAY_MS(30);//����
    if(Go==1)
    {
      while(Go){};//û�ɿ����ȴ�
      
      start_car=1;
      
    }   
  }
  
  if(Change_page==1)//��ҳ
  {
    DELAY_MS(30);//����
    if(Change_page==1)
    {
      while(Change_page){};
      page+=1;
      if(page>3)page=1;
    }
  }
  
  if(page==1)pre_show1();
  if(page==2)
  {    
    if(Sub_line)//����
    {
      DELAY_MS(30);//����
      if(Sub_line)
      {
        while(Sub_line);
        line++;
        if(line>7)line=0;
      }     
    }
    
    if(Add_line)//����
    {
      DELAY_MS(30);//����
      if(Add_line)
      {
        while(Add_line);
        line--;
        if(line<0)line=7;
      }     
    }
    write_6_8_char(0,line,'*');//��ǰ��־
    DELAY_MS(1);
    if(Add)
    {
      DELAY_MS(30);
      if(Add)
      {
        while(Add);
        switch(line)
        {
        case 1:Direction_P_AS=Direction_P_AS+1;break;
        case 2:Direction_D_AS=Direction_D_AS+10;break;
        case 3:SpeedSet=SpeedSet+1;break;
        case 5:Speed_wan_Set=Speed_wan_Set+1;break;
        case 4:Step_Middle1=Step_Middle1+20;break;
        default:break;
        }
      }
    }
    if(Sub)
    {
      DELAY_MS(30);
      if(Sub)
      {
        while(Sub);
        switch(line)
        {
        case 1:Direction_P_AS=Direction_P_AS-1;break;
        case 2:Direction_D_AS=Direction_D_AS-10;break;
        case 3:SpeedSet=SpeedSet-1;break;
        case 4:Step_Middle1=Step_Middle1-20;break;
        case 5:Speed_wan_Set=Speed_wan_Set-1;break;
        default:break;
        }
      }
    }
    write_6_8_char(0,line,'*');
    DELAY_MS(100);
    pre_show2();
  }
  if(page==3)
  {
    if(Sub_line)//����
    {
      DELAY_MS(30);//����
      if(Sub_line)
      {
        while(Sub_line);
        line++;
        if(line>7)line=0;
      }     
    }
    
    if(Add_line)//����
    {
      DELAY_MS(30);//����
      if(Add_line)
      {
        while(Add_line);
        line--;
        if(line<0)line=7;
      }     
    }
    write_6_8_char(0,line,'*');//��ǰ��־
    DELAY_MS(1);
    if(Add)
    {
      DELAY_MS(30);
      if(Add)
      {
        while(Add);
        switch(line)
        {
        case 0:Car=Car+1;break;
//        case 2:Direction_D_AS=Direction_D_AS+10;break;
//        case 3:SpeedSet=SpeedSet+1;break;
//        case 5:Speed_wan_Set=Speed_wan_Set+1;break;
//        case 4:Step_Middle1=Step_Middle1+20;break;
        default:break;
        }
       // parameter_write();
      }
    }
    if(Sub)
    {
      DELAY_MS(30);
      if(Sub)
      {
        while(Sub);
        switch(line)
        {
        case 0:Car=Car-1;break;
//        case 2:Direction_D_AS=Direction_D_AS-10;break;
//        case 3:SpeedSet=SpeedSet-1;break;
//        case 4:Step_Middle1=Step_Middle1-20;break;
//        case 5:Speed_wan_Set=Speed_wan_Set-1;break;
        default:break;
        }
       // parameter_write();
      }
    }
    write_6_8_char(0,line,'*');
    DELAY_MS(100);
    pre_show3();
  }
  
}

/**********************************************flash��д*******************************************************/

void parameter_read()
{
     if(flash_read(SECTOR_NUM, 0, uint16)!=65535) Direction_P_AS  =flash_read(SECTOR_NUM, 0, uint16);
     if(flash_read(SECTOR_NUM, 8, uint16)!=65535) Direction_D_AS  =flash_read(SECTOR_NUM, 8, uint16);
     if(flash_read(SECTOR_NUM, 16, uint16)!=65535)SpeedSet        =flash_read(SECTOR_NUM, 16, uint16);
     if(flash_read(SECTOR_NUM, 24, uint16)!=65535)Speed_wan_Set   =flash_read(SECTOR_NUM, 24, uint16);
     if(flash_read(SECTOR_NUM, 32, uint16)!=65535)Step_Middle1    =flash_read(SECTOR_NUM, 32, uint16);
     if(flash_read(SECTOR_NUM, 40, uint16)!=65535)Car             =flash_read(SECTOR_NUM, 40, uint16);
}
void parameter_write()
{
     flash_erase_sector(SECTOR_NUM);
     flash_write(SECTOR_NUM, 0, Direction_P_AS);
     flash_write(SECTOR_NUM, 8, Direction_D_AS);
     flash_write(SECTOR_NUM, 16, SpeedSet);
     flash_write(SECTOR_NUM, 24, Speed_wan_Set);
     flash_write(SECTOR_NUM, 32, Step_Middle1);
     flash_write(SECTOR_NUM, 40, Car);
}

/********************************************OLED��ʾ***********************************************/
void pre_show1()
{
    LCD_CLS();
    //write_6_8_string(10,0,"STOP!");
    //write_6_8_string(10,0,"M:");write_6_8_number(50,0,Middle_1);
    write_6_8_string(10,0,"S_1:");write_6_8_number(50,0,SubAdd);//sduty
//    write_6_8_string(10,0,"A_B:");write_6_8_number(50,0,ABDistance);
    
    
    write_6_8_string(10,1,"S:");write_6_8_number(50,1,sduty);
    
//    write_6_8_string(10,2,"dis:");write_6_8_number(50,2,distance);
    
    write_6_8_string(10,2,"P_:");write_6_8_number(50,2,P);
    
//    write_6_8_string(10,2,"ML:");write_6_8_number(50,2,Middle_L_1);
    //write_6_8_string(10,2,"f_d:");write_6_8_number(50,2,flag_distance);
//    write_6_8_string(10,2,"A_B:");write_6_8_number(50,2,ABDistance);
    
//    write_6_8_string(10,3,"MR:");write_6_8_number(50,3,Middle_R_1);
    write_6_8_string(10,3,"M_:");write_6_8_number(50,3,Middle_L_1);
//    write_6_8_string(10,3,"PD:");write_6_8_number(50,3,PD_num);
//    write_6_8_string(10,3,"S_H:");write_6_8_number(50,3,SubAdd_H*40);
    
//    write_6_8_string(10,4,"S_Q:");write_6_8_number(50,4,Last_SubAdd_Q*40);

    write_6_8_string(10,4,"S_L:");write_6_8_number(50,4,nLeft_1);
    
    write_6_8_string(10,5,"S_R:");write_6_8_number(50,5,nRight_1);
    
    write_6_8_string(10,6,"LH:");write_6_8_number(50,6,LeftH_1);
    //write_6_8_string(10,6,"can:");write_6_8_number(50,6,S_HD_can);
    
    write_6_8_string(10,7,"RH:");write_6_8_number(50,7,RightH_1);
    //write_6_8_string(10,7,"can_:");write_6_8_number(50,7,B_HD_can);
}

void pre_show2()
{
    LCD_CLS();
    write_6_8_string(10,0,"P_:");write_6_8_number(60,0,P);
    write_6_8_string(10,1,"P:");write_6_8_number(60,1,Direction_P_AS);
    write_6_8_string(10,2,"D:");write_6_8_number(60,2,Direction_D_AS);
    write_6_8_string(10,3,"V:");write_6_8_number(60,3,SpeedSet);
    write_6_8_string(10,4,"S_M:");write_6_8_number(60,4,Step_Middle1);
    write_6_8_string(10,5,"V_W:");write_6_8_number(60,5,Speed_wan_Set);
    write_6_8_string(10,6,"Dis:");write_6_8_number(60,6,distance);
    write_6_8_string(10,7,"vn:");write_6_8_number(60,7,vn);
}

void pre_show3()
{
  LCD_CLS();
  write_6_8_string(10,0,"car:");write_6_8_number(60,0,Car);
  write_6_8_string(10,1,"f_b:");write_6_8_number(60,1,front_or_back);
  write_6_8_string(10,2,"s_d:");write_6_8_number(60,2,singel_car);
  write_6_8_string(10,3,"PD:");write_6_8_number(60,3,PD_flag);
}

void pre_show4()
{
  LCD_CLS();
  write_6_8_string(10,0,"m1:");write_6_8_number(50,0,mode1);
//  write_6_8_string(10,1,"m2:");write_6_8_number(50,1,mode2);
//  write_6_8_string(10,2,"m3:");write_6_8_number(50,2,mode3);
//  write_6_8_string(10,2,"m4:");write_6_8_number(50,2,mode4);
  write_6_8_string(10,2,"V:");write_6_8_number(50,2,Speed);
  
  write_6_8_string(10,3,"A_B:");write_6_8_number(50,3,ABDistance); 
  write_6_8_string(10,4,"re_d:");write_6_8_number(50,4,re_ABDistance);
  write_6_8_string(10,5,"F_B:");write_6_8_number(50,5,front_or_back);
  write_6_8_string(10,6,"hd:");write_6_8_number(50,6,HD_overtake);
 // write_6_8_string(10,6,"go_o:");write_6_8_number(50,6,go_overtake);
  write_6_8_string(10,7,"OVER:");write_6_8_number(50,7,HD_overtake_f);
//  write_6_8_string(10,7,"go_f:");write_6_8_number(50,7,go_overtake_f);
}

void pre_show5()
{
  LCD_CLS();
  write_6_8_string(10,0,"F_B:");write_6_8_number(50,0,front_or_back);
  write_6_8_string(10,1,"go_o:");write_6_8_number(50,1,go_overtake);
  write_6_8_string(10,2,"go_f:");write_6_8_number(50,2,go_overtake_f);
  write_6_8_string(10,3,"A_B:");write_6_8_number(50,3,ABDistance);
  write_6_8_string(10,4,"hd:");write_6_8_number(50,4,HD_overtake);
  write_6_8_string(10,5,"OVER:");write_6_8_number(50,6,HD_overtake_f); 
//  write_6_8_string(10,6,"can:");write_6_8_number(50,6,S_HD_can);
  write_6_8_string(10,6,"V:");write_6_8_number(50,6,Speed);
  write_6_8_string(10,7,"can_:");write_6_8_number(50,7,B_HD_can);
}

/******************************************�ٶȱջ�����***********************************************/
//void SpeedControl()
//{
//  int i;
//  vn=ftm_quad_get(FTM2);
//  v=(float)((vn*500.0)/total);
//  ftm_quad_clean(FTM2);
//  g_fCarSpeed=vn;
//  for(i=0;i<3;i++)
//  {
//    SpeedData[i]=SpeedData[i+1];
//  }
//  SpeedData[3]=g_fCarSpeed;
//  g_fCarSpeed=0;
//  for(i=0;i<4;i++)
//  {
//    g_fCarSpeed+=SpeedData[i]*(i+1)/10;
//  }
//  
//  fDelta=SpeedSet-g_fCarSpeed;
//  
//  fP=fDelta*Speed_P;
//  
//  g_fSpeedControlIntegral+=fDelta*(Speed_I/10.0);//���ַ���
//  if(g_fSpeedControlIntegral<=-50) g_fSpeedControlIntegral=-50;//�����޷�
//  if(g_fSpeedControlIntegral>=50)  g_fSpeedControlIntegral=50;
//
//  g_fSpeedControlOutOld=g_fSpeedControlOutNew;
//  g_fSpeedControlOutNew=fP+g_fSpeedControlIntegral;
//
//  if(g_fSpeedControlOutNew>=Speedlimit)  g_fSpeedControlOutNew=Speedlimit;//������޷�
//  if(g_fSpeedControlOutNew<=-Speedlimit)  g_fSpeedControlOutNew=-Speedlimit;
//}

/******************************************�������ⲿ�ж�************************************/
void PORTD_isr(void)
{
 // PORTD_ISFR  = ~0;   
                                  //���жϱ�־λ
  if(PORTD_ISFR & (1 << 6))
  {
    PORTD_PCR6 |= PORT_PCR_ISF_MASK;
    if(gpio_get(PTD6) == 1)
    {
      PIT_TCTRL(PIT0) &= ~(PIT_TCTRL_TEN_MASK);     //����PIT ���Ա����ü���ֵ��Ч
      PIT_LDVAL(PIT0)  = ~0;                        //��������ж�ʱ��
      PIT_TCTRL(PIT0) &= ~ PIT_TCTRL_TEN_MASK;      //��ֹPITn��ʱ����������ռ���ֵ��
      PIT_TCTRL(PIT0)  = ( 0| PIT_TCTRL_TEN_MASK);  //ʹ�� PITn��ʱ��
    }
    else 
    {
      
      ultrasonic_time = ((~0)-PIT_CVAL(PIT0))*20/(1175);//50M����ʱ�ӣ�����õ�ʱ�䣬��λ��us
      ABDistance = ultrasonic_time * 340/10000;         //һ���ӵ������ٶȼ���Ϊ340�ף�����chaoshengboTime��λ��΢�룬/10000��õ���λ��cm 
      PIT_TCTRL(PIT0)  &= ~PIT_TCTRL_TEN_MASK;          //ֹͣ��ʱ��
      if(ABDistance>550)ABDistance=550;
    }
  }
}

//float time;
/***********************************************�жϺ���**************************************************/
void A_PIT_IRQHandler(void)           //PIT0.1�жϡ�9msˢ�����
{
     if(PIT_TFLG(PIT1)==1)            //���ж�  1ms�ж�  
     {
//        pit_time_start(PIT3);
       
        DirectionControlCount++;
        
        DirectionAverageCalculate();  //180us
        
        if(DirectionControlCount>=4)  //8msת��
	{ 
           DirectionControl();       
           DirectionControlCount=0;	        
        }
        Steel_Contorl();
        fengming();
        
        motor_control();
        SpeedControl();
       
        if(stop==0)
        {
          Motor_Ctrl();
        }
        else
        {
          ftm_pwm_duty(FTM1, FTM_CH0,0);
          ftm_pwm_duty(FTM1, FTM_CH1,0);
        }
           
//       time=pit_time_get_us(PIT3);
//       pit_close(PIT3);
       
       PIT_Flag_Clear(PIT1);         //���жϱ�־λ
     }
}

/***********************************************main����**************************************************/
void SCI0_send_flg(void);
void UART5_isr(void);
 void main(void)
{
    
    DELAY_MS(200);
    DisableInterrupts;//���жϣ�����error
    AD_init(); //AD������ʼ��    
    oled_init();//��ʾ����ʼ��
    
    EnableInterrupts;//���ж�
    
    key_init1();//������ʼ��
    boma();//�����ʼ��
    io_init();//IO�ڳ�ʼ��
    
    flash_init();//flash��ʼ��
    DELAY_MS(500);//flash��ʼ����ü���ʱ
    parameter_read();//��flash
    
    AB_Init();//ǰ�󳵳�ʼ��
    HD_Direction_init();//���������ʼ��
    
    ftm_quad_init(FTM2);//��ȡ����������
    steelPWM_init();//�����ʼ��
    motorPWM_init();//�����ʼ��
    
    uart_init(UART5,115200);//UART5��ʼ��
    
    set_vector_handler(PIT1_VECTORn ,A_PIT_IRQHandler);  //����PIT1���жϷ�����Ϊ PIT_IRQHandler
    set_vector_handler(UART5_RX_TX_VECTORn , UART5_isr);
    set_vector_handler(PORTD_VECTORn ,PORTD_isr);
        
    NVIC_SetPriority(PIT1_IRQn,0);//�ж����ȼ�
    NVIC_SetPriority(UART5_RX_TX_IRQn,1);
    NVIC_SetPriority(PORTD_IRQn,2);
    
    enable_irq (PIT1_IRQn);  //ʹ��PIT1�ж�(������������)
    uart_rx_irq_en (UART5);  //�����ж�
    enable_irq (PORTD_IRQn); //ʹ���ⲿ�ж�
        
    while(!start_car)//����ѭ��
    {
      DELAY_MS(2);
      DirectionControlCount++;
      DirectionAverageCalculate();
      if(DirectionControlCount>=4)  //8msת��
      {         
         DirectionControl();
         DirectionControlCount=0;	
      }
//      Steel_Contorl();
//      P=Direction_P_AS+(ABS_FLOAT(SubAdd)/5.0);//��̬P
//      sduty=P*SubAdd+Direction_D_AS*Last_Error;
//      steel_PWM=(int)sduty+Step_Middle1;
//      ftm_pwm_duty(FTM0,FTM_CH7,steel_PWM);
      KeyScan1();
      //    motor_data[0]=vn/1.0;
      //    motor_data[1]=v/1.0;
      //    motor_data[2]=fDelta/1.0;
      //    motor_data[3]=g_fSpeedControlIntegral/1.0;
      //    vcan_sendware((uint8 *)motor_data,sizeof(motor_data));
    }
    
    parameter_write();
    
    if(singel_car==0)//˫��ģʽ
    {
      if(front_or_back==0)//ǰ��
      {
        go=1;     
        SCI0_send_flg();
      }
      else
      { 
        LCD_CLS();
        while(!go)//�󳵵ȴ�����
        {
           write_6_8_string(36,3,"Waiting...");
        }
        go=0;
      }
      

    }

//    if(singel_car==0)
//    {
//      go_overtake=1;//���ܳ�����־
//      go_overtake_f=0;//���ܳ���������־
//      //clk=1;
//    }
    
    DELAY_MS(1500);//�ӳ�1.5S����    
    pit_init_ms(PIT1,1);    //��ʼ��PIT1����ʱʱ��Ϊ:1ms�ж�
    
    port_init (PTD6, ALT1 | PULLUP | IRQ_EITHER ); //�������˿ڳ�ʼ��

  while(1)
  {
    if(singel_car==0)
    {
      overtake();
    }
    
    if(singel_car==0)//˫��ģʽ
    {     
      SCI0_send_flg();
    }
    if(distance>=45)stop=1;
    pre_show1();
  }
}


/***********************************************���ݽӷ�����**************************************************/
void SCI0_send_flg(void)
{
  DisableInterrupts;
  
//  char ch;
    if(front_or_back==0)//
    {
      if(go)
      {
        while(!(UART5_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
        UART5_D = (uint8)B_GO;
        go_overtake=1;//ֱ��������־��λ
        go=0;
        mode1+=1;
      }

      if(end)
      {
        while(!(UART5_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
        UART5_D = (uint8)END;
      }
      
//      if(go_overtake)
//      {
//        while(!(UART5_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
//        UART5_D = (uint8)go_overtake_ask;
//      }
      
      if(go_overtake_f)
      {
        while(!(UART5_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
        UART5_D = (uint8)go_over;//ֱ���������
        go_overtake=0;
        go_overtake_f=0;
        front_or_back=1; 
        //clk=1;
      }
      
      if(HD_overtake)
      {
        while(!(UART5_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
        UART5_D = (uint8)HD_OVERTAKE_ASK;//�����������
        mode2+=1;
      } 
      
      if(HD_overtake_f)
      {
        while(!(UART5_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
        UART5_D = (uint8)HD_OVER;
        HD_overtake=0;
        HD_overtake_f=0;       
        front_or_back=1;
        mode3+=1;
      }
      
      
    }
    else
    {
      if(end)
      {
        while(!(UART5_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
        UART5_D = (uint8)END;
      }
      
      
    }

  EnableInterrupts;
}

//***************************************************
//�����ж�
//***************************************************
void UART5_isr(void)          
{
  DisableInterrupts;                                                     
  uint8 ch;
  while(!(UART5_S1&UART_S1_RDRF_MASK));//�ȴ����ݵ���
      ch = UART5_D; 

  if(front_or_back==1)//�󳵽���
  { 
    
    if(ch==B_GO)//ǰ������
    {
      go=1;
      go_overtake=1;//ֱ��������־��λ
      mode1+=1;
    }
    
    if(ch==HD_OVER)//������������
    {
      HD_overtake=0;
      front_or_back=0; 
      HD_overtake_f=0;
      S_HD_can=0;      
      //clk=1;
      mode3+=1;
    }
    
    if(ch==go_over)//���ܳ�������
    {
      go_overtake=0;
      go_overtake_f=0;
      front_or_back=0; 
    }
    
//    if(ch=go_overtake_ask)//���ܳ�����ʼ
//    {
//      go_overtake=1;//���ܳ�����־
//      go_overtake_f=0;//���ܳ���������־
//    }
    
    if(ch==HD_OVERTAKE_ASK)//������ʼ
    {
      HD_overtake=1;
      HD_overtake_f=0;
      mode2+=1;
    } 
    
    if(ch==END)//ǰ��Խ��
    {
      stop=1;
    }  
    
    //else clk=0;
    
  }
  else
  {
    if(ch==END)//��Խ��
    {
      stop=1;
    }
    
  }
  
    EnableInterrupts;
 }


//void main()
//{
//  DisableInterrupts;
//  
//  oled_init();
//  uart_init(UART4,9600);
//  pit_init_ms(PIT1,1);    //��ʼ��PIT1����ʱʱ��Ϊ:3ms�ж�
//  set_vector_handler(PIT1_VECTORn ,A_PIT_IRQHandler);  //����PIT1���жϷ�����Ϊ PIT_IRQHandler
//  enable_irq (PIT1_IRQn);  //ʹ��PIT1�ж�
//  ftm_quad_init(FTM2);
//  motorPWM_init();
//  EnableInterrupts;
//  while(1)
//  {
//    
//    write_6_8_string(10,1,"vn:");write_6_8_number(50,1,vn);//12820pre//0.0780037mm
//    write_6_8_string(10,2,"v:");write_6_8_number(50,2,v);
//    write_6_8_string(10,3,"p:");write_6_8_number(50,3,motor_pwm);
//    write_6_8_string(10,4,"f:");write_6_8_number(50,4,fDelta);
//    write_6_8_string(10,5,"g:");write_6_8_number(50,5,g_fSpeedControlOutNew);
//    motor_data[0]=vn/1.0;
//    motor_data[1]=v/1.0;
//    motor_data[2]=fDelta/1.0;
//    motor_data[3]=g_fSpeedControlIntegral/1.0;
//    vcan_sendware((uint8 *)motor_data,sizeof(motor_data));
//    //uart_putchar(UART4,'A');
//   // printf("123");
//  }
//}

