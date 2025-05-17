#include <MsTimer2.h>         //定时中断头文件库
#include <SSD1306.h>
// 引入驱动OLED0.96所需的库
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
 
Servo myservo;  // 创建Servo对象用以控制伺服电机
 
int pos = 90;    // 存储伺服电机角度信息的变量

/*****************OLED显示屏引脚*************/
#define SCREEN_WIDTH 128 // 设置OLED宽度,单位:像素
#define SCREEN_HEIGHT 64 // 设置OLED高度,单位:像素
 
// 自定义重置引脚,虽然教程未使用,但却是Adafruit_SSD1306库文件所必需的
#define OLED_RESET 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/***********激光测距参数定义************/
#define TOF_FRAME_HEADER 0x57//定义TOFSense系列和TOFSense-F系列的帧头
#define TOF_FUNCTION_MARK 0x00//定义TOFSense系列和TOFSense-F系列的功能码

typedef struct {
  unsigned char id;//TOF模块的id
  unsigned long system_time;//TOF模块上电后经过的时间，单位：ms
  float dis;//TOF模块输出的距离，单位：m
  unsigned char dis_status;//TOF模块输出的距离状态指示
  unsigned int signal_strength;//TOF模块输出的信号强度
  unsigned char range_precision;//TOF模块输出的重复测距精度参考值，TOFSense-F系列有效，单位：cm
} tof_parameter;//解码后的TOF数据结构体

unsigned int count_i,count_j=0;//循环计数变量
tof_parameter tof0;//定义一个存放解码后数据的结构体
unsigned char check_sum=0;//校验和
unsigned char rx_buf[32];//串口接收数组

/***********电机控制板引脚定义************/
unsigned int Motor_BIN1=11;       //控制电机的引脚（左轮）  
unsigned int Motor_BIN2=5; 
unsigned int Motor_AIN1=3;        //控制电机的引脚（右轮）
unsigned int Motor_AIN2=6;              
String TargetB_Value;             //串口获取的速度字符串变量
String TargetA_Value;             //串口获取的速度字符串变量
int value_B;                       //用于存储通过PI控制器计算得到的用于调整电机转速的PWM值的整形变量 
int value_A;                       //用于存储通过PI控制器计算得到的用于调整电机转速的PWM值的整形变量 

/***********编码器引脚************/
#define ENCODER_B_1 18              //编码器引脚—————左轮
#define ENCODER_B_2 8
#define ENCODER_A_1 2              //编码器引脚——————右轮
#define ENCODER_A_2 7             
int Velocity_B,Count_B=0;            //Count计数变量 Velocity存储设定时间内A相上升沿和下降沿的个数
int Velocity_A,Count_A=0;            //Count计数变量 Velocity存储设定时间内A相上升沿和下降沿的个数

/***********PID控制器相关参数************/
float Velocity_KP =2  , Velocity_KI= 2;//需要调整********************
volatile float Target_B=0;//右轮速度目标值
volatile float Target_A=0;//左轮速度目标值

/****************循迹红外引脚定义********************/    
//TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
//      A2                  A1                  A3                   A4
const int TrackSensorLeftPin1  =  A2;  //定义左边第一个循迹红外传感器引脚为A2
const int TrackSensorLeftPin2  =  A1;  //定义左边第二个循迹红外传感器引脚为A1
const int TrackSensorRightPin1 =  A3;  //定义右边第一个循迹红外传感器引脚为A3
const int TrackSensorRightPin2 =  A6;  //定义右边第二个循迹红外传感器引脚为A4

//定义各个循迹红外引脚采集的数据的变量
int TrackSensorLeftValue1;
int TrackSensorLeftValue2;
int TrackSensorRightValue1;
int TrackSensorRightValue2;

/************其它变量定义*****************/
int Turn = 0 ;   //转角大小
int Velocity = 5 ; //速度大小
float distance = 0;
float distance_count = 0;
float dis = 0;
float distance_mean = 0;
int flag = 1;
int flag_cd = 1;
float dis1 = 0;
float dis2 = 0;
float dis_f1 = 0;
float angle = PI/18;
/*********** 限幅************
*以下两个参数让输出的PWM在一个合理区间
*arduino mega 2560 单片机的PWM不能超过255 所以 PWM_Restrict 起到限制上限的作用
*****************************/
int PWM_Restrict=255;            //startPW+PWM_Restric=255<256

/***********初始化************/
void setup() 
{
  Serial.begin(115200);            //打开串口
  Serial.println("/*****START*****/");
  pinMode(ENCODER_B_1,INPUT);     //设置两个相线为输入模式
  pinMode(ENCODER_B_2,INPUT);
  pinMode(ENCODER_A_1,INPUT);     //设置两个相线为输入模式
  pinMode(ENCODER_A_2,INPUT);

  pinMode(Motor_AIN1,OUTPUT);   //设置右轮驱动引脚为输出模式
  pinMode(Motor_AIN2,OUTPUT); 
  pinMode(Motor_BIN1,OUTPUT);   //设置左轮驱动引脚为输出模式
  pinMode(Motor_BIN2,OUTPUT);
  
  //定义四路循迹红外传感器为输入接口
  pinMode(TrackSensorLeftPin1, INPUT);
  pinMode(TrackSensorLeftPin2, INPUT);
  pinMode(TrackSensorRightPin1, INPUT);
  pinMode(TrackSensorRightPin2, INPUT);

  //四路循迹红外传感器初始化为高电平
  digitalWrite(TrackSensorLeftPin1, HIGH);
  digitalWrite(TrackSensorLeftPin2, HIGH);
  digitalWrite(TrackSensorRightPin1, HIGH);
  digitalWrite(TrackSensorRightPin2, HIGH);

  attachInterrupt(0, READ_ENCODER_A,CHANGE);
  attachInterrupt(5, READ_ENCODER_B,CHANGE);

  MsTimer2::set(10, control); //10毫秒定时中断函数//control为函数名，可以改变
  MsTimer2::start ();        //中断使能 
  delay(300);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  myservo.attach(9);  // Servo对象连接在9号引脚 
  myservo.write(pos);
}
/***********主程序************/
void loop() 
{ 
  if(flag == 0 ){
    DISTANCE();
    dis = tof0.dis;   
    Serial.println("dis");
    Serial.println(dis);
  }else{
    words_display(distance_mean,dis_f1);
    display.display(); 
  // 检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
  // 未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
  TrackSensorLeftValue1  = digitalRead(TrackSensorLeftPin1);
  TrackSensorLeftValue2  = digitalRead(TrackSensorLeftPin2);
  TrackSensorRightValue1 = digitalRead(TrackSensorRightPin1);
  TrackSensorRightValue2 = digitalRead(TrackSensorRightPin2);
  if((TrackSensorLeftValue2 == LOW && TrackSensorRightValue2 == LOW && TrackSensorLeftValue1 == HIGH && TrackSensorRightValue2 == HIGH)|| (TrackSensorLeftValue2 == HIGH && TrackSensorRightValue1 == HIGH && TrackSensorLeftValue1 == HIGH && TrackSensorRightValue2 == HIGH)){//直行
      Velocity = 3;
      Turn=0;
      delay(10);

  }else if(TrackSensorLeftValue2 == HIGH && TrackSensorRightValue2 == HIGH && TrackSensorLeftValue1 == LOW && TrackSensorRightValue1 == HIGH){//小右行
    Velocity = 3;
    Turn=2;
    delay(40);
    
  }else if(TrackSensorLeftValue2 == HIGH && TrackSensorRightValue2 == HIGH && TrackSensorLeftValue1 == HIGH && TrackSensorRightValue1 == LOW){//小左行
    Velocity = 3;
    Turn=-2;
    delay(40);
    
  }else if(TrackSensorLeftValue2 == LOW && TrackSensorLeftValue1 == LOW && TrackSensorRightValue1 == HIGH){//大右行
    Velocity = 3;
    Turn=3;
    delay(80);
    
  }else if(TrackSensorRightValue2 == LOW && TrackSensorLeftValue1 == HIGH && TrackSensorRightValue1 == LOW){//大左行
    Velocity = 3;
    Turn=-3;
    delay(80);
     
  }else if(TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == LOW && TrackSensorLeftValue1 == LOW && TrackSensorRightValue2 == LOW){//停止
    Turn = 0 ;   
    Velocity = 0 ;  
    flag = 0;
    delay(500);
    // if( pos == 90){
    //   pos =85;
    //   myservo.write(pos);              // 告诉伺服电机达到'pos'变量的角度 
    // }   
  }
  }
}
/**********OLED显示方法************/
void words_display(float dis,float dis_f1)
{
  // 清除屏幕
  display.clearDisplay();
 
  // 设置字体颜色,白色可见
  display.setTextColor(WHITE);
 
  //设置字体大小
  display.setTextSize(1.5);
 
  //设置光标位置
  display.setCursor(0, 0);
  display.print("Distance: ");
  //打印数据
  display.print(dis);
  display.print("m");
 
  display.setCursor(0, 15);
  display.print("Distance_f1: ");
  display.print(dis_f1);
  display.print("m");

  // display.setCursor(0, 30);
  // display.print("voltage: ");
  // display.print(v);
  // display.print("V");
  
  // display.setCursor(0, 45);
  // display.print("angle: ");
  // display.print(angle);
  // display.print("");
}
/**********外部中断触发计数器函数************
*根据转速的方向不同我们将计数器累计为正值或者负值(计数器累计为正值为负值为计数器方向)
*只有方向累计正确了才可以实现正确的调整,否则会出现逆方向满速旋转
*
*※※※※※※超级重点※※※※※※
*
*所谓累计在正确的方向即
*(1)计数器方向
*(2)电机输出方向(控制电机转速方向的接线是正着接还是反着接)
*(3)PI 控制器 里面的误差(Basi)运算是目标值减当前值(Target-Encoder),还是当前值减目标值(Encoder-Target)
*三个方向只有对应上才会有效果否则你接上就是使劲的朝着一个方向(一般来说是反方向)满速旋转

例子里是已经对应好的,如果其他驱动单片机在自己尝试的时候出现满速旋转就是三个方向没对应上

下列函数中由于在A相上升沿触发时,B相是低电平,和A相下降沿触发时B是高电平是一个方向,在这种触发方式下,我们将count累计为正,另一种情况将count累计为负
********************************************/
void READ_ENCODER_A() 
{
    
    if (digitalRead(ENCODER_A_1) ==0) 
    {     
     if (digitalRead(ENCODER_A_2) == 0)      
       Count_A--;  //根据另外一相电平判定方向
     else      
       Count_A++;
    }//识别转动方向
    
    else 
    {    
     if (digitalRead(ENCODER_A_2) == 0)      
     Count_A++; //根据另外一相电平判定方向
     else      
     Count_A--;
    }
    
}
void READ_ENCODER_B() 
{
    
    if (digitalRead(ENCODER_B_1) ==0) 
    {     
     if (digitalRead(ENCODER_B_2) == LOW)      
       Count_B++;  //根据另外一相电平判定方向
     else      
       Count_B--;
    }//识别转动方向
    
    else 
    {    
     if (digitalRead(ENCODER_B_2) == LOW)      
     Count_B--; //根据另外一相电平判定方向
     else      
     Count_B++;
    }
    
}
/**********定时器中断触发函数*********/
void control()
{     

  Velocity_A=Count_A;    //把采用周期(内部定时中断周期)所累计的脉冲下降沿的个数,赋值给速度
  Count_A=0;           //将脉冲计数器清零
  Velocity_B=Count_B;    //把采用周期(内部定时中断周期)所累计的脉冲下降沿的个数,赋值给速度
  Count_B=0;           //将脉冲计数器清零
  Kinematic_Analysis(Velocity,Turn);                                   //小车运动学分析
  value_A=Incremental_PI_A(Velocity_A,Target_A);  //通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
  value_B=Incremental_PI_B(Velocity_B,Target_B);  //通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
  Set_PWM(value_A, value_B);    //将算好的值输出给电机
}
void DISTANCE(){
  /*测距代码*/
      if(Serial.available()>0)//如果串口缓存区接收到了数据
  {
    if(Serial.peek() == TOF_FRAME_HEADER)//如果串口缓存区接收到的数据是TOF_FRAME_HEADER，说明可能是TOF的数据帧头（peek不清除串口接收缓存区的该数据）
     {
       count_i=0;//数组下标计数变量置0
       rx_buf[count_i]=Serial.read();//将帧头放入数组第一个元素位置并清除串口接收缓存区的该数据
     }
     else
     {
       rx_buf[count_i]=Serial.read();//如果不是帧头则正常读取并清除串口接收缓存区的该数据
     }
        
     count_i++;//数组下标计数变量+1，准备将接收到的数据存入下一个位置
    
     if(count_i>15)//如果数组下标计数变量>15，说明接收数组中存满了16个数据，计数变量清零并进行一次解码
     {
       count_i=0;

       for(count_j=0;count_j<15;count_j++)
       {
         check_sum+=rx_buf[count_j];//计算数据的校验和
       }

      if((rx_buf[0] == TOF_FRAME_HEADER)&&(rx_buf[1] == TOF_FUNCTION_MARK)&&(check_sum == rx_buf[15]))//如果接收数组第一和第二个元素分别等于TOF_FRAME_HEADER和TOF_FUNCTION_MARK，且算出的校验和的低字节等于协议的 
       {
         tof0.id=rx_buf[3];//取TOF模块的id
         tof0.system_time=(unsigned long)(((unsigned long)rx_buf[7])<<24|((unsigned long)rx_buf[6])<<16|((unsigned long)rx_buf[5])<<8|(unsigned long)rx_buf[4]);//取TOF模块上电后经过的时间        
         tof0.dis=((float)(((long)(((unsigned long)rx_buf[10]<<24)|((unsigned long)rx_buf[9]<<16)|((unsigned long)rx_buf[8]<<8)))/256))/1000.0;//取TOF模块输出的距离
         tof0.dis_status=rx_buf[11];//取TOF模块输出的距离状态指示
         tof0.signal_strength=(unsigned int)(((unsigned int)rx_buf[13]<<8)|(unsigned int)rx_buf[12]);//取TOF模块输出的信号强度
         tof0.range_precision=rx_buf[14];//取TOF模块输出的重复测距精度参考值
        
         //通过串口打印数据
         Serial.print("id:");
         Serial.println(tof0.id);
         Serial.print("system_time:");
         Serial.println(tof0.system_time);
         Serial.print("dis:");
         Serial.println(tof0.dis);
         distance_count = distance_count + 1 ;
         distance = distance + tof0.dis;
        if(distance_count == 50){
          distance_mean = distance /50;
          distance_count = 0;
          distance = 0 ;
        // if( pos == 85 ){
        //     dis1 = distance_mean;
        //     pos = 95 ; 
        //     myservo.write(pos); 
        //     flag_cd = 0; 
        //   }else if(pos == 95){
        //     dis2 = distance_mean;
        //     flag_cd = -1; 
        //     dis_f1 = dis1*dis2*sin(angle)/sqrt(dis1*dis1+dis2*dis2-2*dis1*dis2*cos(angle));
        //     pos = 90 ; 
        //     myservo.write(pos); 
        // }else{flag = 1;}
        flag = 1;   
        }
         Serial.print("dis_status:");
         Serial.println(tof0.dis_status);
         Serial.print("signal_strength:");
         Serial.println(tof0.signal_strength);
         Serial.print("range_precision:");
         Serial.println(tof0.range_precision);
         Serial.println("");
       }
      
     }
     check_sum=0;//清空校验和
   }
}
/**************************************************************************
函数功能：小车运动数学模型
入口参数：速度和转角
//**************************************************************************/
void Kinematic_Analysis(float velocity, float turn) {
    Target_A=velocity+turn; 
    Target_B=velocity-turn;      //后轮差速
}

/***********PI控制器****************/
int Incremental_PI_A (int Encoder,float Target1)
{  
  static float Bias,PWM=0,Last_bias=0;                    //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   Bias=Target1-Encoder;                                  //计算偏差,目标值减去当前值
   PWM += Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制计算
   
   if(PWM>PWM_Restrict)
   PWM=PWM_Restrict;                                     //限幅
   
   if(PWM<-PWM_Restrict)
   PWM=-PWM_Restrict;                                    //限幅  
   
   Last_bias=Bias;                                       //保存上一次偏差 
 
   Serial.print(PWM);
   Serial.print(" ");
   Serial.println(Encoder);
 
   return PWM;                                           //增量输出
}
int Incremental_PI_B (int Encoder,float Target1)
{  
  static float Bias,PWM=0,Last_bias=0;                    //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   Bias=Target1-Encoder;                                  //计算偏差,目标值减去当前值
   PWM += Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制计算
   
   if(PWM>PWM_Restrict)
   PWM=PWM_Restrict;                                     //限幅
   
   if(PWM<-PWM_Restrict)
   PWM=-PWM_Restrict;                                    //限幅  
   
   Last_bias=Bias;                                       //保存上一次偏差 
 
   Serial.print(PWM);
   Serial.print(" ");
   Serial.println(Encoder);
 
   return PWM;                                           //增量输出
}
/**********PWM控制函数*********/
void Set_PWM(int motora, int motorb)                        
{ 
  if (motora > 0)  //如果算出的PWM为正
  {
    
    analogWrite(Motor_AIN1,motora);  //让PWM在设定正转方向(我们认为的正转方向)正向输出调整，10是死区补偿
    digitalWrite(Motor_AIN2, 0);
                                    //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
  } else if (motora == 0)  //如果PWM为0停车
  {
    digitalWrite(Motor_AIN1, 0);
    digitalWrite(Motor_AIN2, 0);
  } else if (motora < 0)  //如果算出的PWM为负
  {
    
   
     analogWrite(Motor_AIN1, motora+255); //让PWM在设定反转方向反向输出调整
     digitalWrite(Motor_AIN2,1);
   
  }

  if (motorb > 0)  //如果算出的PWM为正
  {
    
    analogWrite(Motor_BIN1,motorb);  //让PWM在设定正转方向(我们认为的正转方向)正向输出调整，10是死区补偿
    digitalWrite(Motor_BIN2, 0);
                                    //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
  } else if (motorb == 0)  //如果PWM为0停车
  {
    digitalWrite(Motor_BIN1, 0);
    digitalWrite(Motor_BIN2, 0);
  } else if (motorb < 0)  //如果算出的PWM为负
  {
    
   
     analogWrite(Motor_BIN1, motorb+255); //让PWM在设定反转方向反向输出调整
     digitalWrite(Motor_BIN2,1);
   
  }
  
}

