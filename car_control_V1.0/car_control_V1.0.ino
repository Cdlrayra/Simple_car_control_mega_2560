#include <SSD1306.h>         //oled显示屏
#include <PinChangeInt.h>    //外部中断
#include <MsTimer2.h>        //定时中断

////////OLED显示屏引脚///////////
#define OLED_DC 10
#define OLED_CLK A5
#define OLED_MOSI 13
#define OLED_RESET 12

/////////TB6612驱动引脚////
#define AIN1 11 //控制左轮
#define AIN2 5
#define BIN1 6  //控制右轮
#define BIN2 3
 
/////////编码器引脚////////
//编码器采集引脚 每路2个 共4个
#define ENCODER_L 8  //左轮
#define DIRECTION_L 4
#define ENCODER_R 7  //右轮
#define DIRECTION_R 2

/////////按键引脚////////
#define KEY 18

/////////创立oled对象/////////
SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0);

/////////定义变量//////////////
volatile long Velocity_L, Velocity_R ;   //左右轮编码器数据
int Velocity_Left = 0 ;
int Velocity_Right = 0 ;  //左右轮速度
int Velocity = 0 ; //速度大小
int Turn = 0 ;   //转角大小
float Velocity_KP = 1.3, Velocity_KI =  0.1;  //pid控制器参数
float Target_A, Target_B;     //左右轮目标速度
unsigned char Flag_Stop = 0 ; //停止标志位
int Battery_Voltage; //电池电压采样变量

void setup() {
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.clear();   // clears the screen and buffer

  pinMode(AIN1, OUTPUT);          //左轮电机控制引脚
  pinMode(AIN2, OUTPUT);           
  pinMode(BIN1, OUTPUT);          //右轮电机控制引脚
  pinMode(BIN2, OUTPUT);           

  pinMode(ENCODER_L, INPUT);       //左轮编码器引脚
  pinMode(DIRECTION_L, INPUT);        
  pinMode(ENCODER_R, INPUT);        //右轮编码器引脚
  pinMode(DIRECTION_R, INPUT);        
  delay(200);                      //延时等待初始化完成

  attachInterrupt(0, READ_ENCODER_R, CHANGE);           //开启外部中断 编码器接口1（右轮）
  attachPinChangeInterrupt(4, READ_ENCODER_L, CHANGE);  //开启外部中断 编码器接口2（左轮）

  MsTimer2::set(10, control);       //使用Timer2设置10ms定时中断
  MsTimer2::start();               //中断使能
  
}

void loop() {
   
    Velocity = 40 ;
    tracking();
  
}

/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_L() {//读取左轮编码器数据
  if (digitalRead(ENCODER_L) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L--;  //根据另外一相电平判定方向
    else      Velocity_L++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L++; //根据另外一相电平判定方向
    else     Velocity_L--;
  }
}
/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_R() {//读取右轮编码器数据
  if (digitalRead(ENCODER_R) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R++;//根据另外一相电平判定方向
    else      Velocity_R--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R--; //根据另外一相电平判定方向
    else     Velocity_R++;
  }
}

/*********函数功能：10ms控制函数 核心代码 作者：平衡小车之家*******/
void control() {
  int Temp, Temp2, Motora, Motorb; //临时变量
  static float Voltage_All; //电压采样相关变量
  static unsigned char Position_Count,Voltage_Count;  //位置控制分频用的变量
  sei();//全局中断开启
  Velocity_Left = Velocity_L;    Velocity_L = 0;  //读取左轮编码器数据，并清零
  Velocity_Right = Velocity_R;    Velocity_R = 0; //读取右轮编码器数据，并清零
 
  Kinematic_Analysis(Velocity,Turn);                                   //小车运动学分析   
  Motora = Incremental_PI_A(Velocity_Left,Target_A); //左轮速度PI控制器
  Motorb = Incremental_PI_B(Velocity_Right,Target_B); //右轮速度PI控制器

  if (Turn_Off() == 0) Set_Pwm(Motora, Motorb); //如果不存在异常，使能电机
  Temp2 = analogRead(0);  //采集一下电池电压
  Voltage_Count++;       //平均值计数器
  Voltage_All += Temp2;   //多次采样累积
  if (Voltage_Count == 200) Battery_Voltage = Voltage_All * 0.05371 / 2, Voltage_All = 0, Voltage_Count = 0; //求平均值
  Temp = My_click();   //按键检查
  if (Temp == 1)Flag_Stop = !Flag_Stop;
}

/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{   
   static float Bias,Pwm,Last_bias;
   Bias=Encoder-Target;                                  //计算偏差
   Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
   if(Pwm>255)Pwm=255;                                 //限幅
   if(Pwm<-255)Pwm=-255;                                 //限幅  
   Last_bias=Bias;                                       //保存上一次偏差 
   return Pwm;                                           //增量输出
}
int Incremental_PI_B (int Encoder,int Target)
{   
   static float Bias,Pwm,Last_bias;
   Bias=Encoder-Target;                                  //计算偏差
   Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
   if(Pwm>255)Pwm=255;                                 //限幅
    if(Pwm<-255)Pwm=-255;                                 //限幅  
   Last_bias=Bias;                                       //保存上一次偏差 
   return Pwm;                                           //增量输出
}
 
/**************************************************************************
函数功能：赋值给PWM寄存器 作者：平衡小车之家
入口参数：PWM
**************************************************************************/
void Set_Pwm(int motora, int motorb) {
  if (motora > 0)       analogWrite(AIN2, motora), digitalWrite(AIN1, LOW); //赋值给PWM寄存器
  else                 digitalWrite(AIN1, HIGH), analogWrite(AIN2, 255 + motora); //赋值给PWM寄存器

  if (motorb > 0)        digitalWrite(BIN2, LOW), analogWrite(BIN1, motorb); //赋值给PWM寄存器
  else                  analogWrite(BIN1,255 +motorb), digitalWrite(BIN2, HIGH); //赋值给PWM寄存器
}
/**************************************************************************
函数功能：小车运动数学模型
入口参数：速度和转角
//**************************************************************************/
void Kinematic_Analysis(float velocity, float turn) {
    Target_A=velocity+turn; 
    Target_B=velocity-turn;      //后轮差速
}
 
/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
/**************************************************************************/
unsigned char  Turn_Off() {
  byte temp;
  if (Flag_Stop == 0 || Battery_Voltage < 1000) { //Flag_Stop置0或者电压太低关闭电机
    temp = 1;
    digitalWrite(AIN1, LOW);  //电机驱动的电平控制
    digitalWrite(AIN2, LOW);  //电机驱动的电平控制
    digitalWrite(BIN1, LOW);  //电机驱动的电平控制
    digitalWrite(BIN2, LOW);  //电机驱动的电平控制
  }
  else      temp = 0;
  return temp;
}
/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击
**************************************************************************/
unsigned char My_click (void) {
    static byte flag_key = 1; //按键按松开标志
    if (flag_key && (digitalRead(KEY) == 0))   { //如果发生单击事件
    flag_key = 0;
    if (digitalRead(KEY) == 0)  return 1;    //M键
  }
  else if (digitalRead(KEY) == 1)     flag_key = 1;
  return 0;//无按键按下
}
/***************************************************************************
函数功能：4路巡线
入口参数：无
返回  值：无
***************************************************************************/
unsigned char  tracking (void) {
 
}