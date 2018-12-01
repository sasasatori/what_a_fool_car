#define Servo_PWM1 9    // SG90橙色线 ←→ Arduino 9
#define Servo_PWM2 5    // SG90橙色线 ←→ Arduino 5

//L298N直流电机驱动器
#define DC_Pin1 11   // L298N IN1 ←→ Arduino ~11(T/C2),左前
#define DC_Pin2 10   // L298N IN2 ←→ Arduino  10      左后
#define DC_Pin3 3   // L298N IN3 ←→ Arduino  ~3 (T/C2)右前
#define DC_Pin4 2   // L298N IN4 ←→ Arduino   2       右后

//红外循迹
#define IR0 A0  // 红外对管0 OUT ←→ A0
#define IR1 A1  // 红外对管1 OUT ←→ A1
#define IR2 A2  // 红外对管2 OUT ←→ A2
#define IR3 A3  // 红外对管3 OUT ←→ A3

//HC SR04超声波测距模块
#define SR_Trig 7       // HC SR04 Trig ←→ Arduino 7
#define SR_Echo 6       // HC SR04 Echo ←→ Arduino 6
/****************************************库函数和公共变量定义****************************************/

#include <Servo.h> // 引用舵机库

Servo SG901;  // 动态调用舵机类
Servo SG902;

const int IR_TH = 800;  // 红外循迹模块阈值，大于此值视为高电平，小于此值视为低电平 -------------------- 待调参数
int IR_Value[10] = {0};  // 定义 红外循迹模块读取值
int IR_STA = 0;         // 定义 红外循迹状态指示符
double SR_Distance = 0;    // 定义 超声波测量距离
int Servo_Angle = 0;  // 定义 舵机角度位置
uint8_t mode;
bool turn;

void Servo_working()
{
  turn = 1;
}

/****************************************红外循迹(IR)相关函数****************************************/

void IR_Init()  //红外循迹模块初始化
{
  pinMode(IR0, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
}

int IR_Read()
{
  int State = 0;
  IR_Value[0] =  analogRead(IR0);
  IR_Value[1] =  analogRead(IR1);
  IR_Value[2] =  analogRead(IR2);
  IR_Value[3] =  analogRead(IR3);
  for (int i = 0; i < 4; i++)
  {
    if (IR_Value[i] > IR_TH) State += 1 << i;
  }
  return State;
}


/****************************************超声波测距(SR)相关函数****************************************/

void SR_Init()//超声波初始化函数
{
  pinMode(SR_Trig, OUTPUT); // Trig ←→ Output
  pinMode(SR_Echo, INPUT);  // Echo ←→ Input
}

double SR_Read()//超声波模块测距函数
{
  unsigned long Duration; // 定义脉持续时间Duration(单位: us)
  double Distance;           // 定义距离Distance(单位: mm)

  //发送高电平脉冲，启动测量
  digitalWrite(SR_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(SR_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR_Trig, LOW);

  // 测量超声波往返时间和计算距离
  Duration = pulseIn(SR_Echo, HIGH);  // 测量超声波往返时间(单位: us)
  Distance = (double)Duration * 171 / 10000; // 计算距离(单位: cm)
  //  Serial.print("Distance:");
  //  Serial.println(Distance);
  return Distance;
}

/****************************************舵机控制(Servo)相关函数****************************************/
void Servo_Init()   // 舵机初始化
{
  SG901.attach(Servo_PWM1);  // 舵机信号的IO口 = Servo_PWM1
  SG901.write(180);
  SG902.attach(Servo_PWM2);  // 舵机信号的IO口 = Servo_PWM2
  SG902.write(120);
}

/****************************************直流电机(DCMotor)相关函数****************************************/
void DCMotor_Init()//直流电机初始化
{
  //规定IO口工作模式
  pinMode(DC_Pin1, OUTPUT);
  pinMode(DC_Pin2, OUTPUT);
  pinMode(DC_Pin3, OUTPUT);
  pinMode(DC_Pin4, OUTPUT);

  //预置高电平，使电机保持不动。
  digitalWrite(DC_Pin1, HIGH);
  digitalWrite(DC_Pin2, HIGH);
  digitalWrite(DC_Pin3, HIGH);
  digitalWrite(DC_Pin4, HIGH);
}

void DCMotor_Move(int d1, int d2) //电机运动函数，d1、d2范围-255~255，负号代表反转
{
  if (d1 >= 0) //电机1 PWM调速
  {
    analogWrite(DC_Pin1, d1);
    digitalWrite(DC_Pin2, LOW);
  }
  else
  {
    analogWrite(DC_Pin1, 255 + d1);
    digitalWrite(DC_Pin2, HIGH);
  }

  if (d2 >= 0) // 电机2 PWM调速
  {
    analogWrite(DC_Pin3, d2);
    digitalWrite(DC_Pin4, LOW);
  }
  else
  {
    analogWrite(DC_Pin3, 255 + d2);
    digitalWrite(DC_Pin4, HIGH);
  }
}

void setup()
{
  Serial.begin(9600);
  IR_Init();           // 红外测距模块初始化
  SR_Init();           // 超声波模块初始化
  Servo_Init();        // 舵机初始化
  DCMotor_Init();      // 直流电机初始化
}

/****************************************蓝牙遥控相关函数****************************************/
void BTControl()
{
  uint8_t dir;
  while (1)
  {
    dir = Serial.read();
    if (dir == 0x03) //forward
    {
      DCMotor_Move(-255, 255);
    }
    if (dir == 0x05) //back
    {
      DCMotor_Move(255, -255);
    }
    if (dir == 0x07) //rotate_left
    {
      DCMotor_Move(-150, -150);
    }
    if (dir == 0x09) //rotate_right
    {
      DCMotor_Move(255, 255);
    }
    if (dir == 0x00)
    {
      DCMotor_Move(0, 0);
    }
    if (dir == 4)   //shoot
    {
      for (Servo_Angle = 120; Servo_Angle >= 0; Servo_Angle -= 5)
        SG902.write(Servo_Angle);

    }
    if (dir == 6)
    {
      for (Servo_Angle = 0; Servo_Angle <= 120; Servo_Angle += 5)
        SG902.write(Servo_Angle);
      delay(50);
    }
    if (dir == 0x01 || dir == 0x02 || dir == 0x0C || dir == 0x0D)
    {
      break;
    }
  }
}
/****************************************蓝牙遥控相关函数****************************************/


/****************************************循迹相关函数****************************************/
void line_tracking()
{
  while (1)
  {
    int IR_total = IR_Read();
    int PID_FEB = 0;
    int V_left, V_right;
    switch (IR_total)
    {
      case 6: {
          PID_FEB = 0;  //1001
          break;
        }
      case 3: {
          PID_FEB = -1;  //0011
          break;
        }
      case 13: {
          PID_FEB = -1;  //1101
          break;
        }
      case 7: {
          PID_FEB = -3;  //0111
          break;
        }
      case 1: {
          PID_FEB = -3;  //0001
          break;
        }

      case 11: {
          PID_FEB = 1;  //1011
          break;
        }
      case 8: {
          PID_FEB = 3;  //1000
          break;
        }
      case 12: {
          PID_FEB =  1;  //1100
          break;
        }
      case 14: {
          PID_FEB =  3;  //1110
          break;
        }
      case 0: {
          PID_FEB = 1;  //0000
          break;
        }
      default: {
          break;
        } 
    }

    V_left = -PID_FEB * 250 - 240;
    V_right = -PID_FEB * 250 + 240;

    if (V_left > 255)
      V_left = 255;
    if (V_left < -255)
      V_left = -255;
    if (V_right > 255)
      V_right = 255;
    if (V_right < -255)
      V_right = -255;

    DCMotor_Move(V_left, V_right);

    //模式退出功能
    if (Serial.available())
      mode = Serial.read();
    if (mode != 0x01 && mode != 0x02) break;
  }
  return;
}
/****************************************循迹相关函数****************************************/

/****************************************避障相关函数****************************************/
double DIS_Last;
float FIFOFilter(float *p_buff, float value, int n_sample)
{

  float temp;
  float sum = 0;
  for (int i = 1; i < n_sample; i++)
  {
    p_buff[i - 1] = p_buff[i];
    sum += p_buff[i] * i;
  }
  p_buff[n_sample - 1] = value;
  sum += value * n_sample;

  temp = sum / (((n_sample + 1) / 2 * n_sample) - 1);
  return temp;

}
double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{

  double R = MeasureNoise_R;
  double Q = ProcessNiose_Q;

  static double x_last;
  double x_mid = x_last;
  double x_now;

  static double p_last;
  double p_mid ;
  double p_now;

  double kg;

  x_mid = x_last;                     //x_last=x(k-1|k-1),x_mid=x(k|k-1)
  p_mid = p_last + Q;                 //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声

  /*
      卡尔曼滤波的五个重要公式
  */
  kg = p_mid / (p_mid + R);           //kg为kalman filter，R 为噪声
  x_now = x_mid + kg * (ResrcData - x_mid); //估计出的最优值
  p_now = (1 - kg) * p_mid;           //最优值对应的covariance
  p_last = p_now;                     //更新covariance 值
  x_last = x_now;                     //更新系统状态值

  return x_now;

}

double amplitudeRestrict()
{
  double dis = 0, disFilter = 0;
  dis = SR_Read();
  uint8_t flag = 0;
  if (dis > 1000)
    return 14;
  // return KalmanFilter(dis,5,10);
  return dis;
}

void AvoidPID()
{
  static double DIS_REF = 14;
  int a,b;
  bool flag2=0;
  while (1)
  {
    double DIS, pe, pd, DIS_Last;
    static double pi = 0;
    DIS_Last = DIS;
    DIS = amplitudeRestrict();
    pe = DIS - DIS_REF;
    pd = DIS - DIS_Last;
    double value;
    static bool Left_or_Right = 1;
    static bool flag1 = 0;
    static float i = 0;
    static float pdsum[10];
    static int j = 0;
    
    pdsum[9] = pe;


    i = FIFOFilter(pdsum, pdsum[9], 10);
    if (j < 1)
    {
      if (i > 12 )
      {
        flag1 = 1;
      }
      if (flag1 == 1 && i < 1)
      {
        Left_or_Right = !Left_or_Right;
        flag1 = 0;
        j++;
        a=millis();
      }
      
    }
    if(j>=1)
    {
      b=millis();
      if((b-a)>5000 && flag2 ==  0)
      {
        Left_or_Right=!Left_or_Right;
        flag2=1;
      }
    }
    //对应左轮情况下的超声波PID
    if (Left_or_Right == 0)
    {
      SG901.write(0);

           if (pd > 1.5)
        value = pe * 3;
      else
        value = pe * 40;
        

      if (value > 0)
        DCMotor_Move(-255 + (int)value, 255);
      else
        DCMotor_Move(-255, 255 + (int)value);
    }
    //对应右轮情况下的PID
    if (Left_or_Right == 1)
    {
      SG901.write(180);

           if (pd > 1 && pd <5)
        value = pe * 3;
      else if(pd > 5 )
        value= pe * 85;
      else
        value = pe * 40;
      if (value > 0)
        DCMotor_Move(-255, 255 + (int)value);
      else
       DCMotor_Move(-255 + (int)value, 255);
      DIS_Last = DIS;
    }

    //模式退出功能
    if (Serial.available())
      mode = Serial.read();
    if (mode != 0x0C && mode != 0x0D) break;
  }
  return;
}
//DCMotor_Move(右轮，左轮);
//右轮 -为正转 +为反转
//左轮 +为正转 -为反转
//<——左      右——>
//     -       +

/****************************************避障相关函数****************************************/



/****************************************模式选取******************************************/


void loop()
{
  mode = Serial.read();

  if (mode == 0x01 || mode == 0x02)
  {
    line_tracking();
  }

  if (mode == 0x0E || mode == 0x0F)
  {
    BTControl();
  }

  if (mode == 0x0C || mode == 0x0D)
  {
    AvoidPID();
  }
}
