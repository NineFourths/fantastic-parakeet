#include<Servo.h>
Servo BaseServo,PpServo,WaistServo,NeckServo,
      HeadServo,ClawServo,TurnServo;
//      机械臂从下到上对应 底座舵机 臀部舵机 腰部舵机（仅固定作用） 颈部舵机 头部舵机（仅固定作用） 爪部舵机 同时声明转向舵机
const int Pin_BaseServo=10;
const int Pin_PpServo =9;
const int Pin_WaistServo=8;
const int Pin_NeckServo =7;
const int Pin_HeadServo =6;
const int Pin_ClawServo =5;
const int Pin_TurnServo =4;
const int Pin_PWMA =2;
const int Pin_PWMB =3;
const int Pin_TraceSensor1= A1;
const int Pin_TraceSensor2= A2;
const int Pin_TraceSensor3= A3;
const int Pin_TraceSensor4= A4;
const int Pin_AIN2= 50;
const int Pin_AIN1= 51  ;
const int Pin_BIN1= 52;
const int Pin_BIN2= 53;
const int Min_BaseServo= 0;
const int Max_BaseServo= 180;
const int Min_PpServo= 0;
const int Max_PpServo =180;
const int Min_WaistServo= 0;
const int Max_WaistServo= 270;
const int Min_NeckServo= 0;
const int Max_NeckServo= 180;
const int Min_ClawServo= 45;
const int Max_ClawServo= 110;
const int Time_Servo_Delay= 30;
const int Move_Step=10;
const int Turn_Move_Step=35;
//以上均为引脚的声明
void Tracing(){//定义自动循迹模式函数
    while(1){
      if(Serial3.available()){
      char Serial_cmd=Serial3.read();
      if(Serial_cmd=='M')return;//循迹模式转遥控模式，会有一定的延迟，可能与循迹模式一直被循迹模块输入信息占用有关，暂无解决方法
    }
    delay(30);
      int data[4];//创建一个数组
    data[0]=digitalRead(Pin_TraceSensor1);
    data[1]=digitalRead(Pin_TraceSensor2);
    data[2]=digitalRead(Pin_TraceSensor3);
    data[3]=digitalRead(Pin_TraceSensor4);//数组的数字储存传感器状态，遇到黑线则为0，同时小灯亮起（有灯有黑线）
    
    if(data[1]==0&&data[0]==1&&data[2]==1&&data[3]==0)//如果传感器0110，前进
    {TurnServo.write(90);Car_Go(50,50); }//方向正向，50速度行驶
    if(data[1]==0&&data[0]==1&&data[2]==0&&data[3]==0)//如果传感器0100，左转
    {TurnServo.write(90);Car_Go(50,50);Car_Small_Left();}//方向摆正，50速度行驶，左小转弯
    if(data[1]==0&&data[0]==0&&data[2]==1&&data[3]==0)//如果传感器0010，右转
    {TurnServo.write(90);Car_Go(50,50);Car_Small_Right();}//方向摆正，50速度行驶，右小转弯
    if(data[0]&&data[1]&&data[2]&&data[3])//如果传感器1111，停止，并退出到loop函数
    {Car_Stop(); return;}
    if(!data[0]&&!data[1]&&!data[2]&&!data[3])//如果传感器0000，前进
    {Car_Go(50,50);}
    if(data[1]==1&&data[0]==0&&data[2]==0&&data[3]==0)//1000，左大转弯
    {Car_Big_Left();}
    if(data[1]==0&&data[0]==0&&data[2]==0&&data[3]==1)//0001，右大转弯
    {Car_Big_Right();}
    if(data[1]==1&&data[0]==1&&data[2]==1&&data[3]==0)//如果传感器0001，左直角
    {Spin_Left();}
    if(data[1]==1&&data[0]==1&&data[2]==0&&data[3]==0)//如果传感器0011，左直角
    {Spin_Left();}
    if(data[1]==0&&data[0]==1&&data[2]==1&&data[3]==1)//如果传感器1000，右直角
    {Spin_Right();}
    if(data[1]==0&&data[0]==0&&data[2]==1&&data[3]==1)//如果传感器1100，右直角
    {Spin_Right();}
    
     }}
void RemoteCtrl(){while(1){
  if(Serial3.available()){
    char Cmd_RemoteCtrl=Serial3.read();
    switch(Cmd_RemoteCtrl){
      case 'L':BaseServo_Left();break;
      case 'R':BaseServo_Right();break;//底座左旋、右旋
      case 'F':PpServo_Front();break;
      case 'B':PpServo_Back();break;//臀部骨盆前倾、后仰
      case 'U':WaistServo_Up();break;
      case 'D':WaistServo_Down();break;//腰部上抬、下垂（已弃用，因为腰部舵机undingable）
      case 'N':NeckServo_Nod();break;
      case 'r':NeckServo_Rise();break;//颈部抬头、低头（此处r为小r，其余机械臂的Cmd_RemoteCtrl均为大写字母）
      case 'O':ClawServo_Open();break;
      case 'C':ClawServo_Close();break;//爪部开合
      case 'w':TurnServo.write(90);Car_Go(120,120);break;//自然选择，前进四
      case 'a':Car_Left(45);break;//左转弯
      case 's':Car_Back();break;//后退
      case 'd':Car_Right(45);break;//右转（控制小车运动的Cmd_RemoteCtrl均为小写字母。）
      case 'S':Car_Stop();break;//方向摆正，停车
      case 'G':for(int Neck_pos=32;Neck_pos<72;Neck_pos+=15){NeckServo.write(Neck_pos);delay(333);}
               for(int Base_pos=113;Base_pos>93;Base_pos-=15){BaseServo.write(Base_pos);delay(333);}
               for(int Pp_pos=141;Pp_pos>116;Pp_pos-=15){PpServo.write(Pp_pos);delay(333);}
               break;//Go口令，做出抓取前的预备动作
               
      case 'I':for(int Pppos=PpServo.read();Pppos<141;Pppos+=15){PpServo.write(Pppos);delay(333);}
               for(int Basepos=BaseServo.read();Basepos<113;Basepos+=15){BaseServo.write(Basepos);delay(333);}
               for(int Neckpos=NeckServo.read();Neckpos>32;Neckpos-=15){NeckServo.write(Neckpos);delay(333);}
               break;//Initial口令，做出出场动作
      case 'T':TurnServo.write(90);break;//用于左右转弯后方向的摆正（软件操控时，长按左右键则左右转，松开则方向摆正【单点没有这种效果】）         
      case 'A':return;//返回loop函数
    }
  }
}}
void Car_Go(int PWMA,int PWMB){//让车子动
  
  analogWrite(Pin_PWMA,PWMA);
  analogWrite(Pin_PWMB,PWMB);
  digitalWrite(Pin_AIN2,HIGH);
  digitalWrite(Pin_AIN1,LOW);
  digitalWrite(Pin_BIN2,HIGH);
  digitalWrite(Pin_BIN1,LOW);
  if(Serial3.available()){if(Serial3.read()=='S')
    Car_Stop();
  }}
void Car_Back(){//让车子退
  analogWrite(Pin_PWMA,150);
  analogWrite(Pin_PWMB,150);
  digitalWrite(Pin_AIN1,HIGH);
  digitalWrite(Pin_AIN2,LOW);
  digitalWrite(Pin_BIN1,HIGH);
  digitalWrite(Pin_BIN2,LOW);
  if(Serial3.available()){if(Serial3.read()=='S')
    Car_Stop();
  }
  }
  void Car_Small_Left()//循迹模式小左转
  {
  Car_Go(30,96);
  delay(363);
  }
  void Car_Small_Right()//循迹模块小右转
  {
    Car_Go(96,30);
    delay(363);
  }
void Car_Left(int Left_Turn_Move_Step){//遥控模式的左转
  Car_Go(96,120); 
    TurnServo.write(90-Left_Turn_Move_Step);
    delay(350);
    Car_Go(120,120);
    
}

void Car_Right(int Right_Turn_Move_Step){//遥控模式的右转
  Car_Go(120,96);
    TurnServo.write(90+Right_Turn_Move_Step);
    delay(350);
    Car_Go(120,120);
   
}
void Car_Stop(){//遥控模式的停车
  TurnServo.write(90);
  digitalWrite(Pin_PWMA,LOW);
  digitalWrite(Pin_PWMB,LOW);
}
void Car_Big_Left()//循迹大左转0111
{
  TurnServo.write(30);
  digitalWrite(Pin_AIN1,HIGH);
  digitalWrite(Pin_AIN2,LOW);
  digitalWrite(Pin_BIN2,HIGH);
  digitalWrite(Pin_BIN1,LOW);
  analogWrite(Pin_PWMA,110);
  analogWrite(Pin_PWMB,120);
  delay(460); 

}
void Car_Big_Right()//循迹大右转1110
{  
  TurnServo.write(150);
  digitalWrite(Pin_BIN1,HIGH);
  digitalWrite(Pin_BIN2,LOW);
  digitalWrite(Pin_AIN2,HIGH);
  digitalWrite(Pin_AIN1,LOW);
  analogWrite(Pin_PWMA,120);
  analogWrite(Pin_PWMB,110);
  delay(460);
}
void Spin_Left()//循迹直角左转1000、1100
{TurnServo.write(45);
  digitalWrite(Pin_AIN1,HIGH);
  digitalWrite(Pin_AIN2,LOW);
  analogWrite(Pin_PWMA,130);
  analogWrite(Pin_PWMB,140);
  delay(999); 
  
}
void Spin_Right()//循迹直角右转1100、1110
{TurnServo.write(135);
  digitalWrite(Pin_BIN1,HIGH);
  digitalWrite(Pin_BIN2,LOW);
  analogWrite(Pin_PWMA,140);
  analogWrite(Pin_PWMB,130);
  delay(999);
 
}
void BaseServo_Left(){
  int temp =BaseServo.read();
  temp+=Move_Step;
    BaseServo.write(temp);
    Serial3.println("-----------------------------");
    Serial3.print("BaseServo pos:");
    Serial3.print(temp);
    Serial3.println("-----------------------------");
  delay(Time_Servo_Delay);
  }
void BaseServo_Right(){
  int temp =BaseServo.read();
  temp-=Move_Step;
    BaseServo.write(temp);
  Serial3.println("-----------------------------");
    Serial3.print("BaseServo pos:");
    Serial3.print(temp);
    Serial3.println("-----------------------------");
  delay(Time_Servo_Delay);
  }
void PpServo_Front(){
  int temp=PpServo.read();
  temp-=Move_Step;
  PpServo.write(temp);
  Serial3.println("-----------------------------");
    Serial3.print("PpServo pos:");
    Serial3.print(temp);
    Serial3.println("-----------------------------");
  delay(Time_Servo_Delay);
  }
void PpServo_Back(){
  int temp=PpServo.read();
 temp+=Move_Step;
  PpServo.write(temp);
   Serial3.println("-----------------------------");
    Serial3.print("PpServo pos:");
    Serial3.print(temp);
    Serial3.println("-----------------------------");
  delay(Time_Servo_Delay);
  }
  void WaistServo_Up(){
    int temp=WaistServo.read();
  temp+=Move_Step;
  WaistServo.write(temp);
  Serial3.println("-----------------------------");
    Serial3.print("WaistServo pos:");
    Serial3.print(temp);
    Serial3.println("-----------------------------");
  delay(Time_Servo_Delay);
  }
  void WaistServo_Down(){
    int temp=WaistServo.read();
 temp-=Move_Step;
  WaistServo.write(temp);
  Serial3.println("-----------------------------");
    Serial3.print("WaistServo pos:");
    Serial3.print(temp);
    Serial3.println("-----------------------------");
  delay(Time_Servo_Delay);
  }
  void NeckServo_Nod(){
    int temp=NeckServo.read();
 temp-=Move_Step;
  NeckServo.write(temp);
  Serial3.println("-----------------------------");
    Serial3.print("NeckServo pos:");
    Serial3.print(temp);
    Serial3.println("-----------------------------");
  delay(Time_Servo_Delay);
  }
  void NeckServo_Rise(){
    int temp=NeckServo.read();
 temp+=Move_Step;
  NeckServo.write(temp);
  Serial3.println("-----------------------------");
    Serial3.print("NeckServo pos:");
    Serial3.print(temp);
    Serial3.println("-----------------------------");
  delay(Time_Servo_Delay);
  }
  void ClawServo_Open(){
    int temp=ClawServo.read();
  temp-=Move_Step;
  ClawServo.write(temp);
  Serial3.println("-----------------------------");
    Serial3.print("ClawServo pos:");
    Serial3.print(temp);
    Serial3.println("-----------------------------");
  delay(Time_Servo_Delay);
  }
  void ClawServo_Close(){
    int temp=ClawServo.read();
   temp+=Move_Step;
  ClawServo.write(temp);
   Serial3.println("-----------------------------");
    Serial3.print("ClawServo pos:");
    Serial3.print(temp);
    Serial3.println("-----------------------------");
  delay(Time_Servo_Delay);
  }
void setup() {
  // put your setup code here, to run once:
Serial3.begin(9600);//监视器准备
BaseServo.attach(Pin_BaseServo);
PpServo.attach(Pin_PpServo);
WaistServo.attach(Pin_WaistServo);
NeckServo.attach(Pin_NeckServo);
HeadServo.attach(Pin_HeadServo);
ClawServo.attach(Pin_ClawServo);
TurnServo.attach(Pin_TurnServo);
pinMode(Pin_TraceSensor1,INPUT_PULLUP);
pinMode(Pin_TraceSensor2,INPUT_PULLUP);
pinMode(Pin_TraceSensor3,INPUT_PULLUP);
pinMode(Pin_TraceSensor4,INPUT_PULLUP);
pinMode(Pin_AIN1,OUTPUT);
pinMode(Pin_AIN2,OUTPUT);
pinMode(Pin_BIN1,OUTPUT);
pinMode(Pin_BIN2,OUTPUT);//以上均为引脚的设置
HeadServo.write(168);
WaistServo.write(-5);
NeckServo.write(32);
PpServo.write(141);
ClawServo.write(78);
BaseServo.write(113);//摆出出场姿势
}

void loop() {
digitalWrite(Pin_PWMA,LOW);
digitalWrite(Pin_PWMB,LOW);//确保模式转换后能停车
  Serial3.println("Communication is ready.");
  // put your main code here, to run repeatedly:
if(Serial3.available()>0){
  Serial3.println("1");
  char ModeCmd=Serial3.read();
  switch(ModeCmd){
    case 'A':Serial3.println("Switch to the tracing mode.");//夹取方块后进入循迹模块
             Tracing();break;
    case 'X':Serial3.println("Switch to the tracing mode.");//启动时先进行一段直角右转弯进入轨道内，后进入循迹模块【在检录时尽
//                                                            量争取尝试此设定是否可以令车子进入轨道之中，若不理想则进行调试】
             Car_Go(50,50);delay(369);Spin_Right();//上面说的调试就是将这个右直角转弯注释掉【在338行前输入双斜杠//(以此操作为准)】，然后将下面的代码注释取消
//            TurnServo(134);
//            Car_Go(66,50);//此差速度设置可以进行改动
//            delay(669);//如果此delay时间不理想，则修改此时间，其他代码尽量不要改动【除上】
//            TurnServo(90);
              Tracing();break;
    case 'M':Serial3.println("Switch to the remote-controlling mode.");
             RemoteCtrl();break;}
}
  else Serial3.println("0");
delay(3000);
} 
