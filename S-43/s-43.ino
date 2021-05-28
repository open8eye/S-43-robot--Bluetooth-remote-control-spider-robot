/*
  -项目名称：蓝牙遥控蜘蛛机器人
  --日期:2021/5/27
  --参考：<RegisHsu的蜘蛛机器人>   <cbirdfly的遥控蜘蛛机器人>
  --描述:发现SerialCommand不能读取蓝牙串口,额,就是不会用RegisHsu大叔的的SerialCommand.h的库!反正蓝牙串口始终不能接受手机数据,只能
  放弃.倒是arduino软件上的串口监视器发送命令没问题RegisHsu叔的pc端的串口通信可以控制机器人,就是蓝牙串口只有arduino 发送的,arduino不
  接受手机的数据.现在改了哈,蓝牙串口(yes)pc串口(yes).做完了在原贴发现RegisHsu应该是pc通信了一个蓝牙模块,作为串口,arduino有个蓝牙模
  块..这反正不能手机控制不爽
  -注意事项:
  --供电:舵机供电最好选择独立5-7v(建议5v多一点 或者6v左右 注:很多6v的电池充满是7v的)电源,否则会烧板子...我的nano就是这么坏的(后来修好
  了,相关资料6,7,8,9就是了).所以我现在用的我的uno.电池就淘宝搜索dc6v之类的玩具电池尺寸自己量一下我现在没加
  --舵机安装时脚的角度问题:看RegisHsu叔的原贴:https://www.instructables.com/DIY-Spider-RobotQuad-robot-Quadruped/ 这里说
  明下最外面的脚与地面垂直,中间的脚与地面平行,最里面的脚与身体垂直,在俯视图中会像个躺着的H.
  角度图片地址:
     https://gitee.com/lulendi/s-43-robot--bluetooth-remote-control-spider-robot/blob/master/images/%E8%84%9A%E8%A7%92%E5%BA%A6%E5%9B%BE.jpg
  --舵机站立时的角度姿态:舵机在接收站立命令时,为了保证三点支撑,会左侧或右侧两足平行,另一侧两足会呈现90°左右的夹角.这在RegisHsu的YouTube
  视频里有演示
      图片地址:https://gitee.com/lulendi/s-43-robot--bluetooth-remote-control-spider-robot/blob/master/video/duojijiaowei.png
  --蓝牙:蓝牙买hc-06就好(蓝牙我选的这个10块钱那个 https://m.tb.cn/h.4H8xdpn?sm=6053eb) 手机app我是在小米应用里搜索的蓝牙调试器其
  实app大多都一样,hc-06接线,如果有扩展板就直接rxd->tx, txd->rx,gnd->gnd/-,vcc->3.3v/5v/+(主要看蓝牙模块电压要求,现在大多数3.2v
  -6v都可以)没扩展板,也可以直接看你的板子tx和rx是哪个数字引脚,像上面这么接就可以了.
  --扩展板:用Nano的会接线的,建议自己洞洞板自己焊,像RegisHsus那样,或者买个nano多用扩展板,会有个dc电压接口,听说红板电压高点可以到7v,具
  体不知道,但我记得这个板子本身可以最高承受7v最好不要这么做,5v可以了,不知道可以询问商家;uno的就用R3 v5扩展板就好(这个跳线帽一拔掉就是独
  立供电)
  --杜邦线:准备几根杜邦线公对公和母对母都来点,用来安装hc-06.但是如果你会焊接,也可以直接吧hc-06焊在扩展板上一样的(焊上去不容易松)
  --上传:uno板在上传时插着扩展板扩展板上接了蓝牙占用了串口接口,会导致上传出错.(上传时拔下来)
  --打印件:https://www.thingiverse.com/thing:1009659 打印里skp文件可以修改
  --RegisHsu源代码:https://github.com/regishsu/SpiderRobot
  --RegisHsu的原贴:https://www.instructables.com/DIY-Spider-RobotQuad-robot-Quadruped/
  -相关资料:
    1 https://blog.csdn.net/C1664510416/article/details/82939771/  cbirdfly的四足蜘蛛机器人--制作过程记录 参考
    2 https://www.instructables.com/DIY-Spider-Robot-PART-II-Remote-control/   RegisHsu的蓝牙模块
    3 https://www.geek-workshop.com/thread-15607-1-1.html  判断板子是否烧毁
    4 https://www.arduino.cn/thread-98597-1-1.html uno扩展板供电
    5 https://support.arduino.cc/hc/en-us/articles/360021557319-Error-avrdude-stk500-recv-programmer-is-not-responding  
    出现avrdude: stk500_recv(): programmer is not responding问题
    6 https://www.cnblogs.com/dz-study/p/11681319.html  用arduino的uno开发板为nano板子烧写bootloader
    7 https://blog.csdn.net/wulala789/article/details/98946833 如何用Arduino Nano给另外一个Arduino Nano烧录bootloader
    8 https://blog.csdn.net/qq_27133869/article/details/104228553 用Arduino板为另一块Arduino烧写（更新）BootLoader(修复nano/uno)
    9 当你发现你用上面的方法烧成功了板子,但依然不能上传成功,这时考虑可能是是microusb接口坏,数据信号不能正常传输(上一面方法都不行的,建议
    通电检查pow灯是否亮和L灯是闪烁)可以看这篇文章https://blog.csdn.net/weixin_44147894/article/details/102852602 可以直接将
    USB数据线接到ch431G(这是Nano改进版的芯片其他板子自行百度)芯片引脚上.  nano板可算修好了!14块钱啊,搞半天芯片没坏,还好还好,不用花钱
    买了!
   10 没得FlexiTimer2.h库文件的https://playground.arduino.cc/Main/FlexiTimer2/这里下
   11 没得SerialCommand.h库文件的 https://github.com/kroimon/Arduino-SerialCommand 这里下,虽然这个代码不用,但是总有人会要的
  -更新说明:
    修改了控制方法
        控制命令在串口端(电脑串口监视器/蓝牙端)改为
    // c01: 站
    // c00: 坐
    // c1x: 前进 x 步
    // c2x: 后退 x 步
    // c3x: 右转 x 步
    // c4x: 左转 x 步
    // c5x: 握手 x 时间
    // c6x: 挥手 x times

*/
//库函数----------------------------------------------------------------------
#include <Servo.h>    //舵机控制库
#include <FlexiTimer2.h>//设置定时器去同时控制多个舵机

#define C_STAND_SIT    0
#define C_FORWARD      1
#define C_BACKWARD     2
#define C_LEFT         3
#define C_RIGHT        4
#define C_SHAKE        5
#define C_WAVE         6
#define S_STRAIGHT     1
#define S_BACK         2

//舵机对象--------4条腿 每条腿3个舵机 一共12个舵机------------------------------
Servo servo[4][3];
//设置信号输出引脚
const int servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13} };
/* 机器的尺寸和大小 ---------------------------------------------------------*/
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;
/* 运动常量
  ----------------------------------------------------*/
const float z_default = -40, z_up = 90, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;
int  In1 = A1;
int  In2 = A2;
int  In3 = A3;
int  In4 = A4;
int  EnA = A0;
int  EnB = A5;
int Check_angle = 1;
int straiht_angle = 180;
/* 运动变量
  ----------------------------------------------------*/
volatile float site_now[4][3];    //每只脚到末端的实时距离
volatile float site_expect[4][3]; //预计每只脚到末端的实时距离
float temp_speed[4][3];   //每个轴的速度  注意：需要在每次运动前重新计算
float speed_multiple = 1; //动作速度执行倍数
const float spot_turn_speed = 4;  //转动速度
const float leg_move_speed = 8;  //每条腿的移动速度
const float body_move_speed = 3; //身体移动速度
const float stand_seat_speed = 1; //站位速度
volatile int rest_counter;      //+1/0.02s, 自动停机时间
//函数传递时用的参数
const float KEEP = 255;
float move_speed = leg_move_speed;   //动作速度
//π值
const float pi = 3.1415926;
/* 转动常量
  --------------------------------------------------------*/
//临时长度
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;
/* ---------------------------------------------------------------------------*/



//初始化函数
void setup()
{
  //启动串口
  Serial.begin(9600);
  //初始4条腿的初始大小
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  //启动舵机控制服务(设置定时器)
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start(); //开始定时器
  Serial.println("舵机定时器开启");
  //初始化舵机
  servo_attach();//设置舵机接口函数
  Serial.println("舵机初始化完成!");
  Serial.println("机器人初始化完成!");
  //直流电机初始化
  straight_attach();
  match_key("c00");//开启默认坐姿
  //  Serial.println("Straight Initialized");
}

void servo_attach(void)   //设置舵机连接串口
{
  for (int i = 0; i < 4; i++)  //4条腿
  {
    for (int j = 0; j < 3; j++)  //每条腿3个舵机
    {
      servo[i][j].attach(servo_pin[i][j]);  //设定舵机的接口
      delay(100);  //等待100毫秒
    }
  }
}

void servo_detach(void) //舵机分离串口
{
  for (int i = 0; i < 4; i++)  //4条腿
  {
    for (int j = 0; j < 3; j++) //每条腿3个舵机
    {
      servo[i][j].detach();  //使舵机与其接口分离
      delay(100);  //等待100毫秒
    }
  }
}

//主函数
void loop()
{

  while (Serial.available())
  {
    // 测试方法
    //        String t1 = Serial.readString();
    //    t1.trim();
    //    Serial.println(t1);
    //    Serial.println(t1);
    //    if (t1 == "t") {
    //      Serial.println("Test");
    //    }
    //    delay(2000);
    String t1 = Serial.readString();
    //匹配字符串
    match_key(t1);


  }
}
//抽取一个方法,用来匹配数据 不返回值 需要输入参数 String str 参数从Serial里读取
void match_key(String string) {
  string.trim();//自动清除字符串中最前面和最后面的空白数据。

  if (string.length() != 3) {
    unrecognized();
  }
  else if (string.startsWith("c")) {
    //控制模式
    control_model(string);
  }
  else  if (string.startsWith("d")) {
    //speed_set
    speed_set(string) ;
  }
  else  if (string.startsWith("s")) {
    //straight_model
    straight_model(string);
  }
  else {
    unrecognized();
  }
}


//检测舵机是否达到直流电机运动角度
int check_angel(void) {
  if (straiht_angle == 0)
    return 1;
  else
    return 0;
}
//设置直流电机运动四足角度
void set_angle(void) {
  set_site(0, x_default + x_offset, y_start , 20);
  set_site(1, x_default + x_offset, y_start , 20);
  set_site(2, x_default + x_offset, y_start , 20);
  set_site(3, x_default + x_offset, y_start , 20);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  straiht_angle = 0;
}
//设置运动速率 (已修改)
void speed_set(String str) {
  String arg;
  int set_speed, no_do;
  Serial.println("Action:");
  arg = str.substring(1, 2);
  set_speed = arg.toInt();
  arg = str.substring(2, 3);
  no_do = arg.toInt();
}
//串口库函数的默认错误调用函数
void unrecognized(void) {
  Serial.println("What?");
}
/*
   void unrecognized(const char *command) {
  Serial.println("What?");
  }
*/
//设置直流电机接口
void straight_attach(void) {
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
}

//直行模式 (已修改)
void straight_model(String str) {
  String arg;
  int action_model, no_do;
  Serial.println("Action:");
  arg = str.substring(1, 2);
  action_model = arg.toInt();
  arg = str.substring(2, 3);
  no_do = arg.toInt();
  switch (action_model) {
    case S_STRAIGHT:
      Serial.println("Straight Format");
      analogWrite(EnA, 255);
      analogWrite(EnB, 255);
      sit();
      if (!check_angel()) {
        set_angle();
      }
      s_straight();
      break;
    case S_BACK:
      Serial.println("Straight Back");
      analogWrite(EnA, 255);
      analogWrite(EnB, 255);
      sit();
      if (!check_angel()) {
        set_angle();
      }
      s_back();
      break;
    default:
      Serial.println("Undefine");
      break;
  }
}

//直流电机前进
void s_straight(void) {
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
}
//直流电机退后
void s_back(void) {
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
}

//直流电机制动
void s_stop(void) {
  straiht_angle = 180;
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}


//循迹模式
void follow_model(void) {

}
//避障模式
void obstacle_model(void)
{

}



//打印动作 (新增方法)
void show_action(int i)
{

  Serial.print("动作模式:");

  switch (i)
  {
    case C_FORWARD:
      Serial.println("前进");
      break;
    case C_BACKWARD:
      Serial.println("后退");
      break;
    case C_LEFT:
      Serial.println("左转");
      break;
    case C_RIGHT:
      Serial.println("右转");
      break;
    case C_STAND_SIT:
      Serial.println("站/坐");
      break;
    case C_SHAKE:
      Serial.println("握手");
      break;
    case C_WAVE:
      Serial.println("挥手");
      break;
    default:
      Serial.println("Error");
      break;
  }
}

//控制模式 (已修改)
void control_model(String str)
{
  String arg;
  int action_mode, n_step; //动作模式，移动步数
  Serial.println("Action:");
  arg = str.substring(1, 2);
  action_mode = arg.toInt();
  //准备一个方法用来打印动作模式
  show_action(action_mode);
  arg = str.substring(2, 3);
  n_step = arg.toInt();
  Serial.println("移动:" + arg + "步");


  switch (action_mode)
  {
    case C_FORWARD:
      Serial.println("Step forward");
      if (!is_stand()) {
        stand();
      }
      s_stop();
      step_forward(n_step);
      break;
    case C_BACKWARD:
      Serial.println("Step back");
      if (!is_stand()) {
        stand();
      }
      s_stop();
      step_back(n_step);
      break;
    case C_LEFT:
      Serial.println("Turn left");
      if (!is_stand())
        stand();
      s_stop();
      turn_left(n_step);
      break;
    case C_RIGHT:
      Serial.println("Turn right");
      if (!is_stand())
        stand();
      s_stop();
      turn_right(n_step);
      break;
    case C_STAND_SIT:
      Serial.println("1:up,0:dn");
      if (n_step) {
        s_stop();
        stand();
      } else {
        s_stop();
        sit();
      }

      break;
    case C_SHAKE:
      Serial.println("Hand shake");
      s_stop();
      hand_shake(n_step);
      break;
    case C_WAVE:
      Serial.println("Hand wave");
      s_stop();
      hand_wave(n_step);
      break;
    default:
      Serial.println("Error");
      break;
  }
}


//是否站立
bool is_stand(void)
{
  if (site_now[0][2] == z_default)
    return true;
  else
    return false;
}

//坐下
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

//站立
void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}


//左转
void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

//  - 右转
void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

//前进
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

//后退
void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

//身体左倾
void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

//身体右倾
void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

//摇手
void hand_wave(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

//招手
void hand_shake(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}



/*
  - 舵机服务/定时器中断功能/50Hz
  - 当设置site expected时，这个函数会移动到目标直线
  - 在设置expect之前，应该设置temp_speed[4][3]，确保
    直线移动，决定移动速度。
   ---------------------------------------------------------------------------*/
void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}

void set_site(int leg, float x, float y, float z)  //设置某一条腿的最终坐标
{
  float length_x = 0, length_y = 0, length_z = 0;  //初始化float类型变量length_x,length_y,length_z

  if (x != KEEP)  //假若x轴不是保持状态
    length_x = x - site_now[leg][0];  //计算x轴长度
  if (y != KEEP)  //假若y轴不是保持状态
    length_y = y - site_now[leg][1];  //计算y轴长度
  if (z != KEEP)  //假若z轴不是保持状态
    length_z = z - site_now[leg][2];  //计算z轴长度

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));   //计算宗长度

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;  //计算对应腿的舵机1移动速度
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;  //计算对应腿的舵机2移动速度
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;  //计算对应腿的舵机3移动速度

  if (x != KEEP)  //假若x轴不是保持状态，则设置目标角度
    site_expect[leg][0] = x;
  if (y != KEEP)  //假若y轴不是保持状态，则设置目标角度
    site_expect[leg][1] = y;
  if (z != KEEP)  //假若z轴不是保持状态，则设置目标角度
    site_expect[leg][2] = z;
}

/*单条腿部动作完成度检测
  -----------------------------------------------------------------------*/
void wait_reach(int leg)  //等待某条腿动作完成函数
{
  while (1) //死循环
    if (site_now[leg][0] == site_expect[leg][0])    //等待目标腿的 舵机 1达到目标角度
      if (site_now[leg][1] == site_expect[leg][1])//等待目标腿的 舵机 2达到目标角度
        if (site_now[leg][2] == site_expect[leg][2])//等待目标腿的 舵机 3达到目标角度
          break;  //跳出循环
}

/*四条动作完成度检测
  -----------------------------------------------------------------------*/
void wait_all_reach(void) //等待全部腿动作完成函数
{
  for (int i = 0; i < 4; i++)   //依次等待4条腿动作完成
    wait_reach(i);
}

/*
  - 从笛卡尔坐标系到极坐标转化
  - 数学模型2/2
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float & alpha, volatile float & beta, volatile float & gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/*
  - 用对应的极坐标控制舵机
  - 数学模型与事实相吻合的情况下
  - EEprom中存储的错误将被添加
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if (leg == 0)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  else if (leg == 1)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 2)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 3)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  //Serial.println(alpha);
  //Serial.println(beta);
  //Serial.println(gamma);
  servo[leg][0].write(alpha);  //设定对应腿上的舵机1旋转角度
  servo[leg][1].write(beta);   //设定对应腿上的舵机2旋转角度
  servo[leg][2].write(gamma);  //设定舵机3旋转角度
}
