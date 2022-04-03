# S-43-robot--Bluetooth-remote-control-spider-robot

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

  角度图片地址:https://gitee.com/lulendi/s-43-robot--bluetooth-remote-control-spider-robot/blob/master/images/%E8%84%9A%E8%A7%92%E5%BA%A6%E5%9B%BE.jpg


  --舵机站立时的角度姿态:舵机在接收站立命令时,为了保证三点支撑,会左侧或右侧两足平行,另一侧两足会呈现90°左右的夹角.这在RegisHsu的YouTube

  视频里有演示:https://gitee.com/lulendi/s-43-robot--bluetooth-remote-control-spider-robot/blob/master/video/Spider%20Robot%20v3.0%20demo%20(quadruped,%20quad%20robot).mp4

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

    // c5x: 挥手 x 时间

    // c6x: 挑衅 x times

![输入图片说明](panda.jpg)
