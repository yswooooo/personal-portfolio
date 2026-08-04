# ***电控(compiling...)***
***



## <u>*软件类*</u> ：

  - ### *C/C++ -> Makefile -> CMake* <br>

      - #### C语言：
        <u>[*【浙江大学】C语言入门与进阶 翁恺（全129讲）*](https://www.bilibili.com/video/BV1XZ4y1S7e1/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u>
      - #### C++：
        ①<u>[*【黑马程序员匠心之作|C++教程从0到1入门编程,学习编程不再难*]( https://www.bilibili.com/video/BV1et411b73Z/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb) </u> <br>
        ② <u>[*The Cherno C++*]( https://www.bilibili.com/video/BV1oD4y1h7S3/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb) </u>  
      - #### Makefile：
        ①<u>[*【Makefile 20分钟入门，简简单单，展示如何使用Makefile管理和编译C++代码】*](  https://www.bilibili.com/video/BV188411L7d2/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u>
         <u> <br>
        ②<u>[*【从零开始学Makefile】*]( https://www.bilibili.com/video/BV1Bv4y1J7QT/?p=14&share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u>
        
        ③<u>[*【GNU Makefile编译C/C++教程（Linux系统、VSCODE)】*](  https://www.bilibili.com/video/BV1EM41177s1/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u>
      - #### CMake：
        ①<u>[*【从零开始详细介绍CMake】*]( https://www.bilibili.com/video/BV1vR4y1u77h/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u> <br>
            <u> ②[*【新坑预警】相信我，我真的可以把CMake讲清楚】*]( https://www.bilibili.com/video/BV1Tw411s7Pk/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
             <u> ③ [*【技术】手把手教你写CMake一条龙教程——421施公队Clang出品*]( https://www.bilibili.com/video/BV16V411k7eF/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u>

 - ### *嵌入式开发*
      - #### 计算机速成课：
        <u>[ *Crash Course Computer Science*](https://www.bilibili.com/video/av21376839/?vd_source=ddae2b7332590050afe28928f52f0bda)</u>

    - 从零到一打造一台计算机：
    
       #### <u>①[*【编程前你最好了解的基本硬件和计算机基础知识 - (模拟电路)】* ](https://www.bilibili.com/video/BV1774114798/?share_source=copy_web)</u><br>
       #### <u>②[*【编程前你最好了解的基本硬件和计算机基础知识 - (数字电路)】* ]( https://www.bilibili.com/video/BV1Hi4y1t7zY/?share_source=copy_web)</u><br>
       #### <u>③[*【从0到1设计一台计算机】*](https://www.bilibili.com/video/BV1wi4y157D3/?share_source=copy_web)</u><br>


   - 开发流：(标准库：Keil5 -> VScode -> jscope 或 HAL库： CubeMX -> VScode -> Ozone)<br>
     
       > 开发软件可以只使用`keil5`配合`vscode`,或者使用`CubeMX``vscode``Ozone`,如果学标准库的化建议使用前者，只需在vscode简单安装一个插件就可以使用vscode阅读和编译代码，但是调试还是要到keil5；如果学习HAL库的话，当然也可以只使用前者的方法(标准库方法+CubeMX),后者开发方式更高阶而已
       >
       > jscope和Ozone都为示波器软件，能将数据图形化,简单而言就是将变量值在图上打点连线，最后呈现为图形
       
       <u>①[*【Keil+vscode合作 ，摆脱keil不友好的界面】*]( https://www.bilibili.com/video/BV13U4y1b7cd/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
       
       #### <u>①[*CubeMX+VSCode+Ozone的STM32开发工作流（一）背景知识介绍 - NeoAndrew的文章*](https://zhuanlan.zhihu.com/p/584903079)</u><br>
       
        #### <u>②[*CubeMX+VSCode+Ozone的STM32开发工作流（二）VSCode环境配置 - NeoAndrew的文章*](https://zhuanlan.zhihu.com/p/584912052)</u><br>
        #### <u>③[*CubeMX+VSCode+Ozone的STM32开发工作流（三）利用Ozone进行可视化调试和代码分析 - NeoAndrew的文章*](https://zhuanlan.zhihu.com/p/584915369)</u><br>
       
   - STM32: GPIO 中断 TIM USART通信 USB通信 SPI通信 IIC通信 C通信 FreeRTOS <br>
     
   - > F4与F1差不了太多，只有少许的库差别，学会F1再学F4没什么太大压力，当然开始就学F4也可以，只是F4的板子普遍较贵，初学者(小白)建议先购买简单的STM32F103C8T6去学习，价格也较低
     
       #### <u>①[*【单片机】野火STM32F103教学视频 (配套霸道/指南者/MINI)【全】(刘火良老师出品)*]( https://www.bilibili.com/video/BV1yW411Y7Gw/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
  #### <u>②[*【正点原子】 手把手教你学STM32入门教学视频单片机 嵌入式  之 F103-基于新战舰V3/精英/MINI板*]( https://www.bilibili.com/video/BV1Lx411Z7Qa/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
  #### <u>③[*【STM32入门教程-2023版 细致讲解 中文字幕】*](https://www.bilibili.com/video/BV1th411z7sn/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
   <u>④[*【野火 FreeRTOS视频教学 配套书籍《FreeRTOS内核实现与应用开发实战指南》配套例程源码 基于STM32开发板硬件教学 操作系统教学视频】*](https://www.bilibili.com/video/BV1Jx411X7NS/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
  
 - ### *控制系统模型仿真*
   - MATLAB/Simulink 

     <u>①[*【【包教会】MATLAB最新教程 零基础入门 手把手带你学习Matlab 有手就行 ！Up持续更新中！2023适用于数学建模|信号处理|控制系统|人工智能】 *](https://www.bilibili.com/video/BV1tA411o7wd/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
   
    - Webot


 - ### *代码版本管理*

      - Git -> Github / Gitee / Gitlab <br>
    #### <u>①[*【尚硅谷Git入门到精通全套教程 (涵盖GitHub\Gitee码云\GitLab)】*]( https://www.bilibili.com/video/BV1vy4y1s7k6/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
    #### <u>②[*【嵌入式必备工具Git】*]( https://www.bilibili.com/video/BV18u4y1p7Xj/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

## <u>*硬件类* </u>

- ### *硬件通识*<br>

<u>①[*【电控硬件通识part.1】*](https://www.bilibili.com/video/BV1RH4y1B7Af/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

<u>②[*【电控硬件通识part.2】*](https://www.bilibili.com/video/BV13b4y1g7of/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

<u>③[*【电控硬件通识part.3】*](https://www.bilibili.com/video/BV18u411F7zc/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

<u>④[*【电烙铁焊接教程（包含热风枪）】*](https://www.bilibili.com/video/BV12j41177Ec/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

 - ### *PCB设计*

    - Altium Designer 

      <u>①[*【Altium Designer 23|AD22|AD23新手入门必备课56讲|凡亿教育】 *](https://www.bilibili.com/video/BV11a411x75o/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

      <u>②[*【Altium Designer 1小时（貌似不够）速成（可能不止一小时*~* 但我觉得仨小时肯定够了---来自up猪的自信!!）】*]( https://www.bilibili.com/video/BV17E411x7dR/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

      <u>③[*【Altium Designer】原理图库和封装库的创建（接上一系列的视频） *](https://www.bilibili.com/video/BV1Jc411h73C/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

      <u>④[*【Altium Designer Rules】规则逐条详解（自封全B站最详细，画PCB必看）*](https://www.bilibili.com/video/BV11u4y1Z76b/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

      <u>⑤[*【AD库资源分享】 带你认识Altium Designer的原理图库、PCB封装库和集成库*](https://www.bilibili.com/video/BV1eo4y1L7KH/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

      

    - 立创EDA<br><u>①[*【嘉立创EDA-PCB设计零基础入门课程（持续更新中......）】*](https://www.bilibili.com/video/BV1fM411Z7cW/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

    - <u>②[*【【教程】零基础入门PCB设计-国一学长带你学立创EDA专业版 全程保姆级教学 中文字幕（持续更新中）】 *](https://www.bilibili.com/video/BV1At421h7Ui/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

    - <u>③[*【小白入门-如何使用立创EDA设计一个简单的PCB】 *](https://www.bilibili.com/video/BV1dU4y187fN/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>

 - ### *集成电路仿真*

   - Multisim

## <u>*控制系统设计*</u>
- ### *控制理论*
    #### <u>[*DR.CAN*](https://space.bilibili.com/230105574?spm_id_from=333.337.search-card.all.click)</u><br>
    
    <u>[*【中英字幕】关于控制理论你需要知道的一切丨Everything You Need to Know About Control Theory】*](https://www.bilibili.com/video/BV1Hg411673H/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
    
    <u>[*【Brian Douglas】理解 PID 控制 | Understanding PID Control】*](https://www.bilibili.com/video/BV1Ko4y1R7J9/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
    
    <u>[*【中英字幕】Brian Douglas Control Theroy | 自动控制原理】*](https://www.bilibili.com/video/BV1WT4y1M7rm/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
    
    <u>[*【Brian Douglas】现代控制理论 | State Space Control】*](https://www.bilibili.com/video/BV1iN411X7LF/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
    
    <u>[*【Brian Douglas】What Is System Identification?】*](https://www.bilibili.com/video/BV1Qu411k7B9/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
    
    <u>[*【Brian Douglas】线性系统辩识】*](https://www.bilibili.com/video/BV1oS4y1878E/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
    
    <u>[*【中英字幕】什么是模糊逻辑_模糊逻辑第 1 部分 | What Is Fuzzy Logic_Fuzzy Logic Part 1.】*](https://www.bilibili.com/video/BV1Wj411Y76i/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
    
    <u>[*【中英字幕】模糊推理系统_模糊逻辑第2部分 | Fuzzy Inference System Walkthrough_Fuzzy Logic Part 2.】*](https://www.bilibili.com/video/BV1oc411X7Rm/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
    
    <u>[*【中英字幕】模糊逻辑倒立摆示例_模糊逻辑第3部分 | Fuzzy Logic Examples_Fuzzy Logic Part 3.】*](https://www.bilibili.com/video/BV1hb4y1M7yC/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
    
    <u>[*【中英字幕】调整模糊逻辑控制器_模糊逻辑第4部分 | Fuzzy Logic Controller Tuning _Fuzzy Logic Part 4.】*](https://www.bilibili.com/video/BV19v411c7SA/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb)</u><br>
    
    
    
    ---

 

