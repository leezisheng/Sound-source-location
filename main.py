# ================================= 导入库文件 =====================================
# 麦克风相关的库
from Maix import MIC_ARRAY as mic
# LCD和延时相关的库
import lcd,time
# 数学计算相关的库
import math
# 延时和PWM输出相关的库
from machine import Timer,PWM

# ================================== 全局变量 =====================================

# 声源方位变量：X坐标，Y坐标，强度，角度
X_Axis   = 0
Y_Axis   = 0
Strength = 0
Angle    = 0

# x轴卡尔曼滤波相关变量
# Q为这一轮的心里的预估误差 X_Axis_Q = 0.1 X_Axis_R = 0.4
X_Axis_Q = 0.07
# R为下一轮的测量误差
X_Axis_R = 0.4
# Accumulated_Error为上一轮的估计误差，具体呈现为所有误差的累计
X_Axis_Accumulated_Error = 1
# 初始旧值
X_Axis_kalman_adc_old = 0

# x轴卡尔曼滤波相关变量
# Q为这一轮的心里的预估误差 Y_Axis_Q = 0.1 Y_Axis_R = 0.4
Y_Axis_Q = 0.07
# R为下一轮的测量误差
Y_Axis_R = 0.4
# Accumulated_Error为上一轮的估计误差，具体呈现为所有误差的累计
Y_Axis_Accumulated_Error = 1
# 初始旧值
Y_Axis_kalman_adc_old = 0

# 均值滤波相关变量
Angle_List   = [0,0,0,0,0,0,0,0,0,0]
Angle_Count  = 0
Angle_Sum    = 0

# ================================== 初始化配置 ===================================
# 麦克风初始化
mic.init(i2s_d0=34, i2s_d1=8, i2s_d2=33, i2s_d3=9, i2s_ws=32, i2s_sclk=10,\
            sk9822_dat=7, sk9822_clk=35)#可自定义配置 IO
#PWM 通过定时器配置，接到 IO17 引脚
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
S1 = PWM(tim, freq=50, duty=0, pin=17)
# LCD屏幕初始化
lcd.init()
# 显示液晶初始化
lcd.fill_rectangle(46,5, 230, 4, (255, 0, 0))    #上边线
lcd.fill_rectangle(46,5, 4, 230, (255, 0, 0))    #左边线
lcd.fill_rectangle(46,235, 230, 4, (255, 0, 0))    #下边线
lcd.fill_rectangle(276,5, 4, 234, (255, 0, 0))   #右边线
lcd.fill_rectangle(237,5, 4, 234, (255, 0, 0))    #右边线2
lcd.fill_rectangle(0,77, 47, 4, (255, 0, 0))    #左区域上边线
lcd.fill_rectangle(0,154, 47, 4, (255, 0, 0))    #左区域下边线
lcd.fill_rectangle(0,77, 4, 77, (255, 0, 0))    #左区域左边线

# ================================== 功能函数 =====================================

# 获取声音信号X坐标，Y坐标，强度，角度
def get_mic_dir():
    AngleX=0
    AngleY=0
    AngleR=0
    Angle=0
    AngleAddPi=0
    mic_list=[]
    imga = mic.get_map()    # 获取声音源分布图像
    b = mic.get_dir(imga)   # 计算、获取声源方向
    for i in range(len(b)):
        if b[i]>=2:
            AngleX+= b[i] * math.sin(i * math.pi/6)
            AngleY+= b[i] * math.cos(i * math.pi/6)

    AngleX=round(AngleX,6) #计算坐标转换值
    AngleY=round(AngleY,6)
    AngleX = X_Axis_kalman(AngleX)
    AngleY = Y_Axis_kalman(AngleY)

    if AngleY<0:AngleAddPi=180
    if AngleX<0 and AngleY > 0:AngleAddPi=360
    if AngleX!=0 or AngleY!=0: #参数修正
        if AngleY==0:
            Angle=90 if AngleX>0 else 270 #填补X轴角度
        else:
            Angle=AngleAddPi+round(math.degrees(math.atan(AngleX/AngleY)),4) #计算角度
        AngleR=round(math.sqrt(AngleY*AngleY+AngleX*AngleX),4) #计算强度
        mic_list.append(AngleX)
        mic_list.append(AngleY)
        mic_list.append(AngleR)
        mic_list.append(Angle)
    a = mic.set_led(b,(0,0,255))# 配置 RGB LED 颜色值
    return AngleX,AngleY,AngleR,Angle

# 数据输出
def Printf_Data():
    global X_Axis
    global Y_Axis
    global Strength
    global Angle
    print("X_Axis:",X_Axis,"Y_Axis:",Y_Axis,"Strength:",Strength,"Angle:",Angle)

# 卡尔曼滤波 x轴
def X_Axis_kalman(ADC_Value):
    global X_Axis_kalman_adc_old
    global X_Axis_Accumulated_Error

    # 新的值相比旧的值差太大时进行跟踪
    if (abs(ADC_Value-X_Axis_kalman_adc_old) > 2):
        Old_Input = ADC_Value*0.5 + X_Axis_kalman_adc_old*0.5
    else:
        Old_Input = X_Axis_kalman_adc_old

    # 上一轮的 总误差=累计误差^2+预估误差^2
    Old_Error_All = (X_Axis_Accumulated_Error**2 + X_Axis_Q**2)**(1/2)

    # R为这一轮的预估误差
    # H为利用均方差计算出来的双方的相信度
    H = Old_Error_All**2/(Old_Error_All**2 + X_Axis_R**2)

    # 旧值 + 1.00001/(1.00001+0.1) * (新值-旧值)
    kalman_adc = Old_Input + H * (ADC_Value - Old_Input)

    # 计算新的累计误差
    X_Axis_Accumulated_Error = ((1 - H)*Old_Error_All**2)**(1/2)
    # 新值变为旧值
    X_Axis_kalman_adc_old = kalman_adc
    return kalman_adc

# 卡尔曼滤波 y轴
def Y_Axis_kalman(ADC_Value):
    global Y_Axis_kalman_adc_old
    global Y_Axis_Accumulated_Error

    # 新的值相比旧的值差太大时进行跟踪
    if (abs(ADC_Value-Y_Axis_kalman_adc_old) > 2):
        Old_Input = ADC_Value*0.5 + Y_Axis_kalman_adc_old*0.5
    else:
        Old_Input = Y_Axis_kalman_adc_old

    # 上一轮的 总误差=累计误差^2+预估误差^2
    Old_Error_All = (Y_Axis_Accumulated_Error**2 + Y_Axis_Q**2)**(1/2)

    # R为这一轮的预估误差
    # H为利用均方差计算出来的双方的相信度
    H = Old_Error_All**2/(Old_Error_All**2 + Y_Axis_R**2)

    # 旧值 + 1.00001/(1.00001+0.1) * (新值-旧值)
    kalman_adc = Old_Input + H * (ADC_Value - Old_Input)

    # 计算新的累计误差
    Y_Axis_Accumulated_Error = ((1 - H)*Old_Error_All**2)**(1/2)
    # 新值变为旧值
    Y_Axis_kalman_adc_old = kalman_adc
    return kalman_adc

# 舵机控制函数
# 180 度舵机：angle:-90 至 90 表示相应的角度
def Servo(servo,angle):
    if(servo == "S1"):
        global S1
        S1.duty((angle+90)/180*10+2.5)

# 均值滤波
# Angle_List   = [0,0,0,0,0,0,0,0,0,0]
# Angle_Count  = 0
# Angle_Sum    = 0
def Average_Filter(NowData):
    global Angle_Sum
    global Angle_Count
    global Angle_List

    Angle_Sum = 0

    if(Angle_Count>=9):
        Angle_Count = 0
    else:
        Angle_Count = Angle_Count + 1

    Angle_List[Angle_Count] = NowData

    for i in Angle_List:
        Angle_Sum = Angle_Sum + i

    average_data = Angle_Sum/10

    return average_data

# LCD屏幕显示函数
def LCD_Show(temp_X_Axis,temp_Y_Axis,temp_Strength,temp_Angle):
    # 文字显示角度
    lcd.draw_string(60, 200, "Angle: " + str(temp_Angle), lcd.BLUE, lcd.BLACK)
    # 文字显示距离
    lcd.draw_string(60, 180, "Distance: " + str(math.sqrt(temp_X_Axis*temp_X_Axis+temp_Y_Axis*temp_Y_Axis)), lcd.BLUE, lcd.BLACK)
    lcd.fill_rectangle(251,10, 25, 225, (0, 0, 0))    #清空右边的区域
    if(temp_Angle >= -30 or temp_Angle <= 30):
        temp_Angle = 30
    lcd.fill_rectangle(251,int(108+math.tan(temp_Angle*math.pi/180)*165), 15, 15, (0, 255,200))#角度位置实时


# ================================== 模式函数 =====================================


# ================================== 主函数 ======================================

# 舵机初始化
Servo("S1",-90)
time.sleep_ms(2000)
Servo("S1",90)
time.sleep_ms(2000)
Servo("S1",0)
time.sleep_ms(2000)

while True:
    X_Axis,Y_Axis,Strength,Angle = get_mic_dir()

    Angle = -(Angle - 210)

    if Angle >= 90:
        Angle = 90
    elif Angle <= -90:
        Angle = -90

    LCD_Show(X_Axis,Y_Axis,Strength,Angle)

    if Angle<0:
        Angle = Average_Filter(Angle)-15
    else:
        Angle = Average_Filter(Angle)-15

    Servo("S1",int(Angle))
    Printf_Data()
    print(Angle_List)
    time.sleep_ms(200)
