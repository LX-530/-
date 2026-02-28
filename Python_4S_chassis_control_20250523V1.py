import serial
import threading
import time
import math
import os,socket,struct
import subprocess
import mmap
import fcntl

IP = '192.168.3.18'

MMWR_PORT = "/dev/ttyTHS1"
MMWR_BAUD_RATE = 115200
SCS_MODE_ACKERMAN = 0
SCS_MODE_4MATIC_DIFF = 1
SCS_MODE_4MATIC_SAME = 2
motor_data_pandn = b'\x00'
SCS_mode = ''
Headlight_off = b'\x00'
Headlight_on = b'\x01'
scs_mode = None

# 传动比
TRANS_RATIO = 6.287
# 轮胎半径
WHEEL_RADIUS = 0.033 
# 占空比安全限幅(浮点型)
MIN_DUTY = -0.2
MAX_DUTY = 0.2
# 电机转速安全限幅（整型）
MIN_RPM = 900
MAX_RPM = 2500

class Motor:
    def __init__(self):
        self.motor = 0.0
        self.last_update_time = time.time()
        self.timeout = 2  #超时时间（秒）

    @staticmethod
    def _vel2duty(vel):
        rpm = vel * 60 * TRANS_RATIO / (2 * 3.14 * WHEEL_RADIUS)
        duty = (abs(rpm) - 100) * 0.11 / 1200 + 0.01 if rpm > 0 else -((abs(rpm) - 100) * 0.11 / 1200 + 0.01)
        return duty
    
    def writeDuty(self, duty):
        self.motor = duty
        self.last_update_time = time.time()

    def writeRPM(self, rpm):
        self.motor = rpm
        self.last_update_time = time.time()

    def writeVelocity(self, velocity):
        self.motor = Motor._vel2duty(velocity)
        self.last_update_time = time.time()

    def check_timeout(self):
        if time.time() - self.last_update_time > self.timeout:
            self.motor = 0.0

# 舵机最小转向角度，角度制->弧度制
MIN_DELTA: float = math.radians(-20) 
# 舵机最大转向角度，角度制->弧度制
MAX_DELTA: float = math.radians(20) 
# detla转position的系数
COEFFICIENT: int = 1
# 舵机中间角度位置
CENTER_POSITION: int = 1500
# 舵机最小角度位置
MIN_POSITION: int = 800 
# 舵机最大角度位置
MAX_POSITION: int = 2200 



class Servo:
    def __init__(self):
        self.servo : int = 1500
        self._h_positionRange: float = (MAX_POSITION - MIN_POSITION) / 2
        self._h_deltaRange: float = (MAX_DELTA - MIN_DELTA) / 2     # 弧度制

    def position_write(self, position):
        self.servo = position

    def delta_write(self, delta):
        delta = delta if delta > MIN_DELTA else MIN_DELTA
        delta = delta if delta < MAX_DELTA else MAX_DELTA
        self.servo = int(COEFFICIENT * ((delta - 0) / self._h_deltaRange 
                                                 * self._h_positionRange) + CENTER_POSITION)

# 串口创建函数
def openMMWPort(port=MMWR_PORT, baudRate=MMWR_BAUD_RATE):
    try:
        uart = serial.Serial(port, baudRate, timeout=5)
    except Exception as error:
        print("Failed to open serial port\n", error)
        exit(0)
    else:
        return uart #返回串口类对象

def send_data_import(uart: serial.Serial,motor_value,scs_steering,scs_mode,Headlight):
    protocol_header = b'\xFF' #协议头
    protocol_ender = b'\xFE'  #协议尾
    if motor_value<0:
        motor_value = -motor_value
        motor_data_pandn = b'\x01'       #电机数据正负符号位（00为正数，01为负数）
    else:
        motor_data_pandn = b'\x00'
        
    motor_input = motor_data_deal(motor_value)
    steering_input = scs_data_deal(scs_steering)

    data_packet = protocol_header  + motor_data_pandn + motor_input + steering_input + scs_mode + Headlight + protocol_ender

    try:
        uart.write(data_packet) #发送数据包
    except Exception as exception_error:
        print("Error occurred. Exiting Program")
        print("Error: " + str(exception_error)) 

def receive_data(uart: serial.Serial):
    data_length = 51  # 增加到 51 个字节
    state_num = 0
    global SCS_mode, headlight, taillight , STSturn_mode

    addr = (IP, 5000)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        try:
            # 使用状态机进行中断接收数据
            if state_num == 0:
                if uart.is_open and uart.in_waiting > 0:  # 未检测到数据时串口没有字节缓存
                    data_buffer = []  # 初始化缓存区，并在循环过程中清除缓存
                    Read_buffer = uart.read()  # 首先读取一个字节
                    if Read_buffer == b'\xff':  # 判断数据帧头
                        state_num = 1
                        data_buffer = uart.read(data_length - 1)  # 读取数据内容，接收除数据帧头以外的剩余字节
                        if data_buffer[data_length - 2] == 254:  # 判断数据帧尾

                            fmt1 = '<BBBBB'

                            struct_modedata = data_buffer[0:5]
                            
                            turn_mode , turn_light , head_light , tail_light , RPM_dir  = struct.unpack(fmt1,struct_modedata)

                            if turn_mode == 0: 
                                STSturn_mode = '阿克曼转向'
                            elif turn_mode == 1:
                                STSturn_mode = '四轮异向'
                            elif turn_mode == 2:
                                STSturn_mode = '四轮同向'

                            if turn_light == 0:
                                    turn_light_mode ='关闭'
                            elif turn_light == 1:
                                    turn_light_mode ='右转'
                            elif turn_light == 2:
                                    turn_light_mode ='左转'

                            if head_light == 0:
                                    headlight_mode ='关闭'
                            elif head_light == 1:
                                    headlight_mode ='打开'
                            
                            if tail_light == 0:
                                    taillight_mode ='关闭'
                            elif tail_light == 1:
                                    taillight_mode ='打开'

                            if RPM_dir == 0:
                                    RPM_dir_mode ='正转'
                            elif RPM_dir == 1:
                                    RPM_dir_mode ='反转'
                                


                            # print("转向模式:",STSturn_mode,  "转向灯:",turn_light_mode,  "大灯:",headlight_mode,  "尾灯:",taillight_mode,  "电机旋转方向:",RPM_dir_mode )

                            fmt2 = '<Hffffff'

                            struct_VESCdata = data_buffer[5:31]

                            Encoder_rpm , rpm ,VESC_volt , Temp_mos , Temp_motor , current_motor ,  duty= struct.unpack(fmt2,struct_VESCdata)

                            # print(f"电机电压:{VESC_volt:.2f} V, "
                            #       f"Mos温度:{Temp_mos:.2f} ℃ , "
                            #       f"电机温度:{Temp_motor:.2f} ℃ , "
                            #       f"电机电流:{current_motor:.2f} A, "
                            #       f"电机转速:{rpm:.2f} RPM, "
                            #       f"编码器转速:{Encoder_rpm:.2f} RPS, "
                            #       f"占空比:{duty*100:.2f} %")

                            fmt3 = '<fBbHb'

                            struct_sts1data = data_buffer[31:40]

                            struct_sts2data = data_buffer[40:49]

                            sts1_volt , sts1_current , sts1_temp ,  sts1_pos , sts1_ts= struct.unpack(fmt3,struct_sts1data)
                            
                            sts2_volt , sts2_current , sts2_temp ,  sts2_pos , sts2_ts= struct.unpack(fmt3,struct_sts2data)
                            
                            # print("前舵机电压：", sts1_volt ,"V","前舵机电流：", sts1_current*6.5 ,"mA","前舵机温度：", sts1_temp ,"℃","前舵机位置：",  sts1_pos , "前舵机微调值：", sts1_ts )

                            # print("后舵机电压：", sts2_volt ,"V","后舵机电流：", sts2_current*6.5 ,"mA","后舵机温度：", sts2_temp ,"℃","后舵机位置：",  sts2_pos , "后舵机微调值：", sts2_ts )

                            Rpm = data_buffer[1] * 256 + data_buffer[0]
     
                            
                            date = struct.pack('dd', sts1_ts , sts2_ts)
                            s.sendto(date, addr)

                            # 状态信息打印
                            status_header = f"{'转向模式':<8}: {STSturn_mode}"
                            status_info = (
                                f"{'转向灯':<8}: {turn_light_mode:<6}"
                                f"{'大灯':<6}: {headlight_mode:<6}"
                                f"{'尾灯':<6}: {taillight_mode:<6}"
                                f"{'电机旋转方向':<10}: {RPM_dir_mode}"
                            )
                            print("\n" + "="*50)
                            print(status_header)
                            print(status_info)

                            # 电机数据打印（带单位对齐）
                            motor_info = (
                                f"\n{'电机电压':<8}: {VESC_volt:<6.2f} V | "
                                f"{'Mos温度':<10}: {Temp_mos:<8.2f} ℃ | "
                                f"{'电机温度':<9}: {Temp_motor:<6.2f} ℃\n"
                                f"{'电机电流':<8}: {current_motor:<6.2f} A | "
                                f"{'电机转速':<8}: {rpm:<6.2f} RPM | "
                                f"{'编码器转速':<8}: {Encoder_rpm:<6.2f} RPS\n"
                                f"{'占空比':<9}: {duty*100:<6.2f} % | "
                            )
                            print("-"*50)
                            print(motor_info)

                            # 舵机数据打印（表格化对齐）
                            # 舵机数据表格模板（修正对齐）
                            servo_template = "│ {:^7.2f} V │ {:^6.1f} mA │ {:^5}℃ │ {:^7} │ {:^7} │"  # 使用^居中，>右对齐

                            print("\n" + "="*56)
                            print(f"{' 前舵机数据 ':^56}")
                            print("┌───────────┬───────────┬────────┬─────────┬─────────┐")
                            print("│  电压     │   电流    │  温度  │  位置   │ 微调值  │")
                            print("├───────────┼───────────┼────────┼─────────┼─────────┤")
                            print(servo_template.format(
                                sts1_volt,
                                sts1_current*6.5,
                                sts1_temp,
                                sts1_pos,
                                sts1_ts+1500
                            ))
                            print("└───────────┴───────────┴────────┴─────────┴─────────┘")

                            print(f"\n{' 后舵机数据 ':^56}")
                            print("┌───────────┬───────────┬────────┬─────────┬─────────┐")
                            print("│  电压     │   电流    │  温度  │  位置   │ 微调值  │")
                            print("├───────────┼───────────┼────────┼─────────┼─────────┤")
                            print(servo_template.format(
                                sts2_volt,
                                sts2_current*6.5,
                                sts2_temp,
                                sts2_pos,
                                sts2_ts+1500
                            ))
                            print("└───────────┴───────────┴────────┴─────────┴─────────┘")
                            print("="*56) 

                            break  # 读取到一帧数据后跳出循环
                        else:
                            state_num = 0  # 读取数据出错重新读取
                    else:
                        state_num = 0  # 未读取到数据帧头重新读取
        except KeyboardInterrupt:
            uart.close()
            exit(0)

# 电机限幅及数据转码
def motor_data_deal(motor_value):
    if isinstance(motor_value, float): #占空比限幅[MIN_DUTY , MAX_DUTY]
        motor_value = motor_value if motor_value>=MIN_DUTY else MIN_DUTY
        motor_value = motor_value if motor_value<=MAX_DUTY else MAX_DUTY
        motor_value = int(motor_value*100)
        motor_input = motor_value.to_bytes(2,byteorder='little')
    elif isinstance(motor_value, int): #转速限幅[MIN_RPM , MAX_RPM]
        motor_value = motor_value if motor_value>=MIN_RPM else MIN_RPM
        motor_value = motor_value if motor_value<=MAX_RPM else MAX_RPM
        motor_input = motor_value.to_bytes(2,byteorder='little')
    else:
        print("电机值数据类型有误！！！请确保电机值为占空比[<1.4](浮点型)或转速[900rpm-3500rpm](整型)")
        exit(0)
    return motor_input

# 舵机限幅保护及数据转码
def scs_data_deal(scs_steering):
    if isinstance(scs_steering, int):
         # 舵机限幅处理
        steering_input = scs_steering if scs_steering> MIN_POSITION else MIN_POSITION
        steering_input = scs_steering if scs_steering< MAX_POSITION else MAX_POSITION
        steering_input = scs_steering.to_bytes(2,byteorder='little')
    else:
        print("舵机值数据类型有误！！！请确保舵机值为[800,2200](整型)")
        exit(0)
    return steering_input

# 底盘赋权
def set_serial_permission(device="/dev/ttyTHS1", password="123"):
    try:
        # 使用 echo + sudo -S 从标准输入读取密码
        cmd = f"echo {password} | sudo -S chmod 777 {device}"
        subprocess.run(cmd, shell=True, check=True)
        # print(f"✅ 成功s {device} 权限为 777")
    except subprocess.CalledProcessError as e:
        print(f"❌ 错误: {e}")

def thread_function_1():
    while True:
        try:
            # 从文件读取电机值,舵机值
            int_val, float_val = read_data()
          
            if MIN_POSITION <= int_val <= MAX_POSITION:
                servoObj.position_write(int_val)
                # print("Received servo position is:", int_val)
            else:
                print("Received servo position out of range:", int_val)
            
            if MIN_DUTY <= float_val <= MAX_DUTY:
                motorObj.writeDuty(float_val)
            else:
                print("Received motor value out of range:", float_val)
        
        
        except struct.error as e:
            print(f"Data unpack failed (check DATA_FORMAT consistency): {e}")
            continue  # 跳过异常，继续循环
        except FileNotFoundError:
            print("File was deleted during read, will recreate on next loop.")
            continue  
        except Exception as e:
            print(f"Unexpected error: {e}")
            continue  # 避免线程退出


            # with open(SERVO_POSITION_FILE_PATH, 'r') as f:
            #     try:
            #         servo_position = int(f.read().strip())  # 假设文件中是字符串格式的浮点数
            #         if MIN_POSITION <= servo_position <= MAX_POSITION:
            #             servoObj.position_write(servo_position)
            #         else:
            #             print("Received servo position out of range:", servo_position)
            #     except ValueError:
            #         print("Invalid data in file. Expected a int.")

def thread_function_2():
    while True:  # 主循环
        send_data_import(Uart, motorObj.motor, servoObj.servo, scs_mode=scs_mode_byte, Headlight=Headlight_off)

def thread_function_3():
    while True:  # 主循环
        receive_data(Uart)


def meminit():
    """确保共享内存文件存在且大小正确"""
    if not os.path.exists(SHM_FILE):
        # 创建文件并初始化大小
        with open(SHM_FILE, 'wb') as f:
            # 直接预分配足够空间（无需检查大小）
            f.truncate(DATA_SIZE)  # 分配空间
            packed_data = struct.pack(DATA_FORMAT, *INITIAL_VALUE)  # 解包元组
            f.write(packed_data)
        print(f"Created mmap file with size {DATA_SIZE}B")
    else:
        # 检查现有文件大小
        current_size = os.path.getsize(SHM_FILE)
        if current_size < DATA_SIZE:
            # 扩展文件大小
            with open(SHM_FILE, 'r+b') as f:
                fcntl.flock(f, fcntl.LOCK_EX)
                f.truncate(DATA_SIZE)
                f.flush()
                print(f"Extended file size from {current_size}B to {DATA_SIZE}B")
                fcntl.flock(f, fcntl.LOCK_UN)

def read_data():
    """读取数据（原子操作）"""
    try:
        with open(SHM_FILE, 'r+b') as f:
            fcntl.flock(f, fcntl.LOCK_SH)  # 共享锁
            mm = mmap.mmap(f.fileno(), DATA_SIZE, access=mmap.ACCESS_READ)
            data = mm.read(DATA_SIZE)
            mm.close()
            fcntl.flock(f, fcntl.LOCK_UN)
        return struct.unpack(DATA_FORMAT, data)
    except FileNotFoundError:
        print("共享内存文件不存在，正在重新初始化...")
        meminit()
        return (INITIAL_VALUE, 0.0)  # 返回默认值
    

def write_data(int_val=None, float_val=None):
    """
    原子更新数据（允许仅更新部分字段）
    - int_val: 要更新的整型值（None 表示不更新）
    - float_val: 要更新的浮点值（None 表示不更新）
    """
    with open(SHM_FILE, 'r+b') as f:
        fcntl.flock(f, fcntl.LOCK_EX)  # 排他锁
        mm = mmap.mmap(f.fileno(), DATA_SIZE, access=mmap.ACCESS_WRITE)
        # 读取当前值
        current_data = mm.read(DATA_SIZE)
        current_int, current_float = struct.unpack(DATA_FORMAT, current_data)
        # 合并新值（保留未更新的字段）
        new_int = current_int if int_val is None else int_val
        new_float = current_float if float_val is None else float_val
        
        # 写入更新后的完整数据
        mm.seek(0)
        mm.write(struct.pack(DATA_FORMAT, new_int, new_float))
        mm.flush()  # 确保数据同步到文件
        mm.close()
        
        fcntl.flock(f, fcntl.LOCK_UN)


if __name__ == "__main__":
    set_serial_permission()
    Uart = openMMWPort(port=MMWR_PORT, baudRate=MMWR_BAUD_RATE)  # 打开串口
    motorObj = Motor()
    servoObj = Servo()

    SHM_FILE = "/dev/shm/servo_shared.dat"
    DATA_FORMAT = 'if'  # 格式：'i'（4字节整型） + 'f'（4字节单精度浮点） → 总8字节
    INITIAL_VALUE  = (0, 0.00)  # 初始值
    INITIAL_SIZE = DATA_SIZE  = struct.calcsize(DATA_FORMAT)  # 初始化文件大小 

    DEFAULT_POSITION = 1500  # 默认位置

    meminit()

    while True:
        scs_mode = None  # 初始化 scs_mode

        # 示例：设置电机和舵机值
        # motorObj.writeDuty(0.08)  # 设置占空比
        # servoObj.position_write(1300)  # 设置舵机位置

        # 根据 scs_mode 的值选择对应的字节
        if scs_mode == SCS_MODE_ACKERMAN:
            scs_mode_byte = b'\x00'
        elif scs_mode == SCS_MODE_4MATIC_DIFF:
            scs_mode_byte = b'\x01'
        elif scs_mode == SCS_MODE_4MATIC_SAME:
            scs_mode_byte = b'\x02'
        else:
            print("Unknown scs_mode value: %d", scs_mode)
            scs_mode_byte = b'\x00'  # 默认值

        # 创建两个线程
        thread1 = threading.Thread(target=thread_function_1)
        thread2 = threading.Thread(target=thread_function_2)
        thread3 = threading.Thread(target=thread_function_3)

        # 启动两个线程
        thread1.start()
        thread2.start()
        thread3.start()

        # 等待两个线程完成
        thread1.join()
        thread2.join()
        thread3.join()

        print("两个线程都已完成。")
        motorObj.check_timeout()
        # time.sleep(0.002)  # 控制循环频率
