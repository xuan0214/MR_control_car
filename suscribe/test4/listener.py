import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time
import smbus
import sys
import math
import json
import netifaces as ni

ip = ''
while ip == '':
    try:
        ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
    except:
        ip = ''
print(ip)
broker =  'test.mosquitto.org'

print("setup")

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# front left
F_IN1 = 27
F_IN2 = 22

# front right
F_IN3 = 17
F_IN4 = 18

# back left
B_IN1 = 13
B_IN2 = 12

#back right
B_IN3 = 5
B_IN4 = 6

#get i2c
bus = smbus.SMBus(1)
bus.write_byte_data(0x53, 0x2C, 0x0B)
value = bus.read_byte_data(0x53, 0x31)
value &= ~0x0F
value |= 0x0B
value |= 0x08
bus.write_byte_data(0x53, 0x31, value)
bus.write_byte_data(0x53, 0x2D, 0x08)

# 驅動板
GPIO.setup(F_IN1,GPIO.OUT)
GPIO.setup(F_IN2,GPIO.OUT)
GPIO.setup(F_IN3,GPIO.OUT)
GPIO.setup(F_IN4,GPIO.OUT)

GPIO.setup(B_IN1,GPIO.OUT)
GPIO.setup(B_IN2,GPIO.OUT)
GPIO.setup(B_IN3,GPIO.OUT)
GPIO.setup(B_IN4,GPIO.OUT)


# 設定馬達初始頻率
PWM_Freq = 40

pwmB_IN1 = GPIO.PWM(B_IN1, PWM_Freq)
pwmB_IN2 = GPIO.PWM(B_IN2, PWM_Freq)
pwmB_IN3 = GPIO.PWM(B_IN3, PWM_Freq)
pwmB_IN4 = GPIO.PWM(B_IN4, PWM_Freq)

pwmF_IN1 = GPIO.PWM(F_IN1, PWM_Freq)
pwmF_IN2 = GPIO.PWM(F_IN2, PWM_Freq)
pwmF_IN3 = GPIO.PWM(F_IN3, PWM_Freq)
pwmF_IN4 = GPIO.PWM(F_IN4, PWM_Freq)


# 設定馬達能量
pwmB_IN1.start(0)
pwmB_IN2.start(0)
pwmB_IN3.start(0)
pwmB_IN4.start(0)

pwmF_IN1.start(0)
pwmF_IN2.start(0)
pwmF_IN3.start(0)
pwmF_IN4.start(0)

t0 = time.perf_counter()
orig_v = 30
flag = 0
stop_flag = 0
acc_flag = 0
orig_speed = 25

#get accelarate
def getAxes(dir):
    bytes = bus.read_i2c_block_data(0x53, 0x32, 6)
    x = bytes[0] | (bytes[1] << 8)
    if(x & (1 << 16-1)):
        x = x - (1 << 16)
    y = bytes[2] | (bytes[3] << 8)
    if(y & (1 << 16-1)):
        y = y - (1 << 16)
    z = bytes[4] | (bytes[5] << 8)
    if(z & (1 << 16-1)):
        z = z - (1 << 16)
    x = x * 0.004
    y = y * 0.004
    z = z * 0.004
    x = x * 9.80665
    y = y * 9.80665
    z = z * 9.80665
    x = round(x, 4)
    y = round(y, 4)
    z = round(z, 4)
    if dir == 0:
        return x
    if dir == 1:
        return y
    if dir == 2:
        return z

# front
def MoveForward(x, acc):
    global orig_speed
    start_time = time.time()
    print('MoveForward ')

    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN3.ChangeDutyCycle(0)

    pwmB_IN2.ChangeDutyCycle(x)
    pwmB_IN4.ChangeDutyCycle(x)
    pwmF_IN2.ChangeDutyCycle(x)
    pwmF_IN4.ChangeDutyCycle(x)
    
    x_speed = getAxes(0) #m/s^2
    y_speed = getAxes(1) #m/s^2
    z_speed = getAxes(2) #m/s^2
    end_time = time.time()
    delta_time = end_time - start_time
    #print('delta = ' + str(delta_time))
    print('x_speed = ' + str(x_speed))
    print('y_speed = ' + str(y_speed))
    print('z_speed = ' + str(z_speed))
    if acc == 0 :
        speed = orig_speed
        #speed = (abs(x_speed) * delta_time * 100)
        orig_speed = speed
    elif acc == 1 :
        #speed = orig_speed + (abs(x_speed) * 100)
        speed = orig_speed + (abs(x_speed) * delta_time * 100)
        orig_speed = speed
    elif acc == 2 :
        #speed = orig_speed - (abs(x_speed) * 100)
        speed = orig_speed - (abs(x_speed) * delta_time * 100)
        orig_speed = speed
    print('speed = ' + str(speed))
    client.publish('speed', json.dumps(speed))

# back
def MoveBack(x, acc):
    global orig_speed
    start_time = time.time()
    print('MoveBack ')
    
    pwmB_IN2.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(0)

    pwmB_IN1.ChangeDutyCycle(x)
    pwmB_IN3.ChangeDutyCycle(x)
    pwmF_IN1.ChangeDutyCycle(x)
    pwmF_IN3.ChangeDutyCycle(x)

    x_speed = getAxes(0) #gs
    y_speed = getAxes(1) #gs
    z_speed = getAxes(2) #gs
    end_time = time.time()
    delta_time = end_time - start_time

    if acc == 0 :
        speed = orig_speed
        orig_speed = speed
    elif acc == 1 :
        speed = orig_speed + (abs(x_speed) * delta_time * 100)
        orig_speed = speed
    elif acc == 2 :
        speed = orig_speed - (abs(x_speed) * delt_time * 100)
        orig_speed = speed
    print('speed = ' + str(speed))
    client.publish('speed', json.dumps(speed))


# left
def MoveLeft(x, acc):
    global orig_speed
    start_time = time.time()
    print('MoveLeft() ')
    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN2.ChangeDutyCycle(0)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(x)

    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(0)
    pwmF_IN3.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(x)

    x_speed = getAxes(0) #gs
    y_speed = getAxes(1) #gs
    z_speed = getAxes(2) #gs
    end_time = time.time()
    delta_time = end_time - start_time

    num = abs(x_speed) * abs(x_speed) + abs(y_speed) * abs(y_speed)
    if acc == 0 :
        speed = orig_speed
        orig_speed = speed
    if acc == 1 :
        speed = orig_speed + (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed
    if acc == 2 :
        speed = orig_speed - (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed
    print('speed = ' + str(speed))
    client.publish('speed', json.dumps(speed))

# right
def MoveRight(x, acc):
    global orig_speed
    start_time = time.time()
    print('MoveRight()')
    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN2.ChangeDutyCycle(x)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(0)

    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(x)
    pwmF_IN3.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(0)

    x_speed = getAxes(0) #gs
    y_speed = getAxes(1) #gs
    z_speed = getAxes(2) #gs
    end_time = time.time()
    delta_time = end_time - start_time

    num = abs(x_speed) * abs(x_speed) + abs(y_speed) * abs(y_speed)
    if acc == 0 :
        speed = orig_speed
        orig_speed = speed
    if acc == 1 :
        speed = orig_speed + (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed
    if acc == 2 :
        speed = orig_speed - (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed
    print('speed = ' + str(speed))
    #payload = x
    client.publish('speed', json.dumps(speed))


def Stop(orig_v,flag,acc) :
    print(orig_v)

    if orig_v > 0 :
        orig_v -= 5
        if flag == 1 :
            MoveForward(orig_v,acc)

        elif flag == 2 :
            MoveLeft(orig_v,acc)

        elif flag == 3 :
            MoveRight(orig_v,acc)

        elif flag == 4 :
            MoveBack(orig_v,acc)
        
        print(orig_v)
    else :
        stop_flag = 0
        flag = 0


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+ str(rc))

    # write subscribe topic on on_connect
    # if we lose connect or re-connect
    # will re-subscribe
    client.subscribe("direction")
    

def on_message(client, userdata, msg):
    global orig_v
    global t0
    global flag
    global stop_flag
    global acc_flag
        
    print(msg.topic+ " "+ msg.payload.decode('utf-8'))
        
    if msg.payload.decode('utf-8') == 'forward' :
        flag = 1
        stop_flag = 0
        acc_flag = 0
        MoveForward(orig_v, acc_flag)

    elif msg.payload.decode('utf-8') == 'left' :
        flag = 2
        stop_flag = 0
        acc_flag = 0
        MoveLeft(orig_v, acc_flag)

    elif msg.payload.decode('utf-8') == 'right' :
        flag = 3
        stop_flag = 0
        acc_flag = 0
        MoveRight(orig_v, acc_flag)

    elif msg.payload.decode('utf-8') == 'back' :
        flag = 4
        stop_flag = 0
        acc_flag = 0
        MoveBack(orig_v, acc_flag)
    
    elif msg.payload.decode('utf-8') == 'stop' :
        t0 = time.perf_counter()
        stop_flag = 1
        Stop(orig_v,flag)

    
    if msg.payload.decode('utf-8') == 'add' :
        if orig_v < 100 :
            orig_v += 5
            if flag == 1 :
                acc_flag = 1
                MoveForward(orig_v, acc_flag)

            elif flag == 2 :
                acc_flag = 1
                MoveLeft(orig_v, acc_flag)

            elif flag == 3 :
                acc_flag = 1
                MoveRight(orig_v, acc_flag)

            elif flag == 4 :
                acc_flag = 1
                MoveBack(orig_v, acc_flag)

    elif msg.payload.decode('utf-8') == 'sub' :
        if orig_v > 0 :
            orig_v -= 5
            if flag == 1 :
                acc_flag = 2
                MoveForward(orig_v, acc_flag)

            elif flag == 2 :
                acc_flag = 2
                MoveLeft(orig_v, acc_flag)

            elif flag == 3 :
                acc_flag = 2
                MoveRight(orig_v, acc_flag)

            elif flag == 4 :
                acc_flag = 2
                MoveBack(orig_v, acc_flag)



print("t0 = ",t0,"\n","now = ",time.perf_counter(),"\n")

while True :
    print("t0 = ",t0,"\n","now = ",time.perf_counter(),"\n")
    if time.perf_counter() - t0 > 0.2 and stop_flag == 1 :
        print("t0 = ",t0,"\n","now = ",time.perf_counter(),"\n")
        t0 = time.perf_counter()
        Stop(orig_v,flag)
    break

# set connection and initialize
client = mqtt.Client()

# set connection action
client.on_connect = on_connect

# set recieve action
client.on_message = on_message

# set login username and password
#client.username_pw_set("try", "xxxx")

# set connection information (ip, port, connect time)

client.connect(broker, 1883)
print("connected");
# start connect
# it also can use the other loop function ot link
client.loop_forever()

