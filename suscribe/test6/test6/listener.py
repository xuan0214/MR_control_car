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
time.sleep(1)
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


dir_mode = 0
acc_mode = 0
orig_v = 30
orig_speed = 27
t0 = time.perf_counter()

#get accelarate
def getAxes(dir) :
    bytes = bus.read_i2c_block_data(0x53, 0x32, 6)
    x = bytes[0] | (bytes[1] << 8)
    if(x & (1 << 16-1)):
        x = x - (1 << 16)
    y = bytes[2] | (bytes[3] << 8)
    if(y & (1 << 16-1)):
        y = y - (1 << 16)

    x = x * 0.004 * 9.80665
    y = y * 0.004 * 9.80665

    x = round(x, 4)
    y = round(y, 4)

    if dir == 0 :
        return x

    if dir == 1 :
        return y


# front
def MoveForward(x, acc_mode) :
    global orig_speed
    global orig_v

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
    
    x_acc = getAxes(0) #m/s^2
    y_acc = getAxes(1) #m/s^2
    end_time = time.time()
    delta_time = end_time - start_time

    print('x_acc = ' + str(x_acc))
    print('y_acc = ' + str(y_acc))

    if x == 30 :
        speed = 27
        orig_speed = 27

    num = abs(x_acc) * abs(x_acc) + abs(y_acc) * abs(y_acc)
    if acc_mode == 0 :
        speed = orig_speed
        orig_speed = speed
    
    elif acc_mode == 1 :
        speed = orig_speed + (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed
    
    elif acc_mode == 2 :
        speed = orig_speed - (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed

    if x == 0 :
        speed = 0
        orig_v = 30 


    print('speed = ' + str(speed))
    client.publish('speed', json.dumps(speed))

# back
def MoveBack(x, acc_mode) :
    global orig_speed
    global orig_v

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

    x_acc = getAxes(0) #gs
    y_acc = getAxes(1) #gs

    end_time = time.time()
    delta_time = end_time - start_time

    if x == 30 :
        speed = 27
        orig_speed = 27

    num = abs(x_acc) * abs(x_acc) + abs(y_acc) * abs(y_acc)
    if acc_mode == 0 :
        speed = orig_speed
        orig_speed = speed
    
    elif acc_mode == 1 :
        speed = orig_speed + (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed
    
    elif acc_mode == 2 :
        speed = orig_speed - (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed

    if x == 0 :
        speed = 0
        orig_v = 30 

    print('speed = ' + str(speed))
    client.publish('speed', json.dumps(speed))


# left
def MoveLeft(x, acc_mode) :
    global orig_speed
    global orig_v

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

    x_acc = getAxes(0) #gs
    y_acc = getAxes(1) #gs

    end_time = time.time()
    delta_time = end_time - start_time

    if x == 30 :
        speed = 27
        orig_speed = 27

    num = abs(x_acc) * abs(x_acc) + abs(y_acc) * abs(y_acc)
    
    if acc_mode == 0 :
        speed = orig_speed
        orig_speed = speed
    
    elif acc_mode == 1 :
        speed = orig_speed + (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed
    
    elif acc_mode == 2 :
        speed = orig_speed - (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed

    if x == 0 :
        speed = 0
        orig_v = 30 

    print('speed = ' + str(speed))
    client.publish('speed', json.dumps(speed))

# right
def MoveRight(x, acc_mode) :
    global orig_speed
    global orig_v

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

    x_acc = getAxes(0) #gs
    y_acc = getAxes(1) #gs

    end_time = time.time()
    delta_time = end_time - start_time

    if x == 30 :
        speed = 27
        orig_speed = 27 

    num = abs(x_acc) * abs(x_acc) + abs(y_acc) * abs(y_acc)

    if acc_mode == 0 :
        speed = orig_speed
        orig_speed = speed
    
    elif acc_mode == 1 :
        speed = orig_speed + (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed
    
    elif acc_mode == 2 :
        speed = orig_speed - (pow(num, 1/2) * delta_time * 100)
        orig_speed = speed

    if x == 0 :
        speed = 0
        orig_v = 30 

    print('speed = ' + str(speed))
    client.publish('speed', json.dumps(speed))


def Stop() :
    global orig_v

    print('Stop()')
    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN2.ChangeDutyCycle(0)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(0)

    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(0)
    pwmF_IN3.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(0)

    speed = 0
    orig_v = 30

    print('speed = ' + str(speed))
    client.publish('speed', json.dumps(speed))

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+ str(rc))

    # write subscribe topic on on_connect
    # if we lose connect or re-connect
    # will re-subscribe
    client.subscribe("direction")
    
def on_message(client, userdata, msg):
    global orig_v
    global t0
    global dir_mode
    global acc_mode   
  
    print(msg.topic+ " "+ msg.payload.decode('utf-8'))
        
    if msg.payload.decode('utf-8') == 'forward' :
        dir_mode = 1
        acc_mode = 0
        MoveForward(orig_v, acc_mode)

    elif msg.payload.decode('utf-8') == 'left' :
        dir_mode = 2
        acc_mode = 0
        MoveLeft(orig_v, acc_mode)

    elif msg.payload.decode('utf-8') == 'right' :
        dir_mode = 3
        acc_mode = 0
        MoveRight(orig_v, acc_mode)

    elif msg.payload.decode('utf-8') == 'back' :
        dir_mode = 4
        acc_mode = 0
        MoveBack(orig_v, acc_mode)
    
    elif msg.payload.decode('utf-8') == 'stop' :
        Stop()
    
    if msg.payload.decode('utf-8') == 'add' :
        if orig_v < 100 :
            acc_mode = 1
            orig_v += 5
            if dir_mode == 1 :
                MoveForward(orig_v, acc_mode)

            elif dir_mode == 2 :
                MoveLeft(orig_v, acc_mode)

            elif dir_mode == 3 :
                MoveRight(orig_v, acc_mode)

            elif dir_mode == 4 :
                MoveBack(orig_v, acc_mode)

    elif msg.payload.decode('utf-8') == 'sub' :
        if orig_v > 0 :
            acc_mode = 2
            orig_v -= 5
            if dir_mode == 1 :
                MoveForward(orig_v, acc_mode)

            elif dir_mode == 2 :
                MoveLeft(orig_v, acc_mode)

            elif dir_mode == 3 :
                MoveRight(orig_v, acc_mode)

            elif dir_mode == 4 :
                MoveBack(orig_v, acc_mode)



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
client.publish('ip', json.dumps(ip), retain = True)
print(ip)

client.loop_forever()
