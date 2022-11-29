import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time
import json
import netifaces as ni
'''
*******************
'''
ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
broker =  'test.mosquitto.org'
'''
*******************
'''
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


# front
def MoveForward(x):
    print('MoveForward ')

    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN3.ChangeDutyCycle(0)

    pwmB_IN2.ChangeDutyCycle(x)
    pwmB_IN4.ChangeDutyCycle(x)
    pwmF_IN2.ChangeDutyCycle(x)
    pwmF_IN4.ChangeDutyCycle(x)


    payload = x
    client.publish('speed', json.dumps(payload)) 

# back
def MoveBack(x):
    print('MoveBack ')

    pwmB_IN2.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(0)

    pwmB_IN1.ChangeDutyCycle(x)
    pwmB_IN3.ChangeDutyCycle(x)
    pwmF_IN1.ChangeDutyCycle(x)
    pwmF_IN3.ChangeDutyCycle(x)

    payload = x
    client.publish('speed', json.dumps(payload))


# left
def MoveLeft(x):
    print('MoveLeft() ')
    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN2.ChangeDutyCycle(0)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(x)

    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(0)
    pwmF_IN3.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(x)


    payload = x
    client.publish('speed', json.dumps(payload))

# right
def MoveRight(x):
    print('MoveRight()')
    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN2.ChangeDutyCycle(x)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(0)

    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(x)
    pwmF_IN3.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(0)

    payload = x
    client.publish('speed', json.dumps(payload))


def Stop(orig_v,flag) :
    print(orig_v)

    if orig_v > 0 :
        orig_v -= 5
        if flag == 1 :
            MoveForward(orig_v)

        elif flag == 2 :
            MoveLeft(orig_v)

        elif flag == 3 :
            MoveRight(orig_v)

        elif flag == 4 :
            MoveBack(orig_v)
        
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
        
    print(msg.topic+ " "+ msg.payload.decode('utf-8'))
        
    if msg.payload.decode('utf-8') == 'forward' :
        flag = 1
        stop_flag = 0
        MoveForward(orig_v)

    elif msg.payload.decode('utf-8') == 'left' :
        flag = 2
        stop_flag = 0
        MoveLeft(orig_v)

    elif msg.payload.decode('utf-8') == 'right' :
        flag = 3
        stop_flag = 0
        MoveRight(orig_v)

    elif msg.payload.decode('utf-8') == 'back' :
        flag = 4
        stop_flag = 0
        MoveBack(orig_v)
    
    elif msg.payload.decode('utf-8') == 'stop' :
        t0 = time.perf_counter()
        stop_flag = 1
        Stop(orig_v,flag)

    
    if msg.payload.decode('utf-8') == 'add' :
        if orig_v < 100 :
            orig_v += 5
            if flag == 1 :
                MoveForward(orig_v)

            elif flag == 2 :
                MoveLeft(orig_v)

        elif flag == 3 :
            MoveRight(orig_v)

        elif flag == 4 :
            MoveBack(orig_v)

    elif msg.payload.decode('utf-8') == 'sub' :
        if orig_v > 0 :
            orig_v -= 5
            if flag == 1 :
                MoveForward(orig_v)

            elif flag == 2 :
                MoveLeft(orig_v)

            elif flag == 3 :
                MoveRight(orig_v)

            elif flag == 4 :
                MoveBack(orig_v)



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
'''
*****************
'''
client.publish('ip', json.dumps(ip), retain = True)
print(ip)
'''
*****************
'''
try:
    client.loop_forever()
except:
    client.publish('ip', payload = None, retain = True)

