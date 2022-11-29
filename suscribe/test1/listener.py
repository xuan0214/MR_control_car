import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time

broker =  'broker.emqx.io'

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
PWM_Freq = 60

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


# front
def MoveForward():
    print('MoveForward ')

    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN3.ChangeDutyCycle(0)

    pwmB_IN2.ChangeDutyCycle(80)
    pwmB_IN4.ChangeDutyCycle(80)
    pwmF_IN2.ChangeDutyCycle(80)
    pwmF_IN4.ChangeDutyCycle(80)


# back
def MoveBack():
    print('MoveBack ')

    pwmB_IN2.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(0)

    pwmB_IN1.ChangeDutyCycle(80)
    pwmB_IN3.ChangeDutyCycle(80)
    pwmF_IN1.ChangeDutyCycle(80)
    pwmF_IN3.ChangeDutyCycle(80)

# stop
def Stop():
    print('Stop() ')
    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN2.ChangeDutyCycle(0)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(0)

    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(0)
    pwmF_IN3.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(0)

# left
def MoveLeft():
    print('MoveLeft() ')
    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN2.ChangeDutyCycle(0)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(80)

    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(0)
    pwmF_IN3.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(80)



# right
def MoveRight():
    print('MoveRight()')
    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN2.ChangeDutyCycle(80)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(0)

    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(80)
    pwmF_IN3.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(0)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+ str(rc))

    # write subscribe topic on on_connect
    # if we lose connect or re-connect
    # will re-subscribe
    client.subscribe("direction")

def on_message(client, userdata, msg):
    print(msg.topic+ " "+ msg.payload.decode('utf-8'))

    if msg.payload.decode('utf-8') == 'forward' :
        MoveForward()

    elif msg.payload.decode('utf-8') == 'left' :
        MoveLeft()

    elif msg.payload.decode('utf-8') == 'right' :
        MoveRight()

    elif msg.payload.decode('utf-8') == 'back' :
        MoveBack()

    elif msg.payload.decode('utf-8') == 'stop' :
        Stop()

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

