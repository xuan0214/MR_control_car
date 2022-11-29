import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time
import smbus
import sys

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

#get i2c
bus = smbus.SMBus(1)
bus.write_byte_data(0x53, 0x2C, 0x0B)
value = bus.read_byte_data(0x53, 0x31)
value &= ~0x0F
value |= 0x0B
value |= 0x08
bus.write_byte_data(0x53, 0x31, value)
bus.write_byte_data(0x53, 0x2D, 0x08)
#bus.write_byte_data(0x53, 0x2C, 0x0A)
#bus.write_byte_data(0x53, 0x2D, 0x08)
#bus.write_byte_data(0x53, 0x31, 0x08)
#time.sleep(0.5)

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
PWM_Freq = 50

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

    pwmB_IN2.ChangeDutyCycle(20)
    pwmB_IN4.ChangeDutyCycle(20)
    pwmF_IN2.ChangeDutyCycle(20)
    pwmF_IN4.ChangeDutyCycle(20)


# back
def MoveBack():
    print('MoveBack ')

    pwmB_IN2.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(0)

    pwmB_IN1.ChangeDutyCycle(20)
    pwmB_IN3.ChangeDutyCycle(20)
    pwmF_IN1.ChangeDutyCycle(20)
    pwmF_IN3.ChangeDutyCycle(20)

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
    pwmB_IN4.ChangeDutyCycle(20)

    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(0)
    pwmF_IN3.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(20)



# right
def MoveRight():
    print('MoveRight()')
    pwmB_IN1.ChangeDutyCycle(0)
    pwmB_IN2.ChangeDutyCycle(20)
    pwmB_IN3.ChangeDutyCycle(0)
    pwmB_IN4.ChangeDutyCycle(0)

    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(20)
    pwmF_IN3.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(0)

def getAxes(dir):
    bytes = bus.read_i2c_block_data(0x53, 0x32, 6)
    x = bytes[0] | (bytes[1] << 8)
    if(x & (1 << 16-1)):
        x = x - (1<<16)
    y = bytes[2] | (bytes[3] << 8)
    if(y & (1 << 16-1)):
        y = y - (1<<16)
    z = bytes[4] | (bytes[5] << 8)
    if(z & (1 << 16-1)):
        z = z - (1<<16)
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

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+ str(rc))

    # write subscribe topic on on_connect
    # if we lose connect or re-connect
    # will re-subscribe
    client.subscribe("direction")

def on_message(client, userdata, msg):
    print(msg.topic+ " "+ msg.payload.decode('utf-8'))

    start_time = time.time()

    if msg.payload.decode('utf-8') == 'forward' :
        MoveForward()
        end_time = time.time()
        x_speed = getAxes(0) * 9.80665
        y_speed = getAxes(1)
        z_speed = getAxes(2)
        #data0 = bus.read_byte_data(0x53, 0x32)
        #data1 = bus.read_byte_data(0x53, 0x33)
        #xAccl = ((data1 & 0x03) * 256) + data0
        #if xAccl > 511:
        #    xAccl -= 1024
        #end_time = time.time()
        #print('xaccl = ' + str(xAccl))
        #speed = abs(xAccl) / (end_time - start_time)
        #speed = str(speed)
        #print('speed = ' + str(speed))
        print('time0 = ' + str(start_time))
        print('time1 = ' + str(end_time))
        speed = x_speed*(end_time - start_time)
        print('speed = ' + str(speed))
        print('xspeed = ' + str(x_speed))
        print('yspeed = ' + str(y_speed))
        print('zspeed = ' + str(z_speed))

    elif msg.payload.decode('utf-8') == 'left' :
        MoveLeft()
        data0 = bus.read_byte_data(0x53, 0x32)
        data1 = bus.read_byte_data(0x53, 0x33)
        xAccl = ((data1 & 0x03) * 256) + data0
        if xAccl > 511:
            xAccl -= 1024
        data0 = bus.read_byte_data(0x53, 0x34)
        data1 = bus.read_byte_data(0x53, 0x35)
        yAccl = ((data1 & 0x03) * 256) + data0
        if yAccl > 511:
            yAccl -= 1024
        end_time = time.time()
        #speed = sqrt(abs(xAccl)*abs(xAccl) + abs(yAccl)*abs(yAccl)) / (end_time - start_time)
        #speed = str(speed)
        #print('speed = ' + speed)

    elif msg.payload.decode('utf-8') == 'right' :
        MoveRight()
        data0 = bus.read_byte_data(0x53, 0x32)
        data1 = bus.read_byte_data(0x53, 0x33)
        xAccl = ((data1 & 0x03) * 256) + data0
        if xAccl > 511:
            xAccl -= 1024
        data0 = bus.read_byte_data(0x53, 0x34)
        data1 = bus.read_byte_data(0x53, 0x35)
        yAccl = ((data1 & 0x03) * 256) + data0
        if yAccl > 511:
            yAccl -= 1024
        end_time = time.time()
        #speed = sqrt(abs(xAccl)*abs(xAccl) + abs(yAccl)*abs(yAccl)) / (end_time - start_time)
        #speed = str(speed)
        #print('speed = ' + speed)

    elif msg.payload.decode('utf-8') == 'back' :
        MoveBack()
        data0 = bus.read_byte_data(0x53, 0x32)
        data1 = bus.read_byte_data(0x53, 0x33)
        xAccl = ((data1 & 0x03) * 256) + data0
        if xAccl > 511:
            xAccl -= 1024
        end_time = time.time()
        #speed = abs(xAccl) / (end_time - start_time)
        #speed = str(speed)
        #print('speed = ' + speed)

    elif msg.payload.decode('utf-8') == 'stop' :
        Stop()
        speed = 0
        speed = str(speed)
        #print('speed = ' + speed)


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

