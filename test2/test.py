import RPi.GPIO as GPIO
import time


print('start program')
print('init')

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# front left
F_IN1 = 17
F_IN2 = 18

# front right
F_IN3 = 27
F_IN4 = 22

# back left
B_IN1 = 12
B_IN2 = 13

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
    pwmB_IN2.ChangeDutyCycle(80)
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
    pwmB_IN4.ChangeDutyCycle(40)

    pwmF_IN1.ChangeDutyCycle(0)
    pwmF_IN2.ChangeDutyCycle(80)
    pwmF_IN3.ChangeDutyCycle(0)
    pwmF_IN4.ChangeDutyCycle(0)


ch = input("front: w, back: s, left: a, right: d\n")

while True :
    if ch == 'w' :
        MoveForward()

    elif ch == 'a' :
        MoveLeft()

    elif ch == 'd':
        MoveRight()

    elif ch == 's' :
        MoveBack()

    elif ch == ' ' :
        Stop()

    ch = input("front: w, back: s, left: a, right: d\n")



# 釋放清理
pwmB_N1.stop()
pwmB_N2.stop()
pwmB_N3.stop()
pwmB_N4.stop()

pwmF_N1.stop()
pwmF_N2.stop()
pwmF_N3.stop()
pwmF_N4.stop()
GPIO.cleanup()
