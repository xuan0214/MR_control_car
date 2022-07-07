import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
# front left
IN1 = 17	# 11
IN2 = 18	# 12
# front right
IN3 = 27	# 13
IN4 = 22	# 15
# back right
IN5 = 5		# 29
IN6 = 6		# 31
#back left
IN7 = 12	# 32
IN8 = 13	# 33

def init() :
	GPIO.setup(IN1,GPIO.OUT)
	GPIO.setup(IN2,GPIO.OUT)
	GPIO.setup(IN3,GPIO.OUT)
	GPIO.setup(IN4,GPIO.OUT)
	GPIO.setup(IN5,GPIO.OUT)
	GPIO.setup(IN6,GPIO.OUT)
	GPIO.setup(IN7,GPIO.OUT)
	GPIO.setup(IN8,GPIO.OUT)

# front
def front(sleep_time) :
	GPIO.output(IN1,GPIO.HIGH)
	GPIO.output(IN2,GPIO.LOW)
	GPIO.output(IN3,GPIO.HIGH)
	GPIO.output(IN4,GPIO.LOW)
	GPIO.output(IN5,GPIO.HIGH)
	GPIO.output(IN6,GPIO.LOW)
	GPIO.output(IN7,GPIO.HIGH)
	GPIO.output(IN8,GPIO.LOW)
	
	time.sleep(sleep_time)
	GPIO.cleanup()

# back
def back(sleep_time) :
	GPIO.output(IN1,GPIO.LOW)
	GPIO.output(IN2,GPIO.HIGH)
	GPIO.output(IN3,GPIO.LOW)
	GPIO.output(IN4,GPIO.HIGH)
	GPIO.output(IN5,GPIO.LOW)
	GPIO.output(IN6,GPIO.HIGH)
	GPIO.output(IN7,GPIO.LOW)
	GPIO.output(IN8,GPIO.HIGH)
	
	time.sleep(sleep_time)
	GPIO.cleanup()

# left
def left(sleep_time) :
	GPIO.output(IN1,False)
	GPIO.output(IN2,False)
	GPIO.output(IN3,GPIO.HIGH)
	GPIO.output(IN4,GPIO.LOW)
	GPIO.output(IN5,False)
	GPIO.output(IN6,False)
	GPIO.output(IN7,GPIO.HIGH)
	GPIO.output(IN8,GPIO.LOW)
	
	time.sleep(sleep_time)
	GPIO.cleanup()

# right
def right(sleep_time) :
	GPIO.output(IN1,GPIO.HIGH)
	GPIO.output(IN2,GPIO.LOW)
	GPIO.output(IN3,False)
	GPIO.output(IN4,False)
	GPIO.output(IN5,GPIO.HIGH)
	GPIO.output(IN6,GPIO.LOW)
	GPIO.output(IN7,False)
	GPIO.output(IN8,False)
	
	time.sleep(sleep_time)
	GPIO.cleanup()

init()
while True:
	ch = input("front: w, back: s, left: a, right: d")
	if ch == 'w':
		front(10)
	elif ch == 'a':
		left(10)
	elif ch == 's':
		back(10)
	elif ch == 'd':
		right(10)