from machine import PWM,Pin
from time import sleep_ms

#Khai bao chan dieu khien L298N
ENA = 25
IN1 = 18
IN2 = 19
IN3 = 21
IN4 = 22
ENB = 26

#Khai bao 5 cam bien do line(ngoai trai->giua->ngoai phai)
sensor_pins = [32,33,34,35,36]
sensors = [ Pin(pin,Pin.IN) for pin in sensor_pins]

#Cam bien cho muc LOW khi phat hien vach den
line_level = 0

#Tham so dieu khien
base_speed = 70
max_speed = 180
Kp = 30
Kd = 15

#Trong so tu trai->phai
weights = [-2,-1,0,1,2]

#Sai so o vong lap truoc va huong lech gan nhat
lastError = 0
lastDirection = 1

#Khoi tao chan va PWM(PWM MicroPython co duty_u16:0->65535)
in1 = Pin(IN1,Pin.OUT)
in2 = Pin(IN2,Pin.OUT)
in3 = Pin(IN3,Pin.OUT)
in4 = Pin(IN4,Pin.OUT)
PWM_left  = PWM(Pin(ENA),freq=1000,duty_u16=0)
PWM_right = PWM(Pin(ENB),freq=1000,duty_u16=0)

#Chuyen toc do 0->255 sang duty PWM MicroPython
def set_pwm(pwm,speed):
    speed = max(0,min(255,int(speed)))
    duty  = speed * 65535 // 255
    pwm.duty_u16(duty)
    
#Dieu khien 1 ben dong co(speed>0:quay 1 chieu;speed<0:quay chieu con lai;speed=0:phanh dien)
def driveMotor(inA,inB,pwm,speed):
    speed=max(-max_speed,min(max_speed,int(speed)))
    if speed > 0:
        inA.value(1)
        inB.value(0)
        set_pwm(pwm,speed)
    elif speed < 0:
        inA.value(0)
        inB.value(1)
        set_pwm(pwm,-speed)
    else:
        inA.value(1)
        inB.value(1)
        set_pwm(pwm,255)
        
#Dieu khien hai ben dong co 
def setMotors(leftSpeed,rightSpeed):
    driveMotor(in1,in2,PWM_left,leftSpeed)
    driveMotor(in3,in4,PWM_right,rightSpeed)

#Tim lai vach khi mat line 
def recoverline():
    speed = 140
    if lastDirection < 0:
        setMotors(-speed,speed)
    else:
        setMotors(speed,-speed)

#Chuong trinh chinh
while True:
    #1.Doc 5 cam bien
    activeCount = 0
    weightedSum = 0
    for i in range(5):
        raw = sensors[i].value()
        if raw == line_level:
            active = 1
        else:
            active = 0
        activeCount += active
        weightedSum += active * weights[i]
    
    #2.Kiem tra mat vach
    if activeCount == 0:
        recoverline()
        sleep_ms(5)
        continue

    #3.Tinh sai so vi tri 
    error = weightedSum / activeCount
    
    #Luu huong lech gan nhat 
    if error < 0:
        lastDirection = -1
    elif error > 0:
        lastDirection = 1
    
    #4.Tinh thanh phan PD
    derivative = error - lastError
    correction = Kp * error + Kd * derivative
    lastError  = error

    #5.Dieu chinh toc do co ban 
    base = base_speed
    if abs(error) >= 0.5:
        base = 65 
    if abs(error) >= 1:
        base = 55 
    if abs(error) >= 1.5:
        base = 45 
    base = max(0,min(max_speed,base))

    #6.Tinh toc do tung ben 
    leftSpeed  = base + int(correction)
    rightSpeed = base - int(correction)
    leftSpeed  = max(-max_speed,min(max_speed,leftSpeed)) 
    rightSpeed = max(-max_speed,min(max_speed,rightSpeed))

    #7.Cap lenh cho dong co 
    setMotors(leftSpeed,rightSpeed)
    sleep_ms(5)