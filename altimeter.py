import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

TRIG = 23 
ECHO = 24

print "Distance Measurement In Progress"

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

def sendAlt():

    GPIO.output(TRIG, False)
    time.sleep(0.05)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    
    timer = time.time()

    while GPIO.input(ECHO)==0 and time.time()-timer < 0.0175:
        continue
    pulse_start = time.time()

    timer = time.time()

    while GPIO.input(ECHO)==1 and time.time()-timer < 0.0175:
        continue
    pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration * 17150

    distance = round(distance, 2)

#    print "Distance:",distance,"cm"

    return distance

    GPIO.cleanup()

sendAlt()
