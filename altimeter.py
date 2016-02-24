import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

TRIG = 23 
ECHO = 24

print "Distance Measurement In Progress"

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

def sendAlt():
    try:
        GPIO.output(TRIG, False)
        time.sleep(0.02)

        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        
        timer = time.time()
        while GPIO.input(ECHO)==0:
            elapsed = timer - time.time()
            pulse_start = time.time()
            if elapsed > 0.5:
                break

        timer = time.time()
        while GPIO.input(ECHO)==1:
            elapsed = timer - time.time()
            pulse_end = time.time()
            if elapsed > 0.5:
                break

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150

        distance = round(distance, 2)

    #    print "Distance:",distance,"cm"

        return distance

        GPIO.cleanup()

    except Exception,error:
        print "Error in sendAlt thread: "+str(error)

sendAlt()
