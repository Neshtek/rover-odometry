import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Ultrasonic:
    def __init__(self, TRIGGER, ECHO):
        print('Created')
        self.TRIGGER = TRIGGER
        self.ECHO = ECHO
        self.drive_ok = False
        self.area_completed = False
        GPIO.setup(self.TRIGGER, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

    def get_distance(self):
        # set Trigger to HIGH
        GPIO.output(self.TRIGGER, True)
    
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.TRIGGER, False)
    
        start_time = time.time()
        stop_time = time.time()
    
        # save start_time
        while GPIO.input(self.ECHO) == 0:
            start_time = time.time()
    
        # save time of arrival
        while GPIO.input(self.ECHO) == 1:
            stop_time = time.time()

        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34300) / 2
    
        if distance > 30:
            return 30
        else:
            return distance
    
    def check_drive_ok(self):
        edge_dist = self.get_distance()    
        if edge_dist <= 10:
            print ("Measured Distance = %.1f cm" % edge_dist)
            return True
        else:
            print ("Measured Distance = %.1f cm" % edge_dist)
            return False
            time.sleep(1) #time too much
            d_edge = self.get_distance()
            if d_edge > 10:
                return False
                print ("Measured Distance 2 = %.1f cm" % d_edge)
            else:
                pass
        print ("Measured Distance = %.1f cm" % edge_dist)
        time.sleep(0.1)

if __name__ == '__main__':
    pass