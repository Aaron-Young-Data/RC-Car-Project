import RPi.GPIO as GPIO
from ADCDevice import *
from gpiozero import OutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

#Setup to connect to 'car' Pi
factory = PiGPIOFactory(host='192.168.1.186')

#Needed Pins on 'Car' Pi
FWDButtonPin = 32
BWDButtonPin = 36
FWDRelayPin = 15
BWDRelayPin = 18
ServoPin = 14
Z_Pin = 12

adc = ADCDevice()


#Setup the needed GPIO pins
GPIO.setmode(GPIO.BOARD)

GPIO.setup(BWDButtonPin,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(FWDButtonPin,GPIO.IN, pull_up_down=GPIO.PUD_UP)
servo = Servo(ServoPin, pin_factory=factory)


def setup():
    #Will setup and connect to the IC2 chip being used for input
    global adc
    if(adc.detectI2C(0x48)): # Detect the pcf8591.
        adc = PCF8591()
    elif(adc.detectI2C(0x4b)): # Detect the ads7830
        adc = ADS7830()
    else:
        print("No correct I2C address found, \n"
        "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
        "Program Exit. \n");
        exit(-1)
    GPIO.setup(Z_Pin,GPIO.IN,GPIO.PUD_UP)

def loop():
    while True:
        if GPIO.input(FWDButtonPin)==GPIO.LOW:
           try:
                #Will turn on FWD relay pin when the button is pressed on the controler
                print('FWD')
                FWDrelay = OutputDevice(FWDRelayPin, pin_factory=factory)
                FWDrelay.on()
           except:
                pass
        elif GPIO.input(FWDButtonPin)==GPIO.HIGH:
            try:
                #Need to close the PIN to turn it of because of the type of relay.
                FWDrelay.close()
            except:
                pass
        if GPIO.input(BWDButtonPin)==GPIO.LOW:
            try:
                #Will turn on BWD relay pin when the button is pressed on the controler
                print('BWD')
                BWDrelay = OutputDevice(BWDRelayPin, pin_factory=factory)
                BWDrelay.on()
            except:
                pass
        elif GPIO.input(BWDButtonPin)==GPIO.HIGH:
            try:
                #Need to close the PIN to turn it of because of the type of relay.
                BWDrelay.close()
            except:
                pass
        #Reads the IC2 input and will convert the output to a number between -1 and 1
        val_Z = GPIO.input(Z_Pin)
        val_Y = adc.analogRead(0)
        val_X = adc.analogRead(1)
        val_X = round((((val_X/255)*2)-1), 2)
        val_Y = round((((val_Y/255)*2)-1),2)
        print(val_X, val_Y)
        #sets the servo position to the X value calculated above.
        servo.value = (val_X)
        #print ('value_X: %f ,\tvlue_Y: %f ,\tvalue_Z: %d'%(val_X,val_Y,val_Z))
        sleep(0.01)

def destroy():
    adc.close()
    GPIO.cleanup()
    
if __name__ == '__main__':
    print ('Program is starting ... ') # Program entrance
    setup()
    try:
        loop()
    except KeyboardInterrupt: # Press ctrl-c to end the program.
        destroy()

