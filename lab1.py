from rbotLib.py import *
def initEncoders():
    lcount = 0
    rcount = 0
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
    GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)
    print("Testing the Encoders, Spin to test")
