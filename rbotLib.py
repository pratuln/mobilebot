import cv2 as cv
import time
from ThreadedWebcam import ThreadedWebcam
from UnthreadedWebcam import UnthreadedWebcam
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/ 
import signal # README: https://raspberry-projects.com/pi/programming-in-c/signal-handling/signal-function
import math
import Adafruit_PCA9685
import adafruit_bno055
import board
from threading import Thread
import VL53L0X
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')


FPS_SMOOTHING = 0.9
#start of blob
# Window names
WINDOW1 = "Adjustable Mask - Press Esc to quit"
WINDOW2 = "Detected Blobs - Press Esc to quit"

# Default HSV ranges
# Note: the range for hue is 0-180, not 0-255
minH =   0; minS = 127; minV =   0;
maxH = 180; maxS = 255; maxV = 255;


# These functions are called when the user moves a trackbar
def onMinHTrackbar(val):
    # Calculate a valid minimum red value and re-set the trackbar.
    global minH
    global maxH
    minH = min(val, maxH - 1)
    cv.setTrackbarPos("Min Hue", WINDOW1, minH)

def onMinSTrackbar(val):
    global minS
    global maxS
    minS = min(val, maxS - 1)
    cv.setTrackbarPos("Min Sat", WINDOW1, minS)

def onMinVTrackbar(val):
    global minV
    global maxV
    minV = min(val, maxV - 1)
    cv.setTrackbarPos("Min Val", WINDOW1, minV)

def onMaxHTrackbar(val):
    global minH
    global maxH
    maxH = max(val, minH + 1)
    cv.setTrackbarPos("Max Hue", WINDOW1, maxH)

def onMaxSTrackbar(val):
    global minS
    global maxS
    maxS = max(val, minS + 1)
    cv.setTrackbarPos("Max Sat", WINDOW1, maxS)

def onMaxVTrackbar(val):
    global minV
    global maxV
    maxV = max(val, minV + 1)
    cv.setTrackbarPos("Max Val", WINDOW1, maxV)


# Initialize the threaded camera
# You can run the unthreaded camera instead by changing the line below.
# Look for any differences in frame rate and latency.
camera = ThreadedWebcam() # UnthreadedWebcam()
camera.start()

# Initialize the SimpleBlobDetector
params = cv.SimpleBlobDetector_Params()
detector = cv.SimpleBlobDetector_create(params)

# Attempt to open a SimpleBlobDetector parameters file if it exists,
# Otherwise, one will be generated.
# These values WILL need to be adjusted for accurate and fast blob detection.
fs = cv.FileStorage("params.yaml", cv.FILE_STORAGE_READ); #yaml, xml, or json
if fs.isOpened():
    detector.read(fs.root())
else:
    print("WARNING: params file not found! Creating default file.")
    
    fs2 = cv.FileStorage("params.yaml", cv.FILE_STORAGE_WRITE)
    detector.write(fs2)
    fs2.release()
    
fs.release()

# Create windows
cv.namedWindow(WINDOW1)
cv.namedWindow(WINDOW2)

# Create trackbars
cv.createTrackbar("Min Hue", WINDOW1, minH, 180, onMinHTrackbar)
cv.createTrackbar("Max Hue", WINDOW1, maxH, 180, onMaxHTrackbar)
cv.createTrackbar("Min Sat", WINDOW1, minS, 255, onMinSTrackbar)
cv.createTrackbar("Max Sat", WINDOW1, maxS, 255, onMaxSTrackbar)
cv.createTrackbar("Min Val", WINDOW1, minV, 255, onMinVTrackbar)
cv.createTrackbar("Max Val", WINDOW1, maxV, 255, onMaxVTrackbar)

fps, prev = 0.0, 0.0
while True:
    # Calculate FPS
    now = time.time()
    fps = (fps*FPS_SMOOTHING + (1/(now - prev))*(1.0 - FPS_SMOOTHING))
    prev = now

    # Get a frame
    frame = camera.read()
    
    # Blob detection works better in the HSV color space 
    # (than the RGB color space) so the frame is converted to HSV.
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # Create a mask using the given HSV range
    mask = cv.inRange(frame_hsv, (minH, minS, minV), (maxH, maxS, maxV))
    
    # Run the SimpleBlobDetector on the mask.
    # The results are stored in a vector of 'KeyPoint' objects,
    # which describe the location and size of the blobs.
    keypoints = detector.detect(mask)
    
    # For each detected blob, draw a circle on the frame
    frame_with_keypoints = cv.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # Write text onto the frame
    cv.putText(frame_with_keypoints, "FPS: {:.1f}".format(fps), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
    cv.putText(frame_with_keypoints, "{} blobs".format(len(keypoints)), (5, 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
    
    # Display the frame
    cv.imshow(WINDOW1, mask)
    cv.imshow(WINDOW2, frame_with_keypoints)
    
    # Check for user input
    c = cv.waitKey(1)
    if c == 27 or c == ord('q') or c == ord('Q'): # Esc or Q
        camera.stop()
        break

camera.stop()

#end of blob

#start of camera
# Initialize camera with a specified resolution.
# It may take some experimenting to find other valid resolutions,
# as the camera may end up displaying an incorrect image.
# Alternatively, frames can be resized afterwards using the resize() function.
capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not capture.isOpened():
    print("Failed to open camera!")
    exit()
print("Testing Camera, press 'q' to quit")
fps, prev = 0.0, 0.0
while True:
    # Calculate FPS
    now = time.time()
    fps = (fps*FPS_SMOOTHING + (1/(now - prev))*(1.0 - FPS_SMOOTHING))
    prev = now

    # Get a frame
    ret, frame = capture.read()
    if not ret:
        break

    # Write text onto the frame
    cv2.putText(frame, "FPS: {:.1f}".format(fps), (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
    
    # Display the frame
    cv2.imshow("Preview - Press Esc to exit", frame)
    
    # Check for user input
    c = cv2.waitKey(1)
    if c == 27 or c == ord('q') or c == ord('Q'): # Esc or Q
        break

#end of camera

#start of encoders
# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

# This function is called when the left encoder detects a rising edge signal.
def onLeftEncode(pin):
    print("Left encoder ticked!")

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    print("Right encoder ticked!")

# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")
    GPIO.cleanup()
    exit()

# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)
    
# Set the pin numbering scheme to the numbering shown on the robot itself.
GPIO.setmode(GPIO.BCM)

# Set encoder pins as input
# Also enable pull-up resistors on the encoder pins
# This ensures a clean 0V and 3.3V is always outputted from the encoders.
GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Attach a rising edge interrupt to the encoder pins
GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

# Prevent the program from exiting by adding a looping delay.
print("Testing the Encoders, Spin to test")
i = 0
while i < 10:
    time.sleep(1)
    i = i+1

#end of encoders

#start of servos
# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 0
RSERVO = 1

# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")
    
    # Stop the servos
    pwm.set_pwm(LSERVO, 0, 0);
    pwm.set_pwm(RSERVO, 0, 0);
    
    exit()

# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)
    
# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

# Write an initial value of 1.5, which keeps the servos stopped.
# Due to how servos work, and the design of the Adafruit library, 
# the value must be divided by 20 and multiplied by 4096.
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));

print("Testing the Servos, Each wheel should spin in both direction")
i = 0
while i < 1:
    # Write a maximum value of 1.7 for each servo.
    # Since the servos are oriented in opposite directions,
    # the robot will end up spinning in one direction.
    # Values between 1.3 and 1.7 should be used.
    pwm.set_pwm(LSERVO, 0, math.floor(1.6 / 20 * 4096));
    pwm.set_pwm(RSERVO, 0, math.floor(1.6 / 20 * 4096));
    time.sleep(4)
    
    # Write a minimum value of 1.4 for each servo.
    # The robot will end up spinning in the other direction.
    pwm.set_pwm(LSERVO, 0, math.floor(1.4 / 20 * 4096));
    pwm.set_pwm(RSERVO, 0, math.floor(1.4 / 20 * 4096));
    time.sleep(4)
    i= i+1

# Stop the servos
pwm.set_pwm(LSERVO, 0, 0);
pwm.set_pwm(RSERVO, 0, 0);

# exit() commented out





#end of servos

#start of imu
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

# If you are going to use UART uncomment these lines
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)

last_val = 0xFFFF


def temperature():
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result

flag = 0

while flag <= 50:
    flag = flag + 1
    #print("Temperature: {} degrees C".format(sensor.temperature))
    """
    print(
        "Temperature: {} degrees C".format(temperature())
    )  # Uncomment if using a Raspberry Pi
    """
    #print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    #print("Magnetometer (microteslas): {}".format(sensor.magnetic))
    #print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    print("Euler angle: {}".format(sensor.euler))
    print("Quaternion: {}".format(sensor.quaternion))
    #print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    #print("Gravity (m/s^2): {}".format(sensor.gravity))
    print()

    time.sleep(1)

    #end of imu

    #start of threaded
    class ThreadedWebcam:
	def __init__(self, src=0, name="ThreadedWebcam"):
		# Initialize camera with a specified resolution.
		# It may take some experimenting to find other valid resolutions,
		# as the camera may end up displaying an incorrect image.
		# Alternatively, frames can be resized afterwards using the resize() function.
		self.stream = cv.VideoCapture(src)
		self.stream.set(cv.CAP_PROP_FRAME_WIDTH, 640)
		self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
		#self.stream.set(cv.CAP_PROP_BUFFERSIZE, 1)
		
		if not self.stream.isOpened():
			print("Failed to open camera!")
			exit()
		else:
			print("Threaded webcam started.")
		
		(self.grabbed, self.frame) = self.stream.read()
		
		self.name = name
		self.stopped = False
	
	# Starts the camera thread.
	def start(self):
		t = Thread(target=self._update, name=self.name, args=())
		t.daemon = True
		t.start()
		return self
	
	# Returns the latest camera frame.
	def read(self):
		return self.frame
	
	# Stops the camera thread.
	def stop(self):
		self.stopped = True
	
	# Private function that constantly reads the camera stream.
	# Do not call this function externally.
	def _update(self):
		while not self.stopped:
			(self.grabbed, self.frame) = self.stream.read()


#end of threaded

#start of timing
t1 = time.monotonic()
print("Testing the System Timer")
print("Current time:")
print("{0}s".format(t1))

# Wait 500 ms.
time.sleep(0.5)

# Get current time again.
t2 = time.monotonic()
msElapsed = (t2 - t1) * 10 ** 3 # Convert seconds to ms

print("500 ms later:")
print("{0}s, {1:.4}ms elapsed".format(t2, msElapsed))

# Wait 100 us. The Pi will most likely end up waiting significantly longer.
time.sleep(0.0001)

# Get current time again.
t3 = time.monotonic()
usElapsed = (t3 - t2) * 10 ** 6 # Convert seconds to us

print("100 us later:")
print("{0}s, {1:.4}us elapsed".format(t3, usElapsed))

#end of timing

#start of tof
# Pins that the sensors are connected to
LSHDN = 27
FSHDN = 22
RSHDN = 23

DEFAULTADDR = 0x29 # All sensors use this address by default, don't change this
LADDR = 0x2a 
RADDR = 0x2b 

# Set the pin numbering scheme to the numbering shown on the robot itself.
GPIO.setmode(GPIO.BCM)

# Setup pins
GPIO.setup(LSHDN, GPIO.OUT)
GPIO.setup(FSHDN, GPIO.OUT)
GPIO.setup(RSHDN, GPIO.OUT)

# Shutdown all sensors
GPIO.output(LSHDN, GPIO.LOW)
GPIO.output(FSHDN, GPIO.LOW)
GPIO.output(RSHDN, GPIO.LOW)

time.sleep(0.01)

# Initialize all sensors
lSensor = VL53L0X.VL53L0X(address=LADDR)
fSensor = VL53L0X.VL53L0X(address=DEFAULTADDR)
rSensor = VL53L0X.VL53L0X(address=RADDR)

# Connect the left sensor and start measurement
GPIO.output(LSHDN, GPIO.HIGH)
time.sleep(0.01)
lSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the right sensor and start measurement
GPIO.output(RSHDN, GPIO.HIGH)
time.sleep(0.01)
rSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the front sensor and start measurement
GPIO.output(FSHDN, GPIO.HIGH)
time.sleep(0.01)
fSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

print("Testing Distance Sensors, should print 100 readings")
for count in range(1, 100):
    # Get a measurement from each sensor
    lDistance = lSensor.get_distance()
    fDistance = fSensor.get_distance()
    rDistance = rSensor.get_distance()
    
    # Print each measurement
    print("Left: {}\tFront: {}\tRight: {}".format(lDistance, fDistance, rDistance))

# Stop measurement for all sensors
lSensor.stop_ranging()
fSensor.stop_ranging()
rSensor.stop_ranging()

#end of tof

#start of unthreaded
class UnthreadedWebcam:
    def __init__(self, src=0, name="ThreadedWebcam"):
        self.stream = cv.VideoCapture(src)
        self.stream.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        #self.stream.set(cv.CAP_PROP_BUFFERSIZE, 1)
        
        if not self.stream.isOpened():
            print("Failed to open camera!")
            exit()
        else:
            print("Unthreaded webcam started.")
            
    def start(self):
        # The pass statement is a "do nothing" statement.
        pass
        
    def read(self):
        (self.grabbed, self.frame) = self.stream.read()
        return self.frame
    
    def stop(self):
        pass
#end of unthreaded