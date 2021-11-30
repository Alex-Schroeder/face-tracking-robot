#If it sees a face, constantly move to make the face more in the center. If it doesn’t see a face, don’t do anything.



#import packages
import RPi.GPIO as GPIO
import time
import threading
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

#pinout setup
GPIO.setmode(GPIO.BOARD)
control_pins_x = [7, 11, 13, 15]
control_pins_y = [8, 10, 12, 16]
for pin in control_pins_x + control_pins_y:
  GPIO.setup(pin, GPIO.OUT)
  GPIO.output(pin, 0)

#sequence for motors to turn
halfstep_seq_rev = [
  [1,0,0,0],
  [1,1,0,0],
  [0,1,0,0],
  [0,1,1,0],
  [0,0,1,0],
  [0,0,1,1],
  [0,0,0,1],
  [1,0,0,1]
]

halfstep_seq = [
  [1,0,0,1],
  [0,0,0,1],
  [0,0,1,1],
  [0,0,1,0],
  [0,1,1,0],
  [0,1,0,0],
  [1,1,0,0],
  [1,0,0,0]
]

#fullstep_seq = [
#  [1,0,0,0],
#  [0,1,0,0],
#  [0,0,1,0],
#  [0,0,0,1],
#]


#function to turn motor 1 by one revolution
def motor_1():
    while True:
        print(len(faces))
        print(x_cor)

        if len(faces) > 0:
            if x_cor < 160:
                for halfstep in range(8):
                    for pin in range(4):
                        GPIO.output(control_pins_x[pin], halfstep_seq_rev[halfstep][pin])
                    time.sleep(0.002)
            elif x_cor > 160:
                for halfstep in range(8):
                    for pin in range(4):
                        GPIO.output(control_pins_x[pin], halfstep_seq[halfstep][pin])
                    time.sleep(0.002)

#function to turn motor 2 by one revolution
def motor_2():
    while True:
        if len(faces) > 0:
            if y_cor < 120:
                for halfstep in range(8):
                    for pin in range(4):
                        GPIO.output(control_pins_y[pin], halfstep_seq_rev[halfstep][pin])
                    time.sleep(0.002)
            elif y_cor > 120:
                for halfstep in range(8):
                    for pin in range(4):
                        GPIO.output(control_pins_y[pin], halfstep_seq[halfstep][pin])
                    time.sleep(0.002)

#creating global varaibles for faces, x coordinate, and y coordinate
faces = []
x_cor = 0
y_cor = 0

#function for camera to run
def camera():
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (320, 240)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(320, 240))

    # allow the camera to warmup
    time.sleep(0.1)

    #define what type of recognition we are doing
    faceCascade = cv2.CascadeClassifier("/home/pi/opencv/data/haarcascades/haarcascade_frontalface_default.xml")

    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        img = frame.array

        #detect the face
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        global faces
        faces = faceCascade.detectMultiScale(imgGray)

        #find the average location of every face
        if len(faces) > 0:
            for (x,y,w,h) in faces:
                x_holder = x+w//2
                y_holder = y+h//2
            global x_cor
            x_cor = x_holder // len(faces)
            global y_cor
            y_cor = y_holder // len(faces)

            #put dot at average cetner of faces
            cv2.rectangle(img,(x_cor,y_cor),(x_cor,y_cor),(255,0,0),20)

        #show face with dot
        cv2.imshow("Frame", img)
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

#threading the functions
if __name__ == "__main__":
        # creating threads
        m1 = threading.Thread(target=motor_1, name='m1')
        m2 = threading.Thread(target=motor_2, name='m2')
        ca = threading.Thread(target=camera, name='ca')

        # starting threads
        m1.start()
        m2.start()
        ca.start()

        # wait until all threads finish
        m1.join()
        m2.join()

        GPIO.cleanup()
        print("complete")