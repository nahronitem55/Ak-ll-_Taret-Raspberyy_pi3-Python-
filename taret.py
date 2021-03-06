from collections import deque
import cv2 # g�r�nt� i�lememizi sa�layacak opencv k�t�phanesi
import argparse 
import imutils
import numpy as np # Bu ve �sttekiler de gene g�r�nt� i�lemede laz�m olan k�t�phaneler
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
# Motor s�r�c�m�z�n indirdi�imiz dosyadan k�t�phanelere eri�iyoruz.
import time
import atexit
import RPi.GPIO as GPIO # Lazeri �al��t�rmak i�in gereken k�t�phane.

# K�t�phaneleri �a��rd�k.

mh = Adafruit_MotorHAT() # Kolayl�k olsun diye motor �al��t�rma fonksiyonumuzu de�i�kene atad�k. 
GPIO.setmode(GPIO.BCM) 
GPIO.setup(18, GPIO.OUT) # ��k�� alaca��m�z raspberry pi �zerindeki 18. pin e ba�lad�k ve belirttik.
vur = False
taraf = True
adim = 0

def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

# Motorumuzu �al��t�racak fonksiyonlar

myStepper = mh.getStepper(200, 2) # Motorun ka� ad�ml� oldu�u ve ka�ar ka�ar d�nece�ini belirttik.
myStepper.setSpeed(5) # Motorun d�nme h�z�n� belirledik.

ileriyon = Adafruit_MotorHAT.FORWARD 
geriyon = Adafruit_MotorHAT.BACKWARD
yon = ileriyon
# ileri ve geri ye d�nme komutlar�n� atad�k.

cap = cv2.VideoCapture(0)  # Kamera �al��t�r�ld�.
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
   help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
   help="max buffer size")
args = vars(ap.parse_args())

while(1): # �al��maya ba�lad�.
    GPIO.output(18, False) # E�er lazer'imiz yan�yor ise kapatt�k.
    # Take each frame
    #_, frame = cap.read()
    (grabbed, frame) = cap.read() # Kameradan gelen g�r�nt�y� okuyoruz.

    if args.get("video") and not grabbed: # Kamera tak�l� de�ilse kapat�yor durduruyor.
        break

    frame = imutils.resize(frame, width=400) # Alaca��m�z g�r�nt�n�n boyutunu belirliyoruz.

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    # define range of blue color in HSV
    lower_blue = (0, 200, 0)
    upper_blue = (19, 255, 255)
    pts = deque(maxlen=args["buffer"])

    # Alg�lamas�n� istedi�iniz rengi belittik.

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    # E�er bizim istedi�imiz rengi buldu iste maskeliyoruz.
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
       # Buldu�umuz rengin orta noktas�n� belirliyoruz.
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            # Buldu�umuz rengin b�y�kl��� yeterli ise daire i�ine al�yoruz.
       if vur == False: # Bulunan rengin oldu�u yerde lazer i yak�yoruz 1 saniye bekliyoruz.
                #myStepper.step(1, geriyon, mh.MICROSTEP)
                GPIO.output(18, True)
  time.sleep(1)
                vur = False
                continue # While d�ng�s�n�n ba��na d�n�yoruz.


 else: # Herhangi bir hedef bulamaz ise motoru d�nd�r�yoruz.
     adim += 2
     print adim
     if adim < 200:
                myStepper.step(2, yon, mh.SINGLE)
     else: # Kablo dolanmas�n diye 360 derece sa� d�nd�kten sonra sola �eviriyoruz ve tekrar ediyor.
  if taraf == True:
      yon = geriyon
      taraf = False
      adim = 0
  else:
      yon = ileriyon
      taraf = True
      adim = 0
     
    # update the points queue
    pts.appendleft(center)
  
    for i in xrange(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:    
     continue

        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    cv2.imshow('frame',frame) # yakalad���m�z g�r�nt�y� g�steriyor.
    #cv2.imshow('mask',mask)
    #cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF # ESC tu�una bast���m�zda durmas�n� sa�l�yor.
    if k == 27:
        break

cv2.destroyAllWindows()