import cv2
import time
import mediapipe as mp 
import numpy
import math
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
import HandTrackingModule as htm
import pyfirmata

board = pyfirmata.Arduino('COM3')
iter8 = pyfirmata.util.Iterator(board)
iter8.start()

pinThumb = board.get_pin('d:9:s')
pinIndex = board.get_pin('d:5:s')
pinMiddle = board.get_pin('d:6:s')
pinRing = board.get_pin('d:10:s')
pinPinky = board.get_pin('d:11:s')

def main():
    ################################
    wCam, hCam = 640, 480
    ################################

    cap = cv2.VideoCapture(0)
    # cap.open('http://192.168.1.199:8080/video') 
    cap.set(3, wCam)    
    cap.set(4, hCam)
    pTime = 0
    CTime = 0
    area = 0

    detector = htm.handDetector(detectionCon=0.7, maxHands=1)

    while cap.isOpened():
        success, image = cap.read()
        image = detector.findhands(image)
        lmLists = detector.findPosition(image, draw=False)
        fingers = detector.fingersUp()
        if len(lmLists) != 0:
            
            #---------------------------------------Index Finger--------------------------------------------
            x1Index, y1Index = lmLists[8][1], lmLists[8][2]
            x2Index, y2Index = lmLists[5][1], lmLists[5][2]

            ix, iy = (x1Index + x2Index)//2, (y1Index + y2Index)//2

            cv2.circle(image, (x1Index, y1Index), 7, (255,0,255), cv2.FILLED)
            cv2.circle(image, (x2Index, y2Index), 7, (255,0,255), cv2.FILLED)
            cv2.circle(image, (ix, iy), 7, (255,0,255), cv2.FILLED)
            cv2.line(image, (x1Index, y1Index), (x2Index, y2Index), (255, 0, 255), 3)

            lengthIndex = math.hypot(x1Index - x2Index, y1Index - y2Index)

            if lengthIndex < 50:
                cv2.circle(image, (ix, iy), 7, (0, 255, 0), cv2.FILLED)
            
            #---------------------------------------Middle Finger--------------------------------------------
            x1Middle, y1Middle = lmLists[12][1], lmLists[12][2]
            x2Middle, y2Middle = lmLists[9][1], lmLists[9][2]

            mx, my = (x1Middle + x2Middle)//2, (y1Middle + y2Middle)//2

            cv2.circle(image, (x1Middle, y1Middle), 7, (255,0,255), cv2.FILLED)
            cv2.circle(image, (x2Middle, y2Middle), 7, (255,0,255), cv2.FILLED)
            cv2.circle(image, (mx, my), 7, (255,0,255), cv2.FILLED)
            cv2.line(image, (x1Middle, y1Middle), (x2Middle, y2Middle), (255, 0, 255), 3)

            lengthMiddle = math.hypot(x1Middle - x2Middle, y1Middle - y2Middle)

            if lengthMiddle < 50:
                cv2.circle(image, (mx, my), 7, (0, 255, 0), cv2.FILLED)

            # --------------------------------------Ring Finger----------------------------------------------------
            x1Ring, y1Ring = lmLists[16][1], lmLists[16][2]
            x2Ring, y2Ring = lmLists[13][1], lmLists[13][2]

            rx, ry = (x1Ring + x2Ring)//2, (y1Ring + y2Ring)//2

            cv2.circle(image, (x1Ring, y1Ring), 7, (255,0,255), cv2.FILLED)
            cv2.circle(image, (x2Ring, y2Ring), 7, (255,0,255), cv2.FILLED)
            cv2.circle(image, (rx, ry), 7, (255,0,255), cv2.FILLED)
            cv2.line(image, (x1Ring, y1Ring), (x2Ring, y2Ring), (255, 0, 255), 3)

            lengthRing = math.hypot(x1Ring - x2Ring, y1Ring - y2Ring)

            if lengthRing < 50:
                cv2.circle(image, (rx, ry), 7, (0, 255, 0), cv2.FILLED)

            #---------------------------------------Pinky Finger-------------------------------------------------------
            x1Pinky, y1Pinky= lmLists[20][1], lmLists[20][2]
            x2Pinky, y2Pinky = lmLists[17][1], lmLists[17][2]

            px, py = (x1Pinky + x2Pinky)//2, (y1Pinky + y2Pinky)//2

            cv2.circle(image, (x1Pinky, y1Pinky), 7, (255,0,255), cv2.FILLED)
            cv2.circle(image, (x2Pinky, y2Pinky), 7, (255,0,255), cv2.FILLED)
            cv2.circle(image, (px, py), 7, (255,0,255), cv2.FILLED)
            cv2.line(image, (x1Pinky, y1Pinky), (x2Pinky, y2Pinky), (255, 0, 255), 3)

            lengthPinky = math.hypot(x1Pinky - x2Pinky, y1Pinky - y2Pinky)

            if lengthPinky < 50:
                cv2.circle(image, (px, py), 7, (0, 255, 0), cv2.FILLED)

            #--------------------------------------Thumb Finger--------------------------------------------------------
            x1Thumb, y1Thumb= lmLists[4][1], lmLists[4][2]
            x2Thumb, y2Thumb = lmLists[17][1], lmLists[17][2]

            tx, ty = (x1Thumb + x2Thumb)//2, (y1Thumb + y2Thumb)//2

            cv2.circle(image, (x1Thumb, y1Thumb), 7, (255,0,255), cv2.FILLED)
            cv2.circle(image, (x2Thumb, y2Thumb), 7, (255,0,255), cv2.FILLED)
            cv2.circle(image, (tx, ty), 7, (255,0,255), cv2.FILLED)
            cv2.line(image, (x1Thumb, y1Thumb), (x2Thumb, y2Thumb), (255, 0, 255), 3)

            lengthThumb = math.hypot(x1Thumb - x2Thumb, y1Thumb - y2Thumb)

            if lengthThumb < 50:
                cv2.circle(image, (tx, ty), 7, (0, 255, 0), cv2.FILLED)
            
            # index Finger 25 - 100
            pointLengthIndex = numpy.interp(lengthIndex, [25, 100], [400, 150])
            servoAngleIndex = numpy.interp(lengthIndex, [25, 100], [0, 170])
            pinIndex.write(servoAngleIndex)

            # Middle Finger 25 - 100
            pointLengthMiddle = numpy.interp(lengthMiddle, [25, 100], [400, 150])
            servoAngleMiddle = numpy.interp(lengthMiddle, [25, 100], [0, 170])
            pinMiddle.write(servoAngleMiddle)

            #Ring Finger 25 - 100
            pointLengthRing = numpy.interp(lengthRing, [25, 100], [400, 150])
            servoAngleRing = numpy.interp(lengthRing, [25, 100], [0, 170])
            pinRing.write(servoAngleRing)

            # Pinky Finger 25 - 100
            pointLengthPinky = numpy.interp(lengthPinky, [25, 100], [400, 150])
            servoAnglePinky = numpy.interp(lengthPinky, [25, 100], [0, 170])
            pinPinky.write(servoAnglePinky)

            #thumb Finger 25 - 100
            pointLengthThumb = numpy.interp(lengthThumb, [50, 130], [400, 150])
            servoAngleThumb = numpy.interp(lengthThumb, [50, 130], [0, 170])
            pinThumb.write(servoAngleThumb)
            
            # cv2.rectangle(image, (50, 150), (85, 400), (255, 0, 0), 3)
            # cv2.rectangle(image, (50, int(pointLength)), (85, 400), (255, 0, 0), cv2.FILLED)
            # cv2.putText(image, f'{int(servoAngleThumb)}%', (40,450), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,0), 3)
        
            # print(fingers)
                
        CTime = time.time()
        fps = 1/(CTime-pTime)
        pTime = CTime
        cv2.putText(image, f'{int(fps)}',(20,40),cv2.FONT_HERSHEY_PLAIN,3,(255,0,0),3)

        cv2.imshow("image",image)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()  

if __name__ == '__main__':
    main()