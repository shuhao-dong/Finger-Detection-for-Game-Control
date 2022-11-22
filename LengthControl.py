"""
Author: S.Dong from University of Leeds
Date: 12/08/2021
Version: 0.2

Update diary:
0.1: added UDP receiver and emitter
0.2: added centre tracking

Created for hand control for upper limb rehabilitation

This programme is used for controlling the mouse and the movement in Unity3D using
several landmarks on hand.
"""

# common import
import time
import cv2
import autopy
import numpy as np
import pyautogui
import HandTrackingModule as htm
import UDPReceiver as ur
import CentreFinder as cf
import math
import serial

# define the initial time for calculating FPS
cTime = 0
pTime = 0

# define the initial location of mouse
plocX, plocY = 0, 0
clocX, clocY = 0, 0

# define and set the space for camera
wCam, hCam = 640, 480
cap = cv2.VideoCapture(0)
cap.set(3, wCam)
cap.set(4, hCam)

# obtain the size of the screen
wSce, hSce = autopy.screen.size()

# define the smooth factor to smoothen the mouse movement
smoothF = 7  # default: 7

# initialise the hand detector
detector = htm.handDetector(maxHands=1, detectionCon=0.8)
print(time.time(), 'Hand detector initialised !!!')

# boolean variables for centre tracking
hist_created = False
is_draw_rect = True
centre = []

# speed and position
finger_speed = []
fingerxSpeed = []
fingerySpeed = []
mos_pos = []

# serial communication set up
# ser = serial.Serial('COM4', 115200)
data = []

# initialise the udp receiver and control
# udp = ur.udpReceiver()
# print('UDP transfer initialised, start the MATLAB programme for hand control.')

ismeasuring = True

while ismeasuring:

    # b = ser.readline()
    # string_n = b.decode()
    # string = string_n.rstrip()
    # result = string.split()
    # data.append(result)
    # print(time.time(), result)

    """
    UDP transfer and control
    1. forward: press enter
    2. left: press backspace
    3. right: press esc
    """

    # UDP transfer and control
    # str = udp.recString()
    # if str == 'forward':
    #     time.sleep(.1)
    #     pyautogui.press('enter')
    #     time.sleep(.1)
    # elif str == 'left':
    #     time.sleep(.1)
    #     pyautogui.press('backspace')
    #     time.sleep(.1)
    # elif str == 'right':
    #     time.sleep(.1)
    #     pyautogui.press('esc')
    #     time.sleep(.1)

    success, img = cap.read()
    img = cv2.flip(img, 1)

    img = detector.findHands(img)

    pressed_key = cv2.waitKey(1)

    # find the position of the landmark
    lmList = detector.findPosition(img)

    if len(lmList) != 0:
        # calculate the distance between thumb and index fingers
        l, img, p = detector.findDistance(img, 0, 4, False)

        # calculate the distance between all fingertips and origin
        disList = []
        positionList = []
        for finger in [4, 8, 12, 16, 20]:
            l, img, p = detector.findDistance(img, 0, finger, False)
            disList.append([l])
            positionList.append(p)

        # position recognition
        img = detector.posRec(img, positionList)

        # finger up detector
        fingers, img, total = detector.fingersUp(img)

        '''
        Below is the code for mouse control using various gestures
        1. index finger: control the position of the mouse
        2. middle finger + index finger up: right click
        3. thumb + index finger up: left click
        4. five or fist: scroll up or scroll down
        '''

        # mouse control
        x1, y1 = lmList[8][1:]  # the position of the index finger
        x2, y2 = lmList[12][1:]  # the position of the middle finger

        # regularise the working area, size controlled by frameR
        cv2.rectangle(img, (40, 20), (600, 334), (255, 0, 255), 2)

        # moving mode, when the index finger is up, the middle finger is not up
        if fingers[1] == 1 and fingers[2] == 0 and fingers[0] == 0 and fingers[3] == 0 and fingers[4] == 0:

            # coordinate transformation working: space ---> computer screen
            x3 = np.interp(x1, (40, 600), (0, wSce))
            y3 = np.interp(y1, (20, 334), (0, hSce))

            # smooth the movement of the mouse
            clocX = plocX + (x3 - plocX) / smoothF
            clocY = plocY + (y3 - plocY) / smoothF

            # move the mouse to the current location
            autopy.mouse.move(clocX, clocY)
            cv2.circle(img, (x1, y1), 8, (255, 0, 0), cv2.FILLED)

            # speed calculation
            distance = math.dist([plocX, plocY], [clocX, clocY])
            cTime = time.time()
            speed = distance / (cTime - pTime)

            # speed calculation per axis
            disX = (clocX - plocX)
            speedX = disX / (cTime - pTime)
            disY = (clocY - plocY)
            speedY = disY / (cTime - pTime)

            fingerxSpeed.append(speedX)
            fingerySpeed.append(speedY)

            pTime = cTime
            finger_speed.append(speed)

            # mouse position
            mouse_pos = pyautogui.position()
            mos_pos.append(mouse_pos)

            # update the previous location
            plocX, plocY = clocX, clocY

        # click mode, when the index finger and the thumb is up while others are down
        if fingers[1] == 1 and fingers[0] == 1 and fingers[2] == 0 and fingers[3] == 0 and fingers[4] == 0:
            length, img, lineinfo = detector.findDistance(img, 4, 8, draw=False)
            # print(length)

            if length >= 95:
                # cv2.circle(img, (lineinfo[4], lineinfo[5]), 10, (0, 255, 0), cv2.FILLED)
                pyautogui.click(button='left')
                time.sleep(.1)

        # click mode, when the index finger and the middle finger is up while others are down
        if fingers[1] == 1 and fingers[2] == 1 and fingers[0] == 0 and fingers[3] == 0 and fingers[4] == 0:
            length, img, lineinfo = detector.findDistance(img, 8, 12, draw=False)

            # set a threshold value to detect click
            if length <= 40:  # threshold may vary from people to people
                # cv2.circle(img, (lineinfo[4], lineinfo[5]), 10, (0, 255, 0), cv2.FILLED)
                pyautogui.click(button='right')
                time.sleep(.1)

        # scroll operation: fist to scroll down while five to scroll up
        # if fingers[0] == 0 and fingers[1] == 0 and fingers[2] == 0 and fingers[3] == 0 and fingers[4] == 0:
        #     pyautogui.scroll(-20)
        # elif fingers[0] == 1 and fingers[1] == 1 and fingers[2] == 1 and fingers[3] == 1 and fingers[4] == 1:
        #     pyautogui.scroll(20)

        # hand gesture
        if fingers[0] == 1 and fingers[1] == 0 and fingers[2] == 0 and fingers[3] == 0 and fingers[4] == 1:
            cv2.putText(img, "six", (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        elif fingers[0] == 1 and fingers[1] == 1 and fingers[2] == 1 and fingers[3] == 1 and fingers[4] == 1:
            cv2.putText(img, "five", (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        elif fingers[0] == 0 and fingers[1] == 1 and fingers[2] == 1 and fingers[3] == 1 and fingers[4] == 1:
            cv2.putText(img, "four", (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        elif fingers[0] == 1 and fingers[1] == 1 and fingers[2] == 1 and fingers[3] == 1 and fingers[4] == 0:
            cv2.putText(img, "four", (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        elif fingers[0] == 0 and fingers[1] == 0 and fingers[2] == 1 and fingers[3] == 1 and fingers[4] == 1:
            cv2.putText(img, "three", (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        elif fingers[0] == 0 and fingers[1] == 1 and fingers[2] == 1 and fingers[3] == 1 and fingers[4] == 0:
            cv2.putText(img, "three", (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        elif fingers[0] == 0 and fingers[1] == 1 and fingers[2] == 1 and fingers[3] == 0 and fingers[4] == 0:
            cv2.putText(img, "two", (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        elif fingers[0] == 0 and fingers[1] == 1 and fingers[2] == 0 and fingers[3] == 0 and fingers[4] == 0:
            cv2.putText(img, "one", (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        elif fingers[0] == 0 and fingers[1] == 0 and fingers[2] == 0 and fingers[3] == 0 and fingers[4] == 0:
            cv2.putText(img, "zero", (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    # find the centre
    if is_draw_rect:
        cf.draw_rect(img)

    # press z to capture the color of the hand
    if pressed_key & 0xFF == ord('z'):
        hand_hist = cf.hand_histogram(img)
        hist_created = True
        is_draw_rect = False

    # back projection method and output the centre to a list 'centre'.
    if hist_created:
        hist_mask_image = cf.hist_masking(img, hand_hist)
        contour_list = cf.contours(hist_mask_image)
        max_cont = max(contour_list, key=cv2.contourArea)
        cnt_centroid = cf.centroid(max_cont)
        cv2.circle(img, cnt_centroid, 8, [0, 255, 255], -1)
        centerList = list(cnt_centroid)
        centre.append(centerList)
        # print(type(centre), centre)

    else:
        img = cf.draw_rect(img)

    # calculate FPS
    fps, img = detector.calculateFPS(img)

    cv2.imshow("Live Inspector", img)
    cv2.waitKey(1)

    if pressed_key == 27:
        fingerspeedProf = np.array(finger_speed, dtype=object)
        np.savetxt('speed.csv', fingerspeedProf, delimiter=',', fmt='%s')

        fingerxSpeedProf = np.array(fingerxSpeed, dtype=object)
        np.savetxt('speedx.csv', fingerxSpeedProf, delimiter=',', fmt='%s')

        fingerySpeedProf = np.array(fingerySpeed, dtype=object)
        np.savetxt('speedy.csv', fingerySpeedProf, delimiter=',', fmt='%s')

        position = np.array(mos_pos, dtype=object)
        np.savetxt('position.csv', position, delimiter=',', fmt='%s')

        accel = np.array(data, dtype=object)
        # np.savetxt('acce.csv', accel, delimiter=',', fmt='%s')

        ismeasuring = False

cv2.destroyAllWindows()
cap.release()
