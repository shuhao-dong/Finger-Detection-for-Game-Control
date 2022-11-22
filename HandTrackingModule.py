"""
Author: S. Dong University of Leeds
Date: 10/08/2021
Version: 0.1

This is the module script for hand tracking and controlling
"""

# common import
import cv2
import mediapipe as mp
import time
import math


class handDetector():
    def __init__(self, mode=False, maxHands = 2, detectionCon = 0.5, trackCon = 0.5):
        # parameters for hand tracking
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

        # parameters for UDP transfer
        # self.UDP_IP = "127.0.0.1"
        # self.UDP_PORT = 5065
        # self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # gesture recognition
        self.tipIds = [4, 8, 12, 16, 20]
        # fps calculation
        self.pTime = 0

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        imgRGB.flags.writeable = False
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handno=0):
        self.lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handno]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lmList.append([id, cx, cy])
        return self.lmList

    def findDistance(self, img, p1, p2, draw=True, radius=8, thickness=2):
        x1, y1 = self.lmList[p1][1:]
        x2, y2 = self.lmList[p2][1:]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if draw:
            cv2.circle(img, (x1, y1), radius, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), radius, (255, 0, 255), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), thickness)
            cv2.circle(img, (cx, cy), radius, (255, 0, 255), cv2.FILLED)

        length = int(math.hypot(x2 - x1, y2 - y1))
        return length, img, [x1, y1, x2, y2, cx, cy]

    def posRec(self, img, posList, draw=True):
        if 0 <= posList[1][2] < 200:
            # self.sock.sendto(("left").encode(), (self.UDP_IP, self.UDP_PORT))
            if draw:
                cv2.putText(img, "left", (200, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        elif 200 <= posList[1][2] <= 400:
            # self.sock.sendto(("still").encode(), (self.UDP_IP, self.UDP_PORT))
            if draw:
                cv2.putText(img, "middle", (200, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        elif 400 < posList[1][2] <= 600:
            # self.sock.sendto(("right").encode(), (self.UDP_IP, self.UDP_PORT))
            if draw:
                cv2.putText(img, "right", (200, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        return img

    def fingersUp(self, img):
        fingers = []
        # Thumb
        if self.lmList[self.tipIds[0]][1] < self.lmList[self.tipIds[0] - 1][1]:
            fingers.append(1)
        else:
            fingers.append(0)

        # Fingers
        for id in range(1, 5):
            if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)

        totalFingers = fingers.count(1)
        return fingers, img, totalFingers

    def calculateFPS(self, img):
        cTime = time.time()
        fps = 1/(cTime - self.pTime)
        self.pTime = cTime
        cv2.putText(img, str(int(fps)), (40, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        return fps, img
