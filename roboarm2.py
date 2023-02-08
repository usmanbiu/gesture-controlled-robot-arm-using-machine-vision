import cv2    #import neccesary libraries after adding them as packages to this project
import mediapipe as mp    # go to settings > python interpreter
import serial
import time
import math

Arduino=serial.Serial('/dev/cu.usbmodem14101', 9600, timeout=0.1)  #initialize the serial port(in quotation marks) for communicatin with the arduino

wCam, hCam = 1240, 720   # variable for setting the camera window width and height
cam = cv2.VideoCapture(0)  # start the webcam, use 0 for and inbuilt camera and 1 for an external one
cam.set(3, wCam)     #setting the camera window width and height
cam.set(4, hCam)
smoothV = 2
pVal = 0
cVal = 0

color = (255, 255, 255)

class mpHands:  #class used to detect hands, hand landmarks, measure distance and angle between hand landmarks
                # watche this video for an explanation of the class; https://www.youtube.com/watch?v=WQeoO7MI0Bs&t=8774s
    def __init__(self, mode=False, modelComplexity=1, maxHands=2, TrackCon=0.5, DetectCon=0.5):
        self.mode = mode
        self.modelComplexity = modelComplexity
        self.maxHands = maxHands
        self.TrackCon = TrackCon
        self.DetCon = DetectCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplexity,
                                        self.TrackCon, self.DetCon)
        self.mpDraw = mp.solutions.drawing_utils
    def Marks(self,frame):
        myHands=[]
        handsType=[]
        frameRGB=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        results=self.hands.process(frameRGB)
        if results.multi_hand_landmarks != None:
            for hand in results.multi_handedness:
                #print(hand)
                #print(hand.classification)
                #print(hand.classification[0])
                handType=hand.classification[0].label
                handsType.append(handType)
            for handLandMarks in results.multi_hand_landmarks:
                myHand=[]
                for landMark in handLandMarks.landmark:
                    h, w, c = frame.shape
                    myHand.append((int(landMark.x*w),int(landMark.y*h)))
                myHands.append(myHand)
        return myHands, handsType

    def findDistance(self, a, b, frame ):
        x1, y1 =a[0], a[1]
        x2, y2 =b[0], b[1]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # center point between thumb and index finger

        cv2.circle(frame, (x1, y1), 5, (255, 255, 255), cv2.FILLED)  # circle on thumb
        cv2.circle(frame, (x2, y2), 5, (255, 255, 255), cv2.FILLED)  # circle on index finger
        cv2.circle(frame, (cx, cy), 5, (255, 255, 255), cv2.FILLED)  # circle at center point bte thumb and index

        cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 3)  # line btw index and center point


        length = int(math.hypot(x2 - x1, y2 - y1))
        # print(length)
        return length, frame, [x1, y1, x2, y2, cx, cy]

    def findAngle(self, a, b, c, frame ):
        #get the landmarks
        x1, y1 = hand[a]
        x2, y2 = hand[b]
        x3, y3 = hand[c]
        #get the angle
        angle = math.degrees(math.atan2(y3-y2, x3-x2) - math.atan2(y1-y2, x1-x2))
        if angle<0:
            angle= angle+360
        return angle
        #print(angle)

width=1280
height=720
findHands=mpHands(2)

while True:
    ignore,  frames = cam.read()
    frame= cv2.flip(frames, 1) #fliping the camera output sideways, comment this line to see the difference
    handData, handType = findHands.Marks(frame)

    for hand, handType in zip(handData, handType):
        right = 0
        left = 0
        if handType == 'Right':
            handColor = (0, 0, 255)

                  #tracking landmark 0 located at the wrist (lower part of the palm)
            l1, l2 = hand[0]  # x and y coordinates for the point at the bottom of the palm (wrist)
            cv2.circle(frame, (l1, l2), 10, (0, 0, 255), cv2.FILLED)  # draw a red circle at the point
                                                                # this point controlls the rotation of the base of the arm
            cv2.line(frame, (50, l2), (220, l2), (46, 98, 84), 3)   #this lines draw a green, white and green line on the screen
            cv2.line(frame, (220, l2), (370, l2), (255, 255, 255), 3)    #movement of landmark 0 to the green part of the line makes the arm
            cv2.line(frame, (370, l2), (550, l2), (46, 98, 84), 3)          # rotate to the the left or right (when it crosses the limits triggers at the upper and lower points of the line (220 and 370 on the x axis respectively)

            cv2.circle(frame, (50, l2), 5, (255, 255, 255), cv2.FILLED)  #these lines draw a circle at the two ends of the gree, white and green line
            cv2.circle(frame, (550,l2), 5, (255, 255, 255), cv2.FILLED)  # they also draw circles at the two boundaries between the green and white line
            cv2.circle(frame, (220, l2), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (370, l2), 5, (255, 255, 255), cv2.FILLED)

            cv2.putText(frame, 'A', (l1,l2-50), cv2.FONT_HERSHEY_SIMPLEX,     # A is shown on the landmark being tracked
                                0.5, color, 2)
            cv2.putText(frame, 'B', (220, l2+50), cv2.FONT_HERSHEY_SIMPLEX,0.5, color, 2)  # prints B at the location of the leftward triggers
            cv2.putText(frame, 'c', (370, l2 + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)  # prints B at the location of the rightward triggers
            cv2.putText(frame, 'servo 0', (280, l2 + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2) # prints  the pin of servo being controlled

            if (51 <= l1 <= 499):   # the total length of the line created above is 448 (449-51) on the X axis
             J = l1                 # if the landmark being tracked is between the range above then it is stored with the variable J
            elif (l1 < 51):         # if the point being tracked (landmark 0) is at a position less than 51 on the X axis (the min point on the line drawn above
                J = 51          #it is still stored as 51 because that is the lower limit and this being absent would result in errors when transferring data to the arduino( nothing will be sent to the arduino and the python script is coded such that something must be sent, thats why errors will come up)
            elif (l1 > 499):   # the same applies here with the upper limit of 499
                J = 499

                # tracking landmark 8 located at the tip of the index finger, this controlls the shoulder joint
            l1, l2 = hand[8]   # the same logic used to track landmark 0 applies here
            cv2.circle(frame, (l1, l2), 10, (0, 0, 255), cv2.FILLED)

            cv2.line(frame, (l1, 20), (l1, 90), (46, 98, 84), 3)
            cv2.line(frame, (l1, 90), (l1, 150), (255, 255, 255), 3)
            cv2.line(frame, (l1, 150), (l1, 220), (46, 98, 84), 3)

            cv2.circle(frame, (l1, 20), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (l1, 220), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (l1, 90), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (l1, 150), 5, (255, 255, 255), cv2.FILLED)
            if (21 <= l2 <= 219):
              X = l2
            elif (l2 < 21):
                X = 21
            elif (l2 > 179):
                X = 179

                # tracking landmark 12 located at the tip of the middle finger, this controlls the elbow joint
            l1, l2 = hand[12]    # the same logic used to track landmark 0 applies here
            cv2.circle(frame, (l1, l2), 10, (0, 0, 255), cv2.FILLED)

            cv2.line(frame, (l1, 20), (l1, 70), (46, 98, 84), 3)
            cv2.line(frame, (l1, 70), (l1, 130), (255, 255, 255), 3)
            cv2.line(frame, (l1, 130), (l1, 180), (46, 98, 84), 3)

            cv2.circle(frame, (l1, 20), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (l1, 180), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (l1, 70), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (l1, 130), 5, (255, 255, 255), cv2.FILLED)
            if (21 <= l2 <= 179):
                 Y = l2
            elif (l2 < 21):
                Y = 21
            elif (l2 > 179):
                Y = 179

                # tracking landmark 16 located at the tip of the ring finger, this controlls the wrist joint
            l1, l2 = hand[16]    # the same logic used to track landmark 0 applies here
            cv2.circle(frame, (l1, l2), 10, (0, 0, 255), cv2.FILLED)

            cv2.line(frame, (l1, 20), (l1, 70), (46, 98, 84), 3)
            cv2.line(frame, (l1, 70), (l1, 130), (255, 255, 255), 3)
            cv2.line(frame, (l1, 130), (l1, 180), (46, 98, 84), 3)

            cv2.circle(frame, (l1, 20), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (l1, 180), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (l1, 70), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (l1, 130), 5, (255, 255, 255), cv2.FILLED)
            if (21 <= l2 <= 179):
                Z = l2
            elif (l2 < 21):
                Z = 21
            elif (l2 > 179):
                Z = 179


                         # this part id used to controll the suction pump. the suction pump is switched on or off depending on weather the tip of the thumb is positioned
                         # at he left or right of the joint below the tip of the thumb
            l1, l2 = hand[4]   #this represents the X and Y coordinated of tip of the thumb
            l3, l4 = hand[3]   #this represents the X and Y coordinated of joint below the tip of the thumb

            cv2.circle(frame, (l1, l2), 10, color, cv2.FILLED)  #this draws a circle at landmark 4, color was decleared in line 17

            cv2.line(frame, (l3, l4+20), (l3, l4-100), (255, 255, 255), 3)   #draws a white vertical line at landmark 3
            #cv2.line(frame, (65, l2), (80, l2), (255, 0, 0), 3)

            cv2.circle(frame, (l3, l4+20), 5, (255, 255, 255), cv2.FILLED)   # draws circles at the two ends of the line drawn above
            cv2.circle(frame, (l3, l4-100), 5, (255, 255, 255), cv2.FILLED)
            #cv2.circle(frame, (80, l2), 5, (255, 255, 255), cv2.FILLED)
            K = l1
            L = l3
            if ( l1 < l3):   # the color of the circle at the tip of the thumb changes to red or green
                color = (46, 98, 84)   # depending on its position (to the right or left of the joint below the thumb (landmark 3))
            elif (l1 > l3):           # this signifies weather the pupmp is powered on or off
                color = (0, 0, 255)


            values = 'X{0}Y{1}Z{2}J{3}K{4}L{5}'.format(X, Y, Z, J, K, L)   # all the variables used to store data are here
            Arduino.write(values.encode('utf-8'))    # the data containing the position of the landmarks being tracked are sent to the arduino

            print(values)  # the data being transferred is displayed




    cv2.imshow('my WEBcam', frame)
    #cv2.moveWindow('my WEBcam',0,0)
    if cv2.waitKey(1) & 0xff ==ord('q'):   # press the Q button on the keyboard to stop the camera
        break