# IMPORTING REQUIRED LIBRARIES
import cv2
import urllib.request
import mediapipe as mp
import numpy as np


# FUNCTION TO SEND DIRECTION OF MOVEMENT TO ESP32-CAM
def direction(nose, mid_sec, feet, frame):
    global url

    # TO ALIGN THE ROBOT WITH HUMAN
    if mid_sec[0] <= DZ_left or mid_sec[0] >= DZ_right:

        # MOVE LEFT
        if mid_sec[0] <= DZ_left:
            resp = urllib.request.urlopen(url + 'LEFT')

            cv2.putText(frame, "LEFT", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 3)

        # MOVE RIGHT
        elif mid_sec[0] >= DZ_right:
            resp = urllib.request.urlopen(url + 'RIGHT')
            cv2.putText(frame, "RIGHT", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 3)

    # ONCE ROBOT IS ALIGNED WITH HUMAN, MOVE IT FORWARD OR STAY
    else:

        # STAY
        if nose[1] < DZ_top and feet[1] > DZ_bottom:
            resp = urllib.request.urlopen(url + 'STAY')
            cv2.putText(frame, "STAY", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 3)

        # MOVE FORWARD
        else:
            resp = urllib.request.urlopen(url + 'FORWARD')
            cv2.putText(frame, "FORWARD", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 3)


mpDraw = mp.solutions.drawing_utils
mpPose = mp.solutions.pose
pose = mpPose.Pose()

# WRITE THE IP ADDRESS GIVEN BY YOUR ESP32-CAM
url = 'http://192.168.1.61/'

#cap = cv2.VideoCapture(0)

DZ_left = 180
DZ_right = 460
DZ_top = 100
DZ_bottom = 380

while True:
    img_resp = urllib.request.urlopen(url+'cam-hi.jpg')
    imgnp = np.array(bytearray(img_resp.read()),dtype = np.uint8)
    frame = cv2.imdecode(imgnp,-1)
    #_, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    frame = cv2.flip(frame, 1)

    # DRAWING THE DEADZONE
    frame = cv2.rectangle(frame, (DZ_left, DZ_top), (DZ_right, DZ_bottom), (0, 255, 0), 4)
    
    # DETECTING HUMAN
    frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frameRGB)
    
    # IF HUMAN IS FOUND THEN FIND ITS NOSE, MID-SEC AND FEET
    if results.pose_landmarks:
        # mpDraw.draw_landmarks(frame, results.pose_landmarks, mpPose.POSE_CONNECTIONS)

        landmarks = results.pose_landmarks.landmark
        h, w, c = frame.shape
        
        # NOSE COORDINATES
        nose = (int(landmarks[0].x * w), int(landmarks[0].y * h))
        
        # MID-SECTION COORDINATES
        mid_sec = (int((landmarks[24].x * w + landmarks[23].x * w) / 2),
                   int((landmarks[24].y * h + landmarks[23].y * h) / 2))
        
        # FEET COORDINATES
        feet = (int((landmarks[28].x * w + landmarks[27].x * w) / 2),
                int((landmarks[28].y * h + landmarks[27].y * h) / 2))
        
        # VISUALIZING NOSE
        frame = cv2.circle(frame, nose, 5, (255, 0, 255), -1)
        cv2.putText(frame, "Nose", nose,
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        # VISUALIZING MID_SECTION
        frame = cv2.circle(frame, mid_sec, 5, (255, 0, 255), -1)
        cv2.putText(frame, "Mid-Section", mid_sec,
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        # VISUALIZING FEET
        frame = cv2.circle(frame, feet, 5, (255, 0, 255), -1)
        cv2.putText(frame, "Feet", feet,
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        # FUNCTION TO GIVE THE DIRECTION TO MOVE TOWARDS HUMAN
        direction(nose, mid_sec, feet, frame)
    
    # IF HUMAN IS NOT DETECTED THEN DON'T MOVE
    else:
        resp = urllib.request.urlopen(url + 'STAY')
        cv2.putText(frame, "STAY", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 3)

    cv2.imshow("WIN", frame)
    if (cv2.waitKey(10) == ord('q')):
        break

cv2.destroyAllWindows()
