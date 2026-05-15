# =========================================================
# AUTONOMOUS FOOTBALL ROBOT
# BALL TRACKING + WALK + TURN + KICK
# =========================================================

import cv2
import numpy as np
import serial
import time

# =========================================================
# STM32 SERIAL
# =========================================================

stm32 = serial.Serial(
    'COM4',
    115200,
    timeout=1
)

time.sleep(2)

# =========================================================
# BALL PARAMETERS
# =========================================================

BALL_DIAMETER_CM = 6.7
FOCAL_LENGTH = 700

LOWER_COLOR = np.array([20, 70, 70])
UPPER_COLOR = np.array([45, 255, 255])

# =========================================================

def estimate_distance(radius_px):

    if radius_px <= 0:
        return 999

    return (BALL_DIAMETER_CM * FOCAL_LENGTH) / (2 * radius_px)

# =========================================================

cap = cv2.VideoCapture(1)

if not cap.isOpened():
    cap = cv2.VideoCapture(0)

last_command = ""

while True:

    ret, frame = cap.read()

    if not ret:
        continue

    frame = cv2.flip(frame, 1)

    blurred = cv2.GaussianBlur(frame, (11,11), 0)

    hsv = cv2.cvtColor(
        blurred,
        cv2.COLOR_BGR2HSV
    )

    mask = cv2.inRange(
        hsv,
        LOWER_COLOR,
        UPPER_COLOR
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    frame_center = frame.shape[1] // 2

    command = ""

    if len(contours) > 0:

        c = max(contours, key=cv2.contourArea)

        area = cv2.contourArea(c)

        if area > 1500:

            ((x, y), radius) = cv2.minEnclosingCircle(c)

            center_x = int(x)

            distance = estimate_distance(radius)

            # draw ball
            cv2.circle(
                frame,
                (int(x), int(y)),
                int(radius),
                (0,255,0),
                3
            )

            cv2.line(
                frame,
                (frame_center, 0),
                (frame_center, frame.shape[0]),
                (255,0,0),
                2
            )

            cv2.putText(
                frame,
                f"{distance:.1f} cm",
                (30,50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0,255,0),
                3
            )

            # =================================================
            # DECISION LOGIC
            # =================================================

            # Ball on left
            if center_x < frame_center - 80:

                command = "L"

            # Ball on right
            elif center_x > frame_center + 80:

                command = "R"

            # Ball centered
            else:

                # close enough -> kick
                if distance < 25:

                    command = "K"

                # otherwise walk forward
                else:

                    command = "F"

    # =====================================================
    # SEND COMMAND
    # =====================================================

    if command != "" and command != last_command:

        stm32.write((command + "\n").encode())

        print("COMMAND:", command)

        last_command = command

    cv2.imshow("Football Robot", frame)

    key = cv2.waitKey(1)

    if key == 27:
        break

cap.release()

cv2.destroyAllWindows()