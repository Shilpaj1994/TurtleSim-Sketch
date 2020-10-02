#!/usr/bin/env python3.8


import cv2
import time
import sys
import numpy as np
import pyscreenshot as ImageGrab


class ScreenGrab:
    def __init__(self, top_left, bottom_right):
        self.x1 = top_left[0]
        self.y1 = top_left[1]
        self.x2 = bottom_right[0]
        self.y2 = bottom_right[1]

        while True:
            self.img = ImageGrab.grab(bbox=(self.x1, self.y1, self.x2, self.y2))
            self.img = np.array(self.img)
            self.img = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)

            cv2.imshow('Out', self.img)

            if cv2.waitKey(1) == 32:
                cv2.imwrite('/home/shilpaj/Pictures/frame.png', self.img)

            if cv2.waitKey(1) == 27:
                break


def impose():
    try:
        img = cv2.imread('/home/shilpaj/Pictures/IMG_top.PNG')
        frame = cv2.imread('/home/shilpaj/Pictures/frame.png')

        img = cv2.resize(img, (500, 500))
        frame = cv2.resize(frame, (500, 500))

        if img is None:
            print("Error loading an image")
            exit(-1)
        elif frame is None:
            print("Error loading a frame")
            exit(-1)

        alpha = 0.1

        while True:
            cv2.imshow("Blended", frame)
            if cv2.waitKey(1) == 32:
                break

        for i in range(1000):
            alpha += 0.001
            beta = 1.0 - alpha
            output = cv2.addWeighted(img, alpha, frame, beta, 0.0)

            cv2.imshow("Blended", output)
            cv2.waitKey(1)

        time.sleep(5)

    except Exception as e:
        print(e)


def main(x1=0, y1=90, x2=500, y2=590):
    global mode
    try:
        if mode == 'Generate':
            top_left = (x1, y1)
            bottom_right = (x2, y2)
            ScreenGrab(top_left, bottom_right)
        elif mode == 'Blend':
            impose()
    except KeyboardInterrupt:
        exit()
    finally:
        cv2.destroyAllWindows()
        exit()


if __name__ == '__main__':
    mode = 'Blend'
    # mode = 'Generate'

    if mode == 'Generate':
        main(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]))
    elif mode == 'Blend':
        main()
