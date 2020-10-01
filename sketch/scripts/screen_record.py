#!/usr/bin/env python3.8


import cv2
import time
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

            if cv2.waitKey(1) == 27:
                break


if __name__ == '__main__':
    try:
        top_left = (0, 0)
        bottom_right = (500, 500)
        ScreenGrab(top_left, bottom_right)
    except KeyboardInterrupt:
        exit()
