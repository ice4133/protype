#!/usr/bin/env python3
import cv2
import time
import numpy as np

def main():
    # Pre-create image buffer
    img = np.zeros((400, 800, 3), dtype=np.uint8)

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 5
    thickness = 10

    # Pre-calculate text position (for 5 digits)
    (text_width, text_height), _ = cv2.getTextSize("00000", font, font_scale, thickness)
    x = (800 - text_width) // 2
    y = (400 + text_height) // 2

    cv2.namedWindow('MS Timer', cv2.WINDOW_AUTOSIZE)

    while True:
        # Clear only (faster than creating new array)
        img[:] = 0

        # Get current time in milliseconds
        ms = int(time.time() * 1000) % 100000

        cv2.putText(img, f"{ms:05d}", (x, y), font, font_scale, (0, 255, 0), thickness)
        cv2.imshow('MS Timer', img)

        # waitKey(10) = ~100Hz
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
