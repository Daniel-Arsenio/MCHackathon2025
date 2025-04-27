import cv2
import numpy as np


# HSV thresholds
red_lower1 = np.array([0, 150, 120]);   red_upper1 = np.array([10, 255, 255])
red_lower2 = np.array([170, 150, 120]); red_upper2 = np.array([180, 255, 255])
green_lower = np.array([30, 50, 50]);   green_upper = np.array([90, 255, 255])

kernel = np.ones((5,5), np.uint8)


def detect(img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        mask_r = cv2.inRange(hsv, np.array([0,150,120]), np.array([10,255,255])) | cv2.inRange(hsv, np.array([170,150,120]), np.array([180,255,255]))
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, k)
        cnts_r,_ = cv2.findContours(mask_r, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pts_r = []
        for c in cnts_r:
            if cv2.contourArea(c) > 100:
                e = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.02*e, True)
                if len(approx) == 4:
                    M = cv2.moments(approx)
                    if M['m00'] > 0:
                        pts_r.append((M['m10']/M['m00'], M['m01']/M['m00']))
        hull_r = cv2.convexHull(np.array(pts_r, np.float32)) if len(pts_r) >= 3 else None

        mask_g = cv2.inRange(hsv, np.array([30,50,50]), np.array([90,255,255]))
        mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_OPEN, k)
        cnts_g,_ = cv2.findContours(mask_g, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pts_g = []
        for c in cnts_g:
            if cv2.contourArea(c) > 100:
                e = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.02*e, True)
                if len(approx) == 4:
                    M = cv2.moments(approx)
                    if M['m00'] > 0:
                        pts_g.append((M['m10']/M['m00'], M['m01']/M['m00']))
        hull_g = cv2.convexHull(np.array(pts_g, np.float32)) if len(pts_g) >= 3 else None
        return hull_r, hull_g