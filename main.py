import math
import time
import cv2
import numpy as np
import serial
from detect_surface import detect
import os
import subprocess

# Var
L1, L2 = 0.10, 0.07
PIXEL_SIZE_MM, FOCAL_LENGTH_MM = 1.4e-3, 3.7
SHELF_DISTANCE_MM = 500
CAMERA_TO_ROBOT_ROT = np.eye(3)
CAMERA_TO_ROBOT_TRANS = np.array([0.0, 0.0, 0.0])
SERIAL_PORT, BAUD_RATE ='/dev/serial0', 115200
BOX_WIDTHS_MM = [120, 85, 150, 200]

class ThreeHingeIK:
    def __init__(self, l1, l2):
        self.l1, self.l2 = l1, l2
    def inverse_kinematics(self, x, y, z):
        yaw = math.atan2(y, x)
        r = math.hypot(x, y)
        d = math.hypot(r, z)
        cos_elbow = (d**2 - self.l1**2 - self.l2**2)/(2*self.l1*self.l2)
        if cos_elbow < -1 or cos_elbow > 1: raise ValueError(f"Unreachable {x,y,z}")
        elbow = math.acos(cos_elbow)
        cos_sh_off = max(-1, min(1,(d**2 + self.l1**2 - self.l2**2)/(2*self.l1*d)))
        shoulder = math.atan2(z, r) - math.acos(cos_sh_off)
        return yaw, shoulder, elbow
    @staticmethod
    def to_degrees(angles): return tuple(math.degrees(a) for a in angles)

class ShelfBoxPlanner:
    def __init__(self, shelf_z_mm, rot, trans):
        self.shelf_z_mm = shelf_z_mm
        self.rot, self.trans = rot, trans
        self.img_center = None
        self.pixels_per_mm = None
        self.available_mask = None
    def configure(self, mask, img_center, pixels_per_mm):
        self.available_mask = mask
        self.img_center = img_center
        self.pixels_per_mm = pixels_per_mm
    def place_box(self, box_width_mm):
        box_w_px = int(box_width_mm * self.pixels_per_mm)
        x, y, w, h = cv2.boundingRect(self.available_mask)
        if box_w_px > w: raise ValueError(f"Only {w/self.pixels_per_mm:.1f}mm left")
        cx, cy = x + box_w_px//2, y + h//2
        self.available_mask[y:y+h, x:x+box_w_px] = 0
        return cx, cy
    def pixel_to_robot(self, cx, cy):
        dx_px = cx - self.img_center[0]
        dy_px = self.img_center[1] - cy
        mm_per_px = (self.shelf_z_mm * PIXEL_SIZE_MM)/FOCAL_LENGTH_MM
        cam_pt = np.array([dx_px*mm_per_px, dy_px*mm_per_px, self.shelf_z_mm])
        return (self.rot.dot(cam_pt)+self.trans)/1000.0

def capture_image():
    path = 'image.jpg'
    cmd  = ['libcamera-still', '-t', '1000', '-o' ,'image.jpg'] 
    subprocess.run(cmd, check=True)
    return cv2.imread(path)

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(0.5)
    ik = ThreeHingeIK(L1, L2)
    planner = ShelfBoxPlanner(SHELF_DISTANCE_MM, CAMERA_TO_ROBOT_ROT, CAMERA_TO_ROBOT_TRANS)
    frame = capture_image()
    hull_r, _ = detect(frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = np.zeros_like(gray)
    cv2.fillPoly(mask, [hull_r.astype(int)], 255)
    img_center = (frame.shape[1]/2, frame.shape[0]/2)
    pixels_per_mm = FOCAL_LENGTH_MM/(SHELF_DISTANCE_MM*PIXEL_SIZE_MM)
    planner.configure(mask, img_center, pixels_per_mm)
    for w in BOX_WIDTHS_MM:
        cx, cy = planner.place_box(w)
        X, Y, Z = planner.pixel_to_robot(cx, cy)
        yaw, sh, el = ik.inverse_kinematics(X, Y, Z)
        yd, sd, ed = ik.to_degrees((yaw, sh, el))
        ser.write(f"{yd:.1f},{sd:.1f},{ed:.1f}\n".encode())
        time.sleep(0.1)
    ser.close()

if __name__=="__main__":
    main()

