#! /usr/bin/python
import argparse
from baxter_interface.camera import *


if __name__ == '__main__':
    desc = 'Script to set up the camera parameters.'
    parser = argparse.ArgumentParser(description = desc)
    parser.add_argument('-x', '--exposure', type=int)
    parser.add_argument('-r', '--resolution', default='1280x800')
    parser.add_argument('-g', '--gain', type=int)
    parser.add_argument('-b', '--balance', default='-1,-1,-1')
    parser.add_argument('-c', '--camera', default='left_hand_camera')
    args = parser.parse_args()

    resolution = tuple(map(int, args.resolution.split('x')))
    br, bg, bb = map(int, args.balance.split(','))
    x = args.exposure
    g = args.gain

    c = CameraController('left_hand_camera')
    if resolution in CameraController.MODES:
        c._settings.width, c._settings.height = resolution

    def set_control_if_in(cnt, min, max, val):
        if val >= min and val <= max:
            c._set_control_value(cnt, val)
    set_control_if_in(CameraControl.CAMERA_CONTROL_EXPOSURE, 0, 100, x)
    set_control_if_in(CameraControl.CAMERA_CONTROL_GAIN, 0, 79, g)
    set_control_if_in(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_R, 0, 4095, br)
    set_control_if_in(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_R, 0, 4095, bg)
    set_control_if_in(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_R, 0, 4095, bb)

    if c._open: c.close()
    c.open()


