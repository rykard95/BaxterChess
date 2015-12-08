#! /usr/bin/python
import argparse
from baxter_interface import camera as baxter_cam


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
    balance = map(int, args.balance.split(','))

    c = baxter_cam.CameraController('left_hand_camera')
    c.resolution = resolution
    if args.exposure != -1: c.exposure = args.exposure
    if args.gain != -1: c.gain = args.gain
    if balance[0] != -1: c.white_balance_red = balance[0]
    if balance[1] != -1: c.white_balance_green = balance[1]
    if balance[2] != -1: c.white_balance_blue = balance[2]
