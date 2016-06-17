#!/usr/bin/env python

'''
camera calibration for distorted images with chess board samples
reads distorted images, calculates the calibration and write undistorted images

usage:
    calibrate.py [--debug <output path>] [--square_size] [<image mask>]

default values:
    --debug:    ./output/
    --square_size: 1.0
    <image mask> defaults to ../data/left*.jpg
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2

# local modules
from common import splitfn

# built-in modules
import os

if __name__ == '__main__':
    import sys
    import getopt
    from glob import glob

    [args, img_mask] = getopt.getopt(sys.argv[1:], '', ['debug=', 'square_size='])
    args = dict(args)
    args.setdefault('--debug', './output/')
    args.setdefault('--square_size', 41.0)
    #args.setdefault('--square_size', 24.0)
    if not img_mask:
        img_mask = '../data/left*.jpg'  # default
    else:
        img_mask = img_mask[0]

    img_names = glob(img_mask)
    debug_dir = args.get('--debug')
    if not os.path.isdir(debug_dir):
        os.mkdir(debug_dir)
    square_size = float(args.get('--square_size'))

    pattern_size = (4, 4)
    #pattern_size = (7, 7)
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    obj_points = []
    img_points = []
    [h, w] = 0, 0
    img_names_undistort = []

    # Open camera
    #cap = cv2.VideoCapture(0)
    # Check if camera is available
    #if(False == cap.isOpened()):
    #    exit()

    #while(cap.isOpened()):
    #for fn in img_names:
    for fn in range(14):
        #print('processing %s... ' % fn, end='')
        #img = cv2.imread(fn, 0)
        #[ret, img] = cap.read()
        file_name = "./input/chessboard_" + str(fn) + ".png"
        #file_name = "./input/chessboard_logitech_" + str(fn) + ".jpg"
        img = cv2.imread(file_name)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img is None:
            print("Failed to load", fn)
            continue

        [h, w] = img.shape[:2]
        #cv2.imshow('img', img)
        #cv2.waitKey(5)

        [found, corners] = cv2.findChessboardCorners(img, pattern_size)
        if found:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)

        if debug_dir:
            vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            #vis = img
            cv2.drawChessboardCorners(vis, pattern_size, corners, found)
            cv2.imshow('corners', vis)
            cv2.waitKey(50)
            #path, name, ext = splitfn(fn)
            #outfile = debug_dir + name + fn + '_chess.png'
            outfile = debug_dir + str(fn) + '_chess.png'
            cv2.imwrite(outfile, img)
            if found:
                img_names_undistort.append(outfile)

        if not found:
            print('chessboard not found')
            continue

        img_points.append(corners.reshape(-1, 2))
        obj_points.append(pattern_points)

        print('ok')

    cv2.destroyAllWindows()

    print("Calculate camera distortion")
    # calculate camera distortion
    """
    fx = 1
    fy = 1
    camera_matrix_init = np.array([[fx, 0, 0], [0, fy, 0], [0, 0, 1]], dtype=float)
    [rms, camera_matrix, dist_coefs, rvecs, tvecs] = cv2.calibrateCamera(obj_points, img_points, (w, h), camera_matrix_init, None, None, None, cv2.CALIB_USE_INTRINSIC_GUESS)
    """
    [rms, camera_matrix, dist_coefs, rvecs, tvecs] = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None, None, None, cv2.CALIB_FIX_K3)

    [height, width] = img.shape
    print("\nRMS:", rms)
    print("image_width: ", width)
    print("image_height: ", height)
    print("camera matrix:\n", camera_matrix)
    print("distortion coefficients: ", dist_coefs.ravel())

    # undistort the image with the calibration
    print('')
    for img_found in img_names_undistort:
        img = cv2.imread(img_found)

        h,  w = img.shape[:2]
        [newcameramtx, roi] = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (w, h), 1, (w, h))

        dst = cv2.undistort(img, camera_matrix, dist_coefs, None, newcameramtx)

        # crop and save the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        outfile = img_found + '_undistorted.png'
        print('Undistorted image written to: %s' % outfile)
        cv2.imwrite(outfile, dst)

    #cv2.destroyAllWindows()
