# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Single Color RGB565 Blob Tracking Example
#
# This example shows off single color RGB565 tracking using the OpenMV Cam.

import pyb
uart = pyb.UART(1, 9600)

import sensor
import time
import math

red_index = 0  # 0 for red,
green_index = 1 # 1 for green,

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [
    (35, 70, 39, 72, -27, 127),  # generic_red_thresholds
    (22, 70, -37, -8, -28, 24),  # generic_green_thresholds
]

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.
while True:
    clock.tick()
    MaxRedPixels = 0
    ClosestBlob = -1;
    img = sensor.snapshot()
    for blob in img.find_blobs(
        [thresholds[red_index]],
        pixels_threshold=200,
        area_threshold=200,
        merge=True,
    ):
        if MaxRedPixels < blob.pixels():
            MaxRedPixels = blob.pixels()
            ClosestBlob = blob
    #print(clock.fps())

    if ClosestBlob != -1:
        if ClosestBlob.elongation() > 0.5:
            img.draw_edges(ClosestBlob.min_corners(), color=(255, 0, 0))
            img.draw_line(ClosestBlob.major_axis_line(), color=(0, 255, 0))
            img.draw_line(ClosestBlob.minor_axis_line(), color=(0, 0, 255))
        # These values are stable all the time.
        img.draw_rectangle(ClosestBlob.rect())
        img.draw_cross(ClosestBlob.cx(), ClosestBlob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints(
        [(ClosestBlob.cx(), ClosestBlob.cy(), int(math.degrees(ClosestBlob.rotation())))], size=20
        )
        uart.write(f"{ClosestBlob.cx()} : {ClosestBlob.cy()}\000")
