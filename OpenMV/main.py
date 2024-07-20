'''

https://github.com/tomertomism
Copyright 2024 Tomertomism

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
'''
import sensor
from machine import UART

cylinder_threshold = 5
cylinder_roi = [0,100, 320, 140]
cylinder = [(43, 77, -1, -18, 19, 72),
            (30, 83, 60, 15, 15, 60),
            (0, 60, 25, 5, -11, -32)]
boarder_thresold = 10
boarder_roiL = [30, 100, 35, 20]
boarder_roiR = [250, 100, 35, 20]
boarder = [(0, 30, 37, -19, 20, -25)]
turn_right_roi = [0, 115, 320, 30]
turn_right_target = [(0, 30, 37, -19, 20, -25)]
unload_box_roi = [65, 90, 185, 10]
unload_box = [(0, 30, 37, -19, 20, -25)]
car_reference = [160, 180]
stainless_steel = [(0, 35, 10, -10, 20, -6)]

debug_en = False
end_game = 0

uart = UART(1, 921600)

a = ' '
current_state = 1

def init():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(25)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)

def commu():
    global a
    global current_state
    if uart.any():
        try:
            a += uart.readline().decode().strip()
            if debug_en is True: print(a)
        except UnicodeError:
            uart.write('Z')
            if debug_en is True: print("UnicodeError")

        if a.find("SC0") != -1:
            current_state = 0
            init()
            uart.write('K')
            a = ''
        if a.find("SC1") != -1:
            current_state = 1
            uart.write('K')
            a = ''
        if a.find("SC2") != -1:
            current_state = 2
            uart.write('K')
            a = ''
        if a.find("SC3") != -1:
            current_state = 3
            uart.write('K')
            a = ''
        if a.find("SC4") != -1:
            current_state = 4
            uart.write('K')
            a = ''
        if a.find("STA") != -1:
            uart.write(str(current_state))
            a = ''
        if a.find("RQ1") != -1:
            uart.write(find_cylinder(img))
            a = ''
        if a.find("RQ2") != -1:
            uart.write(find_boarder(img))
            a = ''
        if a.find("RQ3") != -1:
            uart.write(turn_right(img))
            a = ''
        if a.find("RQ4") != -1:
            uart.write(find_stop_area(img))
            a = ''

def turn_in_mid(obj1x):
    global car_reference
    diff = obj1x - car_reference[0]
    if diff > cylinder_threshold:
        return 'L'
    elif diff < (cylinder_threshold * -1):
        return 'R'
    else:
        return 'S'

def find_cylinder(img):
    global a
    ch = ''
    if debug_en is True: img.draw_rectangle(cylinder_roi)
    cylinder_blob = img.find_blobs(cylinder, roi = cylinder_roi, x_stride = 10,
                                   y_stride = 10, merge = True)
    if cylinder_blob:
        b = cylinder_blob[0]
        if debug_en is True:
            img.draw_cross(b[5], b[6])
            print(b)
        ch = turn_in_mid(b[5])
        if(b[6] > 200 and ch.find('S') != -1):
            return 'G'
        return ch
    else:
        return 'N'

def find_boarder(img):
    boarder_blobL = img.find_blobs(boarder, roi = boarder_roiL, x_stride = 5,
                                   y_stride = 15, margin = 25)
    boarder_blobR = img.find_blobs(boarder, roi = boarder_roiR, x_stride = 5,
                                   y_stride = 15, margin = 25)
    if debug_en is True:
        img.draw_rectangle(boarder_roiL,(0, 255, 0), 1)
        img.draw_rectangle(boarder_roiR,(0, 255, 0), 1)
    if boarder_blobL and boarder_blobR:
        if debug_en is True:
            b = boarder_blobL[0]
            img.draw_cross(b[5], b[6])
            b = boarder_blobR[0]
            img.draw_cross(b[5], b[6])
            print('S1')
        return 'S'
    if boarder_blobL:
        if debug_en is True:
            b = boarder_blobL[0]
            img.draw_cross(b[5], b[6])
            #print(b)
            print('R')
        return 'R'
    if boarder_blobR:
        if debug_en is True:
            b = boarder_blobR[0]
            img.draw_cross(b[5], b[6])
            print('L')
        return 'L'
    if debug_en is True: print('S2')
    return 'S'

def turn_right(img):
    turn_right_blob = img.find_blobs(turn_right_target, roi = turn_right_roi,
                                     pixels_thresold = 2800)
    img.draw_rectangle(turn_right_roi)
    if turn_right_blob:
        for b in turn_right_blob:
            if b.pixels() > 500 and b.w() > 110:
                if debug_en is True: img.draw_cross(b[5], b[6])
                if debug_en is True: img.draw_rectangle(b.rect())
                if debug_en is True: print(b)
                return 'T'
    return find_boarder(img)

def find_stop_area(img):
    global end_game
    stop_area = img.find_blobs(unload_box, roi = unload_box_roi)
    img.draw_rectangle(unload_box_roi, (0, 0, 255))
    if stop_area:
        b = stop_area[0]
        if debug_en is True: print(b)
        if b.pixels() > 300:
            end_game = True
            if debug_en is True:
                img.draw_rectangle(b.rect())
                img.draw_cross(b[5], b[6])
            return 'N'
    if end_game is False:
        return find_boarder(img)
    else:
        return 'N'

init()
while True:
    img = sensor.snapshot().lens_corr(strength = 2.2, zoom = 0.8)
    if debug_en is True: img.draw_circle(car_reference[0], car_reference[1], 1, (0, 255, 0), 3)
    commu()
    if debug_en is True:
        if(current_state == 1): print(find_cylinder(img))
        if(current_state == 2): print(find_boarder(img))
        if(current_state == 3): print(turn_right(img))
        if(current_state == 4): print(find_stop_area(img))
