
# import camera
import AttitudeDetermination
import extractDescpriptors
import createStarMap
from time import sleep
import os
import threading
import traceback
import StarSift as ss
import serial

folder = "star_data"

visualisation = True
uart = True
ser = None
if uart:
    ser = serial.Serial('/dev/serial0', 9600)  # open serial port


def read_int(text):
    if uart:
        #read one byte
        return ser.read()
    else:
        return int(input(text))
    
def send_int(val):
    if uart:
        ser.write(val)
    else:
        print(f"UART got value: {val}")


while True:
    try:
        # TODO replace this with uart command
        
        text = "1) Take pictures and get descriptors\n2) Stitch star map\n3) Find current angle\n4) Set working folder\n5) Redo descriptors\n"
        val = read_int(text)

        if val == 1:
            # check if folder exist
            if not os.path.exists(folder):
                os.makedirs(folder)

            sleep_time = 5
            start_angle = 0
            max_angle = 360
            increment = 15
            angle = start_angle

            # camera.init()
            # while(angle <= max_angle):
            #     print(f"angle set to {angle}")  # TODO sent uart command and wait for answer
            #     sleep(sleep_time) # wait for uart feedback from pointing
            #     image_path = os.path.join(folder, f"image_{angle}.jpg")
            #     camera.take_picture(image_path)
            #     # extractDescpriptors.analyse(image_path, folder)
            #     threading.Thread(target=extractDescpriptors.analyse, args=(image_path, folder, visualisation)).start()
            #     angle = angle + increment
            # camera.deinit()
            # print("Pictures taken and extracted descriptors")
        elif val == 2:
            createStarMap.createStarMap(folder, visualisation)
        elif val == 3:
            image_path = "current_270.jpg"
            # camera.init()
            # camera.take_picture(image_path)
            # camera.deinit()
            # image_path = "run-all copy/image_195.jpg"
            # TODO remove star map and visualization to improve performance
            attidude = AttitudeDetermination.find_attitude(folder, 
                                                        image_path,
                                                        visualisation)
            print(f"Attitude: {attidude}")
            send_int(attidude)
        elif val == 4:
            input = read_int("Enter folder name: ")
            folder = str(input)

            # check if folder exist
            if not os.path.exists(folder):
                os.makedirs(folder)

        elif val == 5:
            images = ss.get_image_paths(folder)
            for img in images:
                threading.Thread(target=extractDescpriptors.analyse, args=(img, folder, visualisation)).start()
        else:
            send_int(255)  # 255 is invalid command
        send_int(0)  # 0 is done
    except KeyboardInterrupt:
        print("Exiting")
        # camera.deinit()
        break
    except Exception as e:
        print("Error occured")
        print(traceback.format_exc())
        send_int(254)  # 254 is error
        # camera.deinit()