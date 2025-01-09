
# import camera
import AttitudeDetermination
import extractDescpriptors
import createStarMap
from time import sleep
import os
import threading
import traceback
import StarSift as ss


# Reciecve UART
# Take pictures
# get descriptors
# stitch star map
# find current angle

# maybe define run? folder?

# TODO configure these
# folder = "star_data"
folder = "run-all"
visualisation = True

while True:
    try:
        # TODO replace this with uart command
        val = input("1) Take pictures and get descriptors\n2) Stitch star map\n3) Find current angle\n4) backup\n5) Redo descriptors\n")

        if val == "1":
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
            print("Pictures taken and extracted descriptors")
        elif val == "2":
            createStarMap.createStarMap(folder, visualisation)
        elif val == "3":
            # image_path = "current"
            # camera.init()
            # camera.take_picture(image_path)
            # camera.deinit()
            image_path = "run-all copy/image_195.jpg"
            # TODO remove star map and visualization to improve performance
            attidude = AttitudeDetermination.find_attitude(folder, 
                                                        image_path,
                                                        visualisation)
            print(f"Attitude: {attidude}") # TODO send this with UART
        elif val == "4":
            # delete old backup
            # copy to backup folder
            # create new folder
            pass
        elif val == "5":
            images = ss.get_image_paths(folder)
            for img in images:
                threading.Thread(target=extractDescpriptors.analyse, args=(img, folder, visualisation)).start()
        else:
            print("Invalid input")
    except KeyboardInterrupt:
        print("Exiting")
        # camera.deinit()
        break
    except Exception as e:
        print("Error occured")
        print(traceback.format_exc())
        # camera.deinit()