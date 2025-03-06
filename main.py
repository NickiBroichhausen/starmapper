
import AttitudeDetermination
import extractDescpriptors
import createStarMap
from time import sleep
import os
import threading
import traceback
import StarSift as ss

folder = "1"

if False:
    import camera

visualisation = True

def read_int(text):
    val = input(text)
    return val

def send_int(val):
    print(f"UART got value: {val}")


while True:
    try:
        text = "1) Take pictures and get descriptors\n2) Stitch star map\n3) Find current angle\n4) Set working folder\n5) Redo descriptors\n6)attitude for image\n"
        val = int(read_int(text))

        if val == 1:
            # check if folder exist
            if not os.path.exists(folder):
                os.makedirs(folder)

            sleep_time = 5
            start_angle = 0
            max_angle = 360
            increment = 15
            angle = start_angle

            camera.init()
            while(angle <= max_angle):
                send_int(angle)  # This is required angle
                read_int("Press any key to take picture") # wait until pointing is done
                image_path = os.path.join(folder, f"image_{angle}.jpg")
                camera.take_picture(image_path)
                threading.Thread(target=extractDescpriptors.analyse, args=(image_path, folder, visualisation)).start()
                angle = angle + increment
            camera.deinit()
            print("Pictures taken and extracted descriptors")
        elif val == 2:
            createStarMap.createStarMap(folder, visualisation)
        elif val == 3:
            image_path = "current.jpg"
            camera.init()
            camera.take_picture(image_path)
            camera.deinit()
            image_path = "run-all copy/image_195.jpg"
            # TODO remove star map and visualization to improve performance
            attidude = AttitudeDetermination.find_attitude(folder, 
                                                        image_path,
                                                        visualisation)
            send_int(attidude)
        elif val == 4:
            val = read_int("Enter folder name: ")
            folder = str(val)

            # check if folder exist
            if not os.path.exists(folder):
                os.makedirs(folder)

        elif val == 5:
            images = ss.get_image_paths(folder)
            for img in images:
                threading.Thread(target=extractDescpriptors.analyse, args=(img, folder, visualisation)).start()
        elif val == 6:
            image_path = str(read_int("Enter image path: "))
            # TODO remove star map and visualization to improve performance
            attidude = AttitudeDetermination.find_attitude(folder, 
                                                        image_path,
                                                        visualisation)
            send_int(attidude)
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
