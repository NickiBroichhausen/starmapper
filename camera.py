from picamzero import Camera
from time import sleep
# import os
# import extractDescpriptors

cam = None

def init(preview=False):
    global cam
    cam = Camera()
    cam.greyscale = True
    #cam.still_size = (2592, 1944)  #maybe reduce to make one star only a few pixels 
    cam.gain = 2.0  # Reduces noise. Smaller values reduce glare, but make dim stars less visible
    #cam.contrast = 1.1 # makes clearer destinction between dark and bright. Make high if all stars have same brightness, otherwise dont touch

    #cam.exposure = 1000000
    #cam.brightness = 0.2 # Post processing -> not helpful to change what sensor detects 
    #cam.white_balance = 'cloudy' #auto, tungsten, fluorescent, indoor, daylight, cloudy. Not relevant for bw image
    
    sleep(2)  # Wait for camera to warm up

    if preview:
        cam.start_preview()

# def take_panorama(folder, start_angle, max_angle, increment=15, sleep_time=5, preview=False):
#     print("WARNING DEPRECATED")
#     angle = start_angle
#     while(angle <= max_angle):
#         print(f"angle set to {angle}")
#         sleep(sleep_time)
#         image_path = os.path.join(folder, f"image_{angle}.jpg")
#         cam.take_photo(image_path)
#         extractDescpriptors.analyse(image_path, folder)
#         angle = angle + increment

#     if preview:
#         cam.stop_preview()

def take_picture(image_path):
    global cam
    # TODO move image distortion here
    cam.take_photo(image_path)

def deinit(preview=False):
    global cam
    if preview:
        cam.stop_preview()
    cam.close()
