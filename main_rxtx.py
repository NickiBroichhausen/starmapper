#!/bin/python3

import time
import struct

#RODOS stuff
import rodosmwinterface as rodos
rodos.printTopicInit(enable=True)

ready_for_picture = False
control_mode_ai_vel = False
control_mode_ai_pos = False
RODOS_take_pictures = False
RODOS_pictures_done = False
RODOS_get_attitude = False

# Receivers
def starSensorCommandReceiver(data):
  try:
    unpacked = struct.unpack("B", data)
    print("stm sends data: {}".format(unpacked[0]))
    if(unpacked[0]):
      ready_for_picture = True
  except Exception as e:
    print(e)
    print(data)
    print(len(data))

def satellite_mode_receiver(data):
  try:
    unpacked = struct.unpack("BBB", data)
    print("stm sends data: {} {} {}".format(unpacked[0],unpacked[1],unpacked[2]))
    if(unpacked[1]==3):
      control_mode_ai_vel = True
    else:
      control_mode_ai_vel = False
    if(unpacked[1]==4):
      control_mode_ai_pos = True
    else:
      control_mode_ai_pos = False
    if(unpacked[2]==2):
      RODOS_take_pictures = True
    else:
      RODOS_take_pictures = False
    #how can i change here to next step get atitude
  except Exception as e:
    print(e)
    print(data)
    print(len(data))

def pos_receiver(data):
  try:
    unpacked = struct.unpack("fffB", data)
    print("stm sends pos data: {} {} {}".format(unpacked[0],unpacked[1],unpacked[2]))
  except Exception as e:
    print(e)
    print(data)
    print(len(data))


ras2stmCommands = rodos.Topic(1021)
stm2rasCommands = rodos.Topic(1020)
ras2stmControlValue = rodos.Topic(1024)
stm2rasMode = rodos.Topic(1014) #1011
stm2rasPos = rodos.Topic(1011)


luart = rodos.LinkinterfaceUART(path="/dev/serial0")
gwUart = rodos.Gateway(luart)
gwUart.run()

#receiver
stm2rasCommands.addSubscriber(starSensorCommandReceiver)
stm2rasMode.addSubscriber(satellite_mode_receiver)
stm2rasPos.addSubscriber(pos_receiver)
#sender
gwUart.forwardTopic(ras2stmCommands)
gwUart.forwardTopic(ras2stmControlValue)

# sensor_index = 0



################### Starmap stuff###################
import camera
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

while True:
  # Dummy sensor data

  if(RODOS_take_pictures):
    # check if folder exist
    if not os.path.exists(folder):
      os.makedirs(folder)
    
    camera.init()
    angle = 0
    while True:  # TODO when break??
      if(ready_for_picture):
        #Take picture
        image_path = os.path.join(folder, f"image_{angle}.jpg")
        camera.take_picture(image_path)
        threading.Thread(target=extractDescpriptors.analyse, args=(image_path, folder, visualisation)).start()
        angle = angle + 1
        
        #command "picture made"
        sensor_struct = struct.pack("B",1)
        stm2rasCommands.publish(sensor_struct)
        ready_for_picture = False

      if(RODOS_pictures_done):
        #stitch map
        createStarMap.createStarMap(folder, visualisation)
        RODOS_pictures_done = False
        break

    camera.deinit()
    RODOS_take_pictures = False

  if(RODOS_get_attitude):
    image_path = "current.jpg"
    camera.init()
    camera.take_picture(image_path)
    camera.deinit()
    attidude = AttitudeDetermination.find_attitude(folder, 
                                                image_path,
                                                visualisation)
    #send attitude
    sensor_struct = struct.pack("f",attidude)  # TODO what to return here? float?
    stm2rasCommands.publish(sensor_struct)
    RODOS_get_attitude = False


  if(control_mode_ai_vel | control_mode_ai_pos):
    data_struct = struct.pack("f",0.123456789) #test value
    ras2stmControlValue.publish(data_struct)


  time.sleep(1)
