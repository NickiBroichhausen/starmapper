#!/bin/python3

import time
import struct

#RODOS stuff
import rodosmwinterface as rodos
rodos.printTopicInit(enable=True)

RODOS_ready_for_picture = False
control_mode_ai_vel = False
control_mode_ai_pos = False
RODOS_take_pictures = False
RODOS_pictures_done = False
RODOS_get_attitude = False
RODOS_set_folder = False
RODOS_pos = 0

# Receivers
def starSensorCommandReceiver(data):
  global RODOS_ready_for_picture
  global RODOS_take_pictures
  try:
    unpacked = struct.unpack("B", data)
    print("stm sends cmd: {}".format(unpacked[0]))
    if(unpacked[0]==1): #take next picture
      RODOS_ready_for_picture = True
    if(unpacked[0]==0): # abort
      RODOS_take_pictures = False
  except Exception as e:
    print(e)
    print(data)
    print(len(data))

def setFolder(data):
  global RODOS_set_folder
  try:
    RODOS_set_folder = struct.unpack("B", data)[0]
    print("stm sends set folder: {}".format(RODOS_set_folder))
  except Exception as e:
    print(e)
    print(data)
    print(len(data))

def satellite_mode_receiver(data):
  global RODOS_take_pictures
  global RODOS_pictures_done
  global RODOS_get_attitude
  try:
    unpacked = struct.unpack("BBB", data)
    print("stm sends mode: {} {} {}".format(unpacked[0],unpacked[1],unpacked[2]))
    # if(unpacked[1]==3):
    #   control_mode_ai_vel = True
    # else:
    #   control_mode_ai_vel = False
    # if(unpacked[1]==4):
    #   control_mode_ai_pos = True
    # else:
    #   control_mode_ai_pos = False
    if(unpacked[0]==2): # attitude is used
      RODOS_get_attitude = True
    else:
      RODOS_get_attitude = False
    if(unpacked[2]==2):  #start to create star map
      RODOS_take_pictures = True
    else:
      RODOS_pictures_done = True
  except Exception as e:
    print(e)
    print(data)
    print(len(data))

def pos_receiver(data):
  global RODOS_pos
  try:
    unpacked = struct.unpack("ffff", data)
    print("stm sends pos data: {} {} {}".format(unpacked[0],unpacked[1],unpacked[2]))
    RODOS_pos = unpacked[2]
  except Exception as e:
    print(e)
    print(data)
    print(len(data))


ras2stmCommands = rodos.Topic(1021)
stm2rasCommands = rodos.Topic(1020)
ras2stmAttitiude = rodos.Topic(1023)
ras2stmControlValue = rodos.Topic(1024)
stm2rasSetFolder = rodos.Topic(1022)
stm2rasMode = rodos.Topic(1014) #1011
stm2rasPos = rodos.Topic(1011)


luart = rodos.LinkinterfaceUART(path="/dev/serial0")
gwUart = rodos.Gateway(luart)
gwUart.run()

#receiver
stm2rasCommands.addSubscriber(starSensorCommandReceiver)
stm2rasMode.addSubscriber(satellite_mode_receiver)
stm2rasPos.addSubscriber(pos_receiver)
stm2rasSetFolder.addSubscriber(setFolder)
#sender
gwUart.forwardTopic(ras2stmAttitiude)
gwUart.forwardTopic(ras2stmCommands)
gwUart.forwardTopic(ras2stmControlValue)


################### Starmap stuff###################
import camera
import AttitudeDetermination
import extractDescpriptors
import createStarMap
from time import sleep
import os
import threading
import traceback
# import StarSift as ss

folder = "1"
visualisation = False

while True:  
  try:
    if(RODOS_take_pictures):
      # check if folder exist
      if not os.path.exists(folder):
        os.makedirs(folder)
      
      camera.init()
      while True:
        if((RODOS_ready_for_picture)):
          #Take picture
          image_path = os.path.join(folder, f"image_{int(RODOS_pos)}.jpg")
          camera.take_picture(image_path)
          threading.Thread(target=extractDescpriptors.analyse, args=(image_path, folder, visualisation)).start()
          
          #command "picture made"
          sensor_struct = struct.pack("B",1)
          ras2stmCommands.publish(sensor_struct)
          RODOS_ready_for_picture = False

        if(RODOS_pictures_done):
          #stitch map
          createStarMap.createStarMap(folder, visualisation)
          #star mapping done
          sensor_struct = struct.pack("B",0)
          ras2stmCommands.publish(sensor_struct)

          RODOS_pictures_done = False
          break

      camera.deinit()
      RODOS_take_pictures = False

    if(RODOS_get_attitude):
      while True:
        image_path = "current.jpg"
        camera.init()
        camera.take_picture(image_path)
        camera.deinit()
        attidude = AttitudeDetermination.find_attitude(folder, 
                                                    image_path,
                                                    visualisation)
        #send attitude
        sensor_struct = struct.pack("f",attidude)  # TODO what to return here? float?
        ras2stmAttitiude.publish(sensor_struct)

        if(not RODOS_get_attitude):
          break
      RODOS_get_attitude = False

    if(RODOS_set_folder):
      #set folder
      folder = RODOS_set_folder
      # check if folder exist
      if not os.path.exists(folder):
          os.makedirs(folder)
      RODOS_set_folder = False

    # if(control_mode_ai_vel | control_mode_ai_pos):
    #   data_struct = struct.pack("f",0.123456789) #test value for torque
    #   ras2stmControlValue.publish(data_struct)

    time.sleep(1)

  except Exception as e:
    print(e)
    print(traceback.format_exc())
    camera.deinit()


