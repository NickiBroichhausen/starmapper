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
RODOS_angular_velocity = 0
RODOS_yaw_target = 0

# Receivers
def starSensorCommandReceiver(data):
  global RODOS_ready_for_picture
  global RODOS_take_pictures
  global RODOS_pictures_done
  try:
    unpacked = struct.unpack("B", data)
    print("stm sends image  cmd: {}".format(unpacked[0]))
    if(unpacked[0]==1): #take next picture
      RODOS_ready_for_picture = True
    if(unpacked[0]==0): # map ready 
      RODOS_pictures_done = True 
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
  global control_mode_ai_pos
  try:
    unpacked = struct.unpack("BBB", data)
    print("stm sends mode: {} {} {}".format(unpacked[0],unpacked[1],unpacked[2]))
    if(unpacked[1]==3):
      control_mode_ai_pos = True
    else:
      control_mode_ai_pos = False
    if(unpacked[0]==2): # attitude is used
      RODOS_get_attitude = True
    if(unpacked[0]!=2):
      RODOS_get_attitude = False
    if(unpacked[2]==2):  #start to create star map
      RODOS_take_pictures = True
      RODOS_pictures_done = False
    if(unpacked[2]!=2): #aborT:
      RODOS_take_pictures = False
      RODOS_ready_for_picture = False
      RODOS_pictures_done = False 
  except Exception as e:
    print(e)
    print(data)
    print(len(data))

def pos_receiver(data):
  global RODOS_pos
  global RODOS_angular_velocity
  try:
    unpacked = struct.unpack("ffff", data)
#    print("stm sends pos data: {} {} {}".format(unpacked[0],unpacked[1],unpacked[2]))
#    print(time.time()) 
    RODOS_pos = unpacked[2]
    RODOS_angular_velocity = unpacked[3]
  except Exception as e:
    print(e)
    print(data)
    print(len(data))

def target_receiver(data):
  global RODOS_yaw_target
  try:
    unpacked = struct.unpack("ff", data)
    RODOS_yaw_target = unpacked[0]
    print(f"recieved target: {RODOS_yaw_target}") 
  except Exception as e:
    print(e)
    print(data)
    print(len(data))


ras2stmCommands = rodos.Topic(1121)
stm2rasCommands = rodos.Topic(1120)
ras2stmAttitiude = rodos.Topic(1123)
ras2stmControlValue = rodos.Topic(1124)
stm2rasSetFolder = rodos.Topic(1122)
stm2rasMode = rodos.Topic(1014) #1011
stm2rasPos = rodos.Topic(1011)
stm2rasTarget = rodos.Topic(1017)


luart = rodos.LinkinterfaceUART(path="/dev/serial0")
gwUart = rodos.Gateway(luart)
gwUart.run()

#receiver
stm2rasCommands.addSubscriber(starSensorCommandReceiver)
stm2rasMode.addSubscriber(satellite_mode_receiver)
stm2rasPos.addSubscriber(pos_receiver)
stm2rasTarget.addSubscriber(target_receiver)
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


from FloatSat_AI_Controller import AI_Controller as AI_Controller

folder = "1"
visualisation = False

while True:  
  try:
    if(RODOS_take_pictures):
      print("start mapping")
      # check if folder exist
      if not os.path.exists(folder):
        os.makedirs(folder)
      
      camera.init()
      threads = []
      while True:
        if((RODOS_ready_for_picture)):
          #Take picture
          print("taking picture") 
          time.sleep(1)
          image_path = os.path.join(folder, f"image_{int(RODOS_pos)%360}.jpg")
          camera.take_picture(image_path)
          time.sleep(1)
          t = threading.Thread(target=extractDescpriptors.analyse, args=(image_path, folder, visualisation))
          t.start()
          threads.append(t)

          #command "picture made"
          sensor_struct = struct.pack("B3x",1)
          ras2stmCommands.publish(sensor_struct)
          RODOS_ready_for_picture = False

        if(RODOS_pictures_done):
          #stitch map
          print("waiting for threads to finish")
          for t in threads:
            t.join()
          print("stitching map") 
          createStarMap.createStarMap(folder, visualisation)
          #star mapping done
          sensor_struct = struct.pack("B3x",0)
          ras2stmCommands.publish(sensor_struct)

          RODOS_pictures_done = False
          break

        time.sleep(1)

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
        sensor_struct = struct.pack("f",attidude)
        ras2stmAttitiude.publish(sensor_struct)
        os.delete(image_path)

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

    if(control_mode_ai_pos):
      yaw = RODOS_pos
      angular_velocity = -RODOS_angular_velocity
      #angular_velocity /= 100 
      target = RODOS_yaw_target
      control = AI_Controller.get_control(yaw, target, angular_velocity)
      data_struct = struct.pack("f",control) #value for torque
      ras2stmControlValue.publish(data_struct)
      print(f"ai recieved: {yaw}, {target}, {angular_velocity}")
      print(f"ai sent: {control}")
    time.sleep(0.1)

  except Exception as e:
    print(e)
    print(traceback.format_exc())
    camera.deinit()


