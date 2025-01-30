#!/bin/python3

import time
import struct

import rodosmwinterface as rodos
rodos.printTopicInit(enable=True)

ready_for_picture = False
control_mode_ai_vel = False
control_mode_ai_pos = False
mission_star_mapper = False

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
      mission_star_mapper = True
    else:
      mission_star_mapper = False
  except Exception as e:
    print(e)
    print(data)
    print(len(data))

def pos_receiver(data):
  try:
    unpacked = struct.unpack("ffff", data)
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


#receiver
stm2rasCommands.addSubscriber(starSensorCommandReceiver)
stm2rasMode.addSubscriber(satellite_mode_receiver)
stm2rasPos.addSubscriber(pos_receiver)
#sender
gwUart.forwardTopic(ras2stmCommands)
gwUart.forwardTopic(ras2stmControlValue)
gwUart.run()
sensor_index = 0

while True:
  # Dummy sensor data

  #if(mission_star_mapper):
    # Pack sensor data to a struct that RODOS recognizes
  #  if(ready_for_picture):
      #take picture then:
  sensor_struct = struct.pack("B",1)
  ras2stmCommands.publish(sensor_struct)#command "picture made"
  ready_for_picture = False

  #if(control_mode_ai_vel | control_mode_ai_pos):
  data_struct = struct.pack("f",0.123456789) #test value
  ras2stmControlValue.publish(data_struct)

  print("sent") 
  time.sleep(1)
