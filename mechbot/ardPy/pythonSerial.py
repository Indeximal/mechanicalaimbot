import serial
import time 
import struct


port = 'COM3'

ard = serial.Serial(port,9600)
time.sleep(2)

step1 = 0
step2 = 0

step1 = step1 + 127
step2 = step2 + 127 

byte = True
ard.write(struct.pack('BBBB', 1,int(step1), int(step2), 0))

#time.sleep(1)

while byte:

	if ard.inWaiting():
		#print(int.from_bytes(ard.read(),"big"))
		print(int.from_bytes(ard.read(),"big") == 1)
		byte = False
	else:
		print("not received yet")