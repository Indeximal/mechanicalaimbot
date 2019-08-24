import serial
import time 
import struct


port = 'COM3'

ard = serial.Serial(port,9600)
time.sleep(2)

step1 = 30
step2 = 10


if step1<0:
	step1 = abs(step1)+127
if step2<0:
	step2 = abs(step2)+127

byte = True
ard.write(struct.pack('BB',int(step1), int(step2)))


time.sleep(1)
while byte:

	if ard.inWaiting():
		print(str(int.from_bytes(ard.read(),"big"))+"  "+str(int.from_bytes(ard.read(),"big")))

		print(str(struct.pack('BB', int(step1), int(step2))))
		
		byte = False
	else:
		print("not received yet")