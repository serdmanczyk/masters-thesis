import serial
from time import sleep, time

def hexformat(bytebuffer):
	return " ".join("{0:02x}".format(byte) for byte in bytebuffer)

def unescape(buff):
	nonescaped = bytearray()
	for i, byte in enumerate(buff):
		if byte is 0x7D:
			nonescaped.append(buff[i+1] ^ 0x20)
		else:
			nonescaped.append(byte)
	return nonescaped

def checksum(buff):
	cksm = 0
	for byte in unescape(buff):
		cksm += byte
	return 0xFF - (cksm & 0xFF)

def apiread(apitype="MY"):
	command = 0x0000
	if apitype is "WR":
		command = 0x5752
	elif apitype is "MY":
		command = 0x4D59
	elif apitype is "RR":
		command = 0x5252
	elif apitype is "RN":
		command = 0x524E
	elif apitype is "BD":
		command = 0x4244
	elif apitype is "CA":
		command = 0x4341
	elif apitype is "AP":
		command = 0x4150
	elif apitype is "D6":
		command = 0x4436
	elif apitype is "D7":
		command = 0x4437
	return readcmd(command)

def apiwrite(apitype="MY", parameters=0x00):
	command = 0x0000
	if apitype is "WR":
		command = 0x5752
	elif apitype is "MY":
		command = 0x4D59
	elif apitype is "RR":
		command = 0x5252
	elif apitype is "RN":
		command = 0x524E
	elif apitype is "BD":
		command = 0x4244
	elif apitype is "CA":
		command = 0x4341
	elif apitype is "AP":
		command = 0x4150
	elif apitype is "D6":
		command = 0x4436
	elif apitype is "D7":
		command = 0x4437
	return writecmd(command, parameters)

def readcmd(command=0x4D59):
	message = bytearray()
	message.append(0x7e)			 # start
	message.append(0x00)			 # msb
	message.append(0x04)			 # lsb
	message.append(0x08)			 # API
	message.append(0x01)			 # frame id
	message.append((command&0xFF00)>>8)
	message.append(command&0xFF)		 
	message.append(checksum(message[3:]))
	return message

def writecmd(command=0x4D59, parameters=0x00):
	message = bytearray()
	message.append(0x7e)			 # start
	message.append(0x00)			 # msb
	message.append(0x06)			 # lsb
	message.append(0x08)			 # API
	message.append(0x01)			 # frame id
	message.append((command&0xFF00)>>8)   
	message.append(command&0xFF)		 
	message.append((parameters&0xFF00)>>8)   
	message.append(parameters&0xFF)		 
	message.append(checksum(message[3:]))
	return message


def waiting(serial):
	return serial.inWaiting()

def rserial(serial):
	message = bytearray()
	while serial.inWaiting():
		message += serial.read()
	return message

def read(serial):
	# sleep(0.05)
	while not serial.inWaiting():
		pass
	while serial.inWaiting():
		print(hexformat(rserial(serial)))



if __name__ == "__main__":
	# Sx = serial.Serial(port="/dev/tty.usbserial-A1014JWM", baudrate=9600, timeout=0)
	Sx = serial.Serial(port="COM3", baudrate=57600, timeout=0, rtscts=True)

	# Sx.setRTS(True)
	# print(hexformat(apiread("D6")))
	# print(hexformat(apiread("D6", 0x01)))

	# Sx.write(apiwrite("RR", 0x00))
	# Sx.write(apiwrite("RN", 0x00))
	# Sx.write(apiwrite("CA", 0x24))
	# Sx.write(apiwrite("BD", 0x06))
	# Sx.write(apiwrite("WR"))
	# read(Sx)

	# RR (0x52 0x52)
	# 	0x00 - 0 XBee level retries after three CSMA-CA level attemps
	# 	\x7e\x00\x05\x08\x01\x52\x52\x00\x00\x

	# RN (0x52 0x4E)
	# 	0x00 - disable backoff in first iteration of CSMA-CA
	# 	\x7e\x00\x05\x08\x01\x52\x4E\x00\x00\x

	# CA (0x43 0x41)
	# 	0x24 - CCA threshold -36dbm
	# 	\x7e\x00\x05\x08\x01\x43\x41\x00\x24\x

	# BD (0x42 0x44)
	# 	0x06 - baud rate 57600
	# 	\x7e\x00\x05\x08\x01\x52\x44\x00\x07\x