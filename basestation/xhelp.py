def hexformat(bytebuffer):
	return " ".join("{0:02x}".format(byte) for byte in bytebuffer)

def unescape(buff):
	nonescaped = bytearray()
	for i, byte in enumerate(buff):
		if byte == 0x7D and i <= len(buff):
			nonescaped.append(buff[i+1] ^ 0x20)
		else:
			nonescaped.append(byte)
	return nonescaped

def escape(buff):
	escaped = buff[0:3]
	for i, byte in enumerate(buff[3:len(buff)-1]):
		if (byte == 0x7D) or (byte == 0x7E) or (byte == 0x11) or (byte == 0x13):
			escaped.append(0x7D)
			escaped.append(byte ^ 0x20)
		else:
			escaped.append(byte)
	escaped.append(buff[len(buff)-1])
	# if buff != escaped:
		# print(hexformat(buff))
		# print(hexformat(escaped))
	return escaped

def checksum(buff):
	cksm = 0
	for byte in unescape(buff):
		cksm += byte
	return 0xFF - (cksm & 0xFF)