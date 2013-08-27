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

def checksum(buff):
	cksm = 0
	for byte in unescape(buff):
		cksm += byte
	return 0xFF - (cksm & 0xFF)