from Xhelp import *
import serial
from threading import Thread, Event
from dllist import LinkedList
from time import sleep, time

class XBee(Thread):
	nodes = LinkedList()
	pings = []
	outmsgs = []
	addr = 0
	tick = 0
	frameid = 0
	rxbuffer = bytearray()
	stop = Event()
	
	UNINT, REALZ, NORM = range(0,3) # States
	state = UNINT

	def __init__(self, serialport):
		Thread.__init__(self)
		self.serial = serial.Serial(port=serialport, baudrate=57600, timeout=0, rtscts=True)
		self.serial.setRTS(True)
		self.last = time()
		self.start()

	def shutdown(self):
		self.stop.set()
		self.join()

	def GetNodes(self):
		return list(self.nodes.elements())

	def id(self):
		self.frameid = self.frameid + 1
		if (self.frameid == 0x7D) or (self.frameid == 0x7E):
			self.frameid = 0x7F
		elif (self.frameid == 0x11) or (self.frameid == 0x13) or (self.frameid == 0x0a) or (self.frameid == 0x0d):
			self.frameid = self.frameid + 1
		if self.frameid > 255:
			self.frameid = 0
		return self.frameid

	def run(self):
		self.state = self.REALZ
		self.flushrx()
		self.MY()
		while not self.stop.is_set():
			sleep(0.005) # 1ms
			self.Rx()
			
			if (self.tick % 182) == 0: #100ms
				self.msgaudit()
				self.pingaudit()

			if self.state == self.NORM:
			
				if (self.tick % 46) == 0: #400ms
					self.PingNodes()
					# print(time() - self.last)
					# self.last = time()
			
				if (self.tick % 109) == 0: #.6s
					self.BXRSS()

				if (self.tick % 147) == 0: #.8s
					self.NeighborRSSResponse()

			if ((self.tick) % 182) and (self.state == self.UNINT):
				self.MY()

			if (self.tick % 182) == 0: #1s
				self.tick = 0

			self.tick = self.tick+1
		self.serial.close()

	def Rx(self):	
		for i in range(self.serial.inWaiting()):
			self.rxbuffer += self.serial.read()
		if len(self.rxbuffer):
			self.parseRx()

	def parse(self, message):
		rxtype = message[0]
		if rxtype == 0x81: # received message
			addr = (message[1]<<8) + message[2]
			rssi = message[3]
			# options = message[4]
			msgid = message[5]
			appid = message[6]

			if appid == 0x10:
				self.updatenodeinfo(addr, rssi)
			
			elif appid == 0x12:
				nrssi = message[7]
				self.updatenodeinfo(addr, rssi, nrssi)
			
			elif appid == 0x22:
				naddr = (message[7]<<8) + message[8]
				nrssi = message[9]
				neaddr = (message[10]<<8) + message[11]
				nerssi = message[12]
				self.updatenodeneighborinfo(naddr, nrssi, neaddr, nerssi)
				self.ACK(addr, msgid)
			
			elif appid == 0x26:
				pid = message[7]
				# nodeaddr = (message[8]<<8) + message[9]
				self.pingmark(pid)
				self.ACK(addr, msgid)
			
			elif appid == 0x27:
				self.msgmark(msgid)
		
		elif rxtype == 0x89: # transmit status
			frameid = message[1]
			status  = message[2]
			# if status == 0:
			# 	self.msgmark(frameid)
			if (status & 3):
				self.msgretry(frameid)
		
		elif rxtype == 0x88: # AT command response
			frameid = message[1]
			cmd = ((message[2]<<8) + message[3])
			status = message[4]
			if cmd == ((0x4D<<8) + 0x59):
				print("initialized")
				self.state = self.NORM
				self.addr = ((message[5]<<8) + message[6])

	def getnode(self, addr):
		for node in self.nodes.elements():
			if node['addr'] == addr:
				return node
		return None

	def getneighbor(self, node, neaddr):
		for neighbor in node['neighbors']:
			if neighbor['addr'] == neaddr:
				return neighbor
		return None

	def AddNode(self, addr, rssi, nrssi=0x00):
		node = {'addr':addr ,'rssi':rssi ,'nrssi':nrssi, 'time':time(), 'neighbors':[], 'pingsuccess':[0], 'prev':None, 'next':None, 'deployed':False}
		self.nodes.append(node)
		return node

	def addneighbor(self, node, neaddr, nrssi, nerssi):
		neighbor =  {'addr':neaddr, 'rssi':nrssi, 'nrssi':nerssi, 'time':time()}
		node['neighbors'].append(neighbor)
		return neighbor

	def updatenodeinfo(self, naddr, rssi, nrssi=0x00):
		node = self.getnode(naddr)
		if node is None:
			node = self.AddNode(naddr, rssi, nrssi)
		else:
			node['rssi'] = rssi
			node['time'] = time()
			if nrssi != 0x00:
				node['nrssi'] = nrssi

	def updatenodeneighborinfo(self, naddr, nrssi, neaddr, nerssi):
		node = self.getnode(naddr)
		if node is None:
			node = self.AddNode(naddr, 0x00, 0x00)

		neighbor = self.getneighbor(node, neaddr)
		if neighbor is None:
			neighbor = self.addneighbor(node, neaddr, nrssi, neaddr)
		else:
			neighbor['rssi'] = nrssi
			neighbor['nrssi'] = nerssi
			neighbor['time'] = time()

	def pingmark(self, iden):
		for packet in self.pings:
			if packet['id'] == iden:
				packet['received'] = True
				self.pingsucceed(packet, 0)
				self.pings.remove(packet)
				break

	def pingaudit(self):
		deletelist = []
		for packet in self.pings: 
			age = time() - packet['sent']
			expire = age > 2
			if expire:
				deletelist.append(packet)
				if not packet['received']:
					self.pingsucceed(packet, 1)
		for entry in deletelist:
			self.pings.remove(entry) 

	def pingsucceed(self, packet, stat):
		node = self.getnode(packet['addr'])
		if node is not None:
			node['pingsuccess'].append(stat)
			# if stat == 1:
			# 	print("ping    fail: adr:{:02x} t:{:02.2f} id:{:02x}".format(node['addr'], time() - packet['sent'], packet['id']))
			# if stat == 0:
			# 	print("ping success: adr:{:02x} t:{:02.2f} id:{:02x}".format(node['addr'], time() - packet['sent'], packet['id']))
			if len(node['pingsuccess']) > 25:
				node['pingsuccess'].remove(node['pingsuccess'][0])

	# def msgsucceed(self, msg, stat):
		# if stat == 1:
		# 	message = "msg     fail: adr:{:02x} t:{:02.2f} id:{:02x} r:{}".format(msg['addr'], time() - msg['sent'], msg['id'], msg['retries'])
		# if stat == 0:
		# 	message = "msg  success: adr:{:02x} t:{:02.2f} id:{:02x} r:{}".format(msg['addr'], time() - msg['sent'], msg['id'], msg['retries'])

	def msgmark(self, frameid):
		# print("ack:" + str(frameid))
		for msg in self.outmsgs:
			if msg['id'] == frameid:
				# self.msgsucceed(msg, 0)
				self.outmsgs.remove(msg)
				break

	def msgaudit(self):
		deletelist = []
		for msg in self.outmsgs:
			if (time() - msg['sent']) > 0.1:
				self.serial.write(escape(msg['msg']))
				msg['retries'] = msg['retries'] + 1
			if ((time() - msg['sent']) > 1.0) or (msg['retries'] > 3):
				deletelist.append(msg)
				# self.msgsucceed(msg, 1)
		for msg in deletelist:
			self.outmsgs.remove(msg)

	def msgretry(self, frameid):
		for msg in self.outmsgs:
			if msg['id'] == frameid:
				self.serial.write(escape(msg['msg']))
				msg['retries'] = msg['retries'] + 1
				if (msg['retries'] > 3):
					self.outmsgs.remove(msg)
					# self.msgsucceed(msg, 1)
					# print("too many retries time:{}".format(time() - msg['sent']))
				break

	def buffout(self, message, addr, frameid, send=True):
		new = {
		'msg':message,
		'addr':addr,
		'id':frameid,
		'sent':time(),
		'retries':0
		}
		self.outmsgs.append(new)
		if send:
			self.serial.write(escape(msg['msg']))

	def inject(self, buff):
		self.rxbuffer += buff

	def flushrx(self):
		sleep(0.100)
		for i in range(self.serial.inWaiting()):
			self.serial.read()

	def MY(self):
		self.serial.write(bytearray(b'\x7e\x00\x04\x08\x01\x4D\x59\x50'))

	def BXRSS(self):
		self.serial.write(bytearray(b'\x7E\x00\x07\x01\x00\xff\xff\x00\x00\x10\xf0'))

	def ACK(self, addr, fid):
		message = bytearray(b'\x7e\x00\x07\x01\x00\xee\xee\x00\xee\x27\x00')
		message[5] = (addr&0xFF00)>>8
		message[6] = addr&0xFF
		message[8] = fid # id (for ack)
		message[10] = checksum(message[3:])
		self.serial.write(message)

	def NeighborRSSResponse(self):
		for node in self.nodes.elements():
			message = bytearray(b'\x7e\x00\x08\x01\x00\xee\xee\x00\x00\x12\xee\x00')
			message[5] = (node['addr']&0xFF00)>>8
			message[6] = node['addr']&0xFF
			message[10] = node['rssi']
			message[11] = checksum(message[3:])
			self.serial.write(escape(message))

	def PingNodes(self):
		for node in self.nodes.elements():
			fid = self.id()
			addr = node['addr']
			message = bytearray(b'\x7e\x00\x0B\x01\xee\x00\x01\x00\xee\x24\xee\xee\xee\x00\x00')
			message[4] = fid # frame id
			message[8] = fid     # id (for ack)
			message[10] = fid    # ping id
			message[11] = (addr&0xFF00)>>8
			message[12] = addr&0xFF
			message[14] = checksum(message[3:])
			self.AddPing(message, addr, fid)

	def AddPing(self, message, addr, fid):
		ping = {
			'id': fid,
			'addr':addr,
			'sent':time(),
			'msg':message,
			'received':False
			}
		for op in self.pings:
			if op['id'] == ping['id']:
				self.pingsucceed(op, 1)
				break
		self.pings.append(ping)
		self.buffout(message, addr, fid, False)

	def parseRx(self):
		oldend = 0
		found = False
		for place, byte in enumerate(self.rxbuffer):
			if byte == 0x7E:
				found = True
				oldend = place
				if len(self.rxbuffer[place:]) < 3:
					break
				msb = self.rxbuffer[place+1]
				lsb = self.rxbuffer[place+2]
				if lsb < msb:
					oldend = lsb
					continue
				start = place + 3 + msb
				end = start + lsb
				if len(self.rxbuffer) <= end:
					break
				end = end + escapedchars(self.rxbuffer[start:end])
				if len(self.rxbuffer) <= end:	
					break
				msg = unescape(self.rxbuffer[start:end])
				if checksum(msg) == self.rxbuffer[end]:
					self.parse(msg)
				oldend = end+1
			elif not found:
				oldend = place + 1
		if oldend > 0:
			self.rxbuffer = self.rxbuffer[oldend:]