from xhelp import *
from threading import Thread, Event
from copy import deepcopy
from time import sleep, time
import random

class XBee(Thread):
	nodes = []
	pings = []
	outmsgs = []
	success = []
	addr = 0
	tick = 0
	frameid = 0
	rxbuffer = bytearray()
	stop = Event()
	
	# States
	UNINT, REALZ, NORM = range(0,3)
	state = UNINT

	def __init__(self, serial):
		Thread.__init__(self)
		serial.setRTS(True)
		self.serial = serial
		self.start()

	def run(self):
		self.state = self.REALZ
		self.flushrx()
		self.MY()
		while not self.stop.is_set():
			sleep(0.005) # 1ms
			self.Rx()
			if (self.tick % 18) == 0: #100ms
				self.auditmsgs()
				self.auditpings()
			if self.state == self.NORM:
				if (self.tick % 72) == 0: #400ms
					self.PingNodes()
				if (self.tick % 182) == 0: #1s
					self.PingNodes()
					self.BXRSS()
				if (self.tick % 270) == 0: #1.5s
					self.NeighborRSSResponse()
					# self.ackreport()
			if ((self.tick) % 270) and (self.state == self.UNINT):
				self.MY()
			if (self.tick % 14450) == 0: #9s
				self.tick = 0
			self.tick = self.tick+1
		self.serial.close()

	def Rx(self):	
		for i in range(self.serial.inWaiting()):
			self.rxbuffer += self.serial.read()
		if len(self.rxbuffer):
			self.parseRx()

	def id(self):
		self.frameid = self.frameid + 1
		if (self.frameid == 0x7D) or (self.frameid == 0x7E):
			self.frameid = 0x7F
		elif (self.frameid == 0x11) or (self.frameid == 0x13) or (self.frameid == 0x0a) or (self.frameid == 0x0d):
			self.frameid = self.frameid + 1
		if self.frameid > 255:
			self.frameid = 0
		return self.frameid

	def msgsucceed(self, msg, stat):
		# if stat == 1:
		# 	message = "msg     fail: adr:{:02x} t:{:02.2f} id:{:02x} r:{}".format(msg['addr'], time() - msg['sent'], msg['id'], msg['retries'])
		# if stat == 0:
		# 	message = "msg  success: adr:{:02x} t:{:02.2f} id:{:02x} r:{}".format(msg['addr'], time() - msg['sent'], msg['id'], msg['retries'])
		self.success.append(stat)
		if len(self.success) > 20: 
			self.success.remove(self.success[0])

	def pingsucceed(self, packet, stat):
		# [print("{:02x}:{:.0f} ".format(ping['id'], time() - ping['sent']), end="") for ping i n self.pings]
		# print("")
		for node in self.nodes:
			# print(node['failures'])
			if node['addr'] == packet['addr']:
				node['failures'].append(stat)
				# if stat == 1:
				# 	print("ping    fail: adr:{:02x} t:{:02.2f} id:{:02x}".format(node['addr'], time() - packet['sent'], packet['id']))
				# if stat == 0:
				# 	print("ping success: adr:{:02x} t:{:02.2f} id:{:02x}".format(node['addr'], time() - packet['sent'], packet['id']))
				if len(node['failures']) > 25:
					node['failures'].remove(node['failures'][0])
				break

	def ackreport(self):
		if len(self.success):
			# print(self.success)
			print("success rate:{}/{}:{}".format(sum(self.success), len(self.success) , 100-((sum(self.success) / len(self.success))*100)))

	def GetNodes(self):
		return list(self.nodes)

	def shutdown(self):
		self.stop.set()
		self.join()

	def parse(self, message):
		# print(hexformat(message))
		rxtype = message[0]
		if rxtype == 0x81: # received message
			addr = (message[1]<<8) + message[2]
			rssi = message[3]
			options = message[4]
			msgid = message[5]
			appid = message[6]
			# print(hexformat(message))

			if appid == 0x10:
				self.updatenodeinfo(addr, rssi)
			
			elif appid == 0x12:
				nrssi = message[7]
				self.updatenodeinfo(addr, rssi, nrssi)
			
			elif appid == 0x22:
				srcaddr = (message[7]<<8) + message[8]
				srcrssi = message[9]
				neighadd = (message[10]<<8) + message[11]
				neighrssi = message[12]
				self.updatenodeneighborinfo(srcaddr, srcrssi, neighadd, neighrssi)
				self.ACK(addr, msgid)
			
			elif appid == 0x26:
				pid = message[7]
				nodeaddr = (message[8]<<8) + message[9]
				self.markping(pid)
				self.ACK(addr, msgid)
			
			elif appid == 0x27:
				self.markmsg(msgid)
		
		elif rxtype == 0x89: # transmit status
			frameid = message[1]
			status  = message[2]
			# if status == 0:
			# 	self.markmsg(frameid)
			if (status & 3):
				self.retrytmsg(frameid)
		
		elif rxtype == 0x88: # AT command response
			frameid = message[1]
			cmd = ((message[2]<<8) + message[3])
			status = message[4]
			if cmd == ((0x4D<<8) + 0x59):
				print("initialized")
				self.state = self.NORM
				self.addr = ((message[5]<<8) + message[6])

	def updatenodeinfo(self, addr, rssi, nrssi=0x00):
		found = False
		for node in self.nodes:
			if node['addr'] == addr:
				node['rssi'] = rssi
				if nrssi is not 0x00:
					node['nrssi'] = nrssi
				node['time'] = time()
				found = True
				break
		if not found:
			new = {'addr':addr ,'rssi':rssi ,'nrssi':nrssi, 'time':time(), 'neighbors':[], 'sends':0, 'failures':[0]}
			self.nodes.append(new)

	def updatenodeneighborinfo(self, saddr, srssi, naddr, nrssi):
		found = False
		for node in self.nodes:
			if node['addr'] == saddr:
				found = True
				nfound = False
				for neighbor in node['neighbors']:
					if neighbor['addr'] == naddr:
						neighbor['rssi'] = srssi
						neighbor['nrssi'] = nrssi
						neighbor['time'] = time()
						nfound = True
						break
				if nfound:
					break
				elif not nfound: 
					newneigb =  {'addr':naddr, 'rssi':srssi, 'nrssi':nrssi, 'time':time()}
					node['neighbors'].append(newneigb)
				break
		if not found:
			new = {'addr':saddr, 'rssi':0x00, 'nrssi':0x00, 'time':time(), 'neighbors':[], 'sends':0, 'failures':[0]} # new node
			newneigb = {'addr':naddr, 'rssi':srssi, 'nrssi':nrssi, 'time':time()}
			new['neighbors'].append(newneigb)
			self.nodes.append(new)

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
					continue
				if checksum(self.rxbuffer[start:end]) == self.rxbuffer[end]:
					msg = unescape(self.rxbuffer[start:end])
					# print("in: " + hexformat(self.rxbuffer[start-3:end+1]))
					self.parse(msg)
				oldend = end+1
			elif not found:
				oldend = place + 1
		if oldend > 0:
			self.rxbuffer = self.rxbuffer[oldend:]

	def markping(self, iden):
		for packet in self.pings:
			if packet['id'] == iden:
				packet['received'] = True
				self.pingsucceed(packet, 0)
				self.pings.remove(packet)
				break

	def auditpings(self):
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

	def markmsg(self, frameid):
		# print("ack:" + str(frameid))
		for msg in self.outmsgs:
			if msg['id'] == frameid:
				# print("ackfound")
				self.msgsucceed(msg, 0)
				self.outmsgs.remove(msg)
				break

	def auditmsgs(self):
		deletelist = []
		for msg in self.outmsgs:
			# print(hexformat(msg['msg']))
			if (time() - msg['sent']) > 0.2:
				self.serial.write(escape(msg['msg']))
				msg['retries'] = msg['retries'] + 1
			if ((time() - msg['sent']) > 1.0) or (msg['retries'] > 3):
				deletelist.append(msg)
				self.msgsucceed(msg, 1)
		for msg in deletelist:
			self.outmsgs.remove(msg)

	def retrytmsg(self, frameid):
		for msg in self.outmsgs:
			if msg['id'] == frameid:
				self.serial.write(escape(msg['msg']))
				msg['retries'] = msg['retries'] + 1
				if (msg['retries'] > 3):
					self.outmsgs.remove(msg)
					self.msgsucceed(msg, 1)
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

	def flushrx(self):
		sleep(0.100)
		for i in range(self.serial.inWaiting()):
			self.serial.read()

	def MY(self):
		message = self.serial.write(bytearray(b'\x7e\x00\x04\x08\x01\x4D\x59\x50'))

	def BXRSS(self):
		self.serial.write(bytearray(b'\x7E\x00\x07\x01\x00\xff\xff\x00\x00\x10\xf0'))

	def ACK(self, addr, fid):
		message = bytearray(b'\x7e\x00\x07\x01\x00\xee\xee\x00\xee\x27\x00')
		message[5] = (addr&0xFF00)>>8
		message[6] = addr&0xFF
		message[8] = fid # id (for ack)
		message[10] = checksum(message[3:])
		# print("out:" + hexformat(message))
		self.serial.write(message)

	def NeighborRSSResponse(self):
		for node in self.nodes:
			message = bytearray(b'\x7e\x00\x08\x01\x00\xee\xee\x00\x00\x12\xee\x00')
			message[5] = (node['addr']&0xFF00)>>8
			message[6] = node['addr']&0xFF
			message[10] = node['rssi']
			message[11] = checksum(message[3:])
			# print(hexformat(message))
			self.serial.write(escape(message))

	def PingNodes(self):
		for node in self.nodes:
			fid = self.id()
			addr = node['addr']
			node['sends'] = node['sends'] + 1
			message = bytearray(b'\x7e\x00\x0B\x01\xee\x00\x01\x00\xee\x24\xee\xee\xee\x00\x00')
			message[4] = fid # frame id
			message[8] = fid     # id (for ack)
			message[10] = fid    # ping id
			message[11] = (addr&0xFF00)>>8
			message[12] = addr&0xFF
			message[14] = checksum(message[3:])
			self.serial.write(escape(message))
			self.buffout(message, addr, fid, False)
			packetid = {
				'id': fid,
				'addr':addr,
				'sent':time(),
				'msg':message,
				'received':False
				}
			found = False
			for packet in self.pings:
				if packet['id'] == packetid['id']:
					self.pingsucceed(packet, 1)
					break
					# packetid['sent'] = time()
					# packet['sent'] = time()
					# packet = packetid
					# found = True
			if not found:
				self.pings.append(packetid)