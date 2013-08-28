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
		self.addrreq()
		prev = time()
		while not self.stop.is_set():
			sleep(0.005) # 1ms
			self.Rx()
			if (self.tick % 18) == 0: #100ms
				self.auditmsgs()
				self.auditpings()
			if self.state == self.NORM:
				# if (self.tick % 62) == 0: #0.5s
				if (self.tick % 182) == 0: #1s
					self.BxNeighborCheck()
					self.PingNodes()
				if (self.tick % 270) == 0: #1.5s
					self.BxNeighborResponse()
					# self.ackreport()
			if ((self.tick) % 270) and (self.state == self.UNINT):
				self.addrreq()
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
		elif (self.frameid == 0x11) or (self.frameid == 0x13):
			self.frameid = self.frameid + 1
		if self.frameid > 255:
			self.frameid = 0
		return self.frameid

	def msgsucceed(self, msg, stat):
		if stat == 1:
			print("msg    fail: adr:{:02x} t:{:.4f} id:{:02x} {}".format(msg['addr'], time() - msg['sent'], msg['id'], hexformat(msg['msg'])))
		if stat == 0:
			print("msg success: adr:{:02x} t:{:.4f} id:{:02x} {}".format(msg['addr'], time() - msg['sent'], msg['id'], hexformat(msg['msg'])))
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
				if stat == 1:
					print("ping    fail: adr:{:02x} t:{:02.2f} id:{:02x}".format(node['addr'], time() - packet['sent'], packet['id']))
				if stat == 0:
					print("ping success: adr:{:02x} t:{:02.2f} id:{:02x}".format(node['addr'], time() - packet['sent'], packet['id']))
				if len(node['failures']) > 20:
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
			appid = message[5]
			if appid == 0x10:
				# print("brx")
				self.updatenodeinfo(addr, rssi)
			elif appid == 0x12:
				nrssi = message[6]
				# print("rsp")
				self.updatenodeinfo(addr, rssi, nrssi)
			elif appid == 0x22:
				srcaddr = (message[6]<<8) + message[7]
				srcrssi = message[8]
				neighadd = (message[9]<<8) + message[10]
				neighrssi = message[11]
				# print(hexformat(message))
				self.updatenodeneighborinfo(srcaddr, srcrssi, neighadd, neighrssi)
			elif appid == 0x24:
				if message[6] == 0x22 and message[7] == 0x22:
					self.gotten = self.gotten + 1
			elif appid == 0x26:
				pid = message[6]
				nodeaddr = (message[7]<<8) + message[8]
				# print(hexformat(message))
				self.markping(pid)
			# else:
				# print(message)
		elif rxtype == 0x89: # transmit status
			frameid = message[1]
			status  = message[2]
			if status == 0:
				self.markmsg(frameid)
			elif status | 3:
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
			expire = age > 20
			if expire:
				deletelist.append(packet)
				if not packet['received']:
					self.pingsucceed(packet, 1)
		for entry in deletelist:
			self.pings.remove(entry) 

	def markmsg(self, frameid):
		for msg in self.outmsgs:
			if msg['id'] == frameid:
				self.msgsucceed(msg, 0)
				self.outmsgs.remove(msg)
				break

	def retrytmsg(self, frameid):
		for msg in self.outmsgs:
			if msg['id'] == frameid:
				self.serial.write(escape(msg['msg']))
				msg['retries'] = msg['retries'] + 1
				if (msg['retries'] > 3):
					self.outmsgs.remove(msg)
					self.msgsucceed(msg, 1)
					print("too many retries time:{}".format(time() - msg['sent']))
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

	def buffout(self, message, addr, frameid, send=True):
		# print(hexformat(message))
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

	def PingNodes(self):
		for node in self.nodes:
			fid = self.id()
			packetid = {
				'id': self.id(),
				'addr':node['addr'],
				'sent':time(),
				'received':False
				}
			node['sends'] = node['sends'] + 1
			message = bytearray(b'\x7e\x00\x09\x01\xee\x00\x01\x00\x24\xee\xee\xee\x00')
			message[4] = fid
			message[9] = packetid['id']
			message[10] = (packetid['addr']&0xFF00)>>8
			message[11] = packetid['addr']&0xFF
			message[12] = checksum(message[3:])
			# print(hexformat(message))
			self.serial.write(escape(message))
			self.buffout(message, node['addr'], fid, False)
			for packet in self.pings:
				if packet['id'] == packetid['id']:
					packet = packetid
					# print('overwrite')
			self.pings.append(packetid)

	def addrreq(self):
		fid = self.id()
		message = bytearray(b'\x7e\x00\x04\x08\x00\x4D\x59\x00')
		message[4] = fid
		message[7] = checksum(message[3:])
		self.serial.write(escape(message))

	def BxNeighborResponse(self):
		for node in self.nodes:
			message = bytearray(b'\x7e\x00\x07\x01\xee\xee\xee\x00\x12\xee\x00')
			message[4] = self.id()
			message[5] = (node['addr']&0xFF00)>>8
			message[6] = node['addr']&0xFF
			message[9] = node['rssi']
			message[10] = checksum(message[3:])
			self.serial.write(escape(message))

	def BxNeighborCheck(self):
		self.serial.write(escape(bytearray(b'\x7E\x00\x06\x01\x00\xff\xff\x00\x10\xf0')))
