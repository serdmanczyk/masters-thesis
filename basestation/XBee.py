from Xhelp import *
import serial
from threading import Thread, Event
from dllist import LinkedList
from time import sleep, time

class XBee(Thread):
	nodes = []
	pings = []
	outmsgs = []
	pingsuccess = []
	addr = 0
	tick = 1
	frameid = 0
	rxbuffer = bytearray()
	stop = Event()
	currdeploy = None

	startup, listen, processing = range(0,3) # States

	def __init__(self, serialport):
		Thread.__init__(self)
		self.serial = serial.Serial(port=serialport, baudrate=57600, timeout=0, rtscts=True)
		self.serial.setRTS(True)
		self.last = time()
		self.starttime = time()
		self.ready = False
		self.start()

	def shutdown(self):
		self.stop.set()
		self.join()

	def Ready(self):
		return self.ready

	def OutDebug(self):
		now = time()
		print("success rate:", 100-(100*(sum(self.pingsuccess) / len(self.pingsuccess))))
		for node in self.nodes:
			if node['deployed']:
				print("Node:{} RSSI:{} NRSSI:{} age:{}".format(node['addr'], node['rssi'], node['nrssi'], 'y' if (now-node['time']) > 1.5 else 'n'))
				print("\tFront:{}  RSSI:{} NRSSI:{} age:{}".format(node['faddr'], node['frssi'], node['fnrssi'],  'y' if (now-node['nt']) > 3 else 'n'))
				print("\tRear:{}   RSSI:{} NRSSI:{} age:{}".format(node['raddr'], node['rrssi'], node['rnrssi'],  'y' if (now-node['nt']) > 3 else 'n'))
		print("time:{:.2f}\n".format(now-self.start))


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
		self.state = self.startup
		self.flushrx()
		self.MY()
		print("start")		
		while not self.stop.is_set():
			sleep(0.005) # 1ms
			self.Rx()
			
			if (((time() - self.starttime) > 10) and (self.state == self.listen)):
				print("stop listening, start deploying")
				self.state = self.processing

			if self.state == self.processing:
				if (self.tick % 18) == 0: #100ms
					self.msgaudit()
					self.pingaudit()

				if (self.tick % 364) == 0: #2s
					self.evalDeploy()
			
				if (self.tick % 91) == 0: #400ms
					self.PingNodes()
					# print(time() - self.last)
					# self.last = time()
			
				if (self.tick % 109) == 0: #.6s
					self.BXRSS()

				if (self.tick % 147) == 0: #.8se
					self.NeighborRSSResponse()

			if (((self.tick) % 182) == 0) and (self.state == self.startup):
				self.MY()

			if (self.tick % 364) == 0: #1s
				self.tick = 1

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
			naddr = (message[1]<<8) + message[2]
			rssi = message[3]
			# options = message[4]
			msgid = message[5]
			appid = message[6]

			self.updatenodeinfo(addr, rssi)

			if msgid != 0:
				self.ACK(naddr, msgid)
			
			elif appid == 0x12:
				nrssi = message[7]
				self.updatenodeinfo(naddr, rssi, nrssi)
			
			elif appid == 0x22:
				srcaddr = (message[7]<<8) + message[8]
				faddr = (message[9]<<8) + message[10]
				frssi = message[11]
				fnrssi = message[12]
				raddr = (message[13]<<8) + message[14]
				rrssi = message[15]
				rnrssi = message[16]
				self.updatenodeneighborinfo(naddr, faddr, frssi, fnrssi, raddr, rrssi, rnrssi)
			
			elif appid == 0x26:
				pid = message[7]
				# nodeaddr = (message[8]<<8) + message[9]
				self.pingmark(pid)
			
			elif appid == 0x27:
				self.DeployAck(msgid)
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
				print("listening")
				self.state = self.listen
				self.starttime = time()
				self.addr = ((message[5]<<8) + message[6])


	def getClosestDeployed(self):
		rnode = None
		for node in self.nodes():
			if not node['deployed']:
				break
			else:
				rnode = node
		return rnode

	def getNextnonDeployed(self):
		for node in self.nodes():
			if not node['deployed']:
				# print("non-deployed:{}".format(node['addr']))
				return node
		return None

	def CheckNodeThreshold(self, node):
		nrssi = node['rssi']
		nerssi = node['nrssi']
		if nrssi > nerssi:
			rss = nerssi
		else:
			rss = nrssi
		print("check threshold:{} rss:{}".format(node['addr'], rss))
		if rss > 70:
			return True
		return False

	def evalDeploy(self):
		if self.currdeploy is not None:
			self.doNextDeploymentStep()
			return
		node = self.getClosestNode()
		if node is not None:
			if self.CheckNodeThreshold(node):
				rear = self.getnode(node['raddr'])
				if rear is not None:
					self.Deploy(node['raddr'])
			return
		node = self.getFirstnonDeployed()
		if node is not None:
			self.Deploy(node['addr'])

	def Deploy(self, node):
		steplist = []

		print("deploy:{}".format(node['addr']))
		front = self.getnode(node['faddr'])
		if rear is not None:
			front['raddr'] = node['addr']
			front['rrssi'] = 0x00
			front['rnrssi'] = 0x00
			steplist.append('step':'assign', 'fid':self.id(), 'node':rear)

		steplist.append('step':'assign', 'fid':self.id(), 'node':node)
		steplist.append('step':'deploy', 'fid':self.id(), 'node':node)
		
		self.currdeploy = {'node':node, 'start':time(), steps':steplist'}
		self.doNextDeploymentStep()

	def doNextDeploymentStep(self):
		if self.currdeploy is None:return
		if (time() - self.currdeploy['start']) > 5:
			self.currdeploy = None
			return
		if len(self.currdeploy['steps']) == 0:
			self.currdeploy = None
			return
		nextstep = self.currdeploy['steps'][0]
		if nextstep['name'] == 'assign':
			self.AssignAddress(nextstep['fid'], nextstep['node'])
		if nextstep['name'] == 'deploy':
			self.DeployMsg(nextstep['fid'], nextstep['node'])

	def DeployAck(self, frameid):
		if self.currdeploy is None:return
		for step in self.currdeploy['steps']:
			if frameid == step['fid']:
				# print("success:{}".format(step['name']))
				self.currdeploy['steps'].remove(step)
		if len(self.currdeploy['steps']) != 0:
			self.doNextDeploymentStep()
		else:
			print("deployment successful")
			self.currdeploy['node']['deployed'] = True
			if self.getFirstnonDeployed() is None:
				self.ready = True
			self.currdeploy = None


	def getnode(self, addr):
		for node in self.nodes():
			if node['addr'] == addr:
				return node
		return None

	def AddNode(self, addr, rssi, nrssi=0x00):
		node = {'addr':addr ,'rssi':rssi ,'nrssi':nrssi, 'time':time(), 'deployed':False,
			'raddr':0x00, 'rrssi':0x00, 'rsnrssi':0x00, 
			'faddr':0x7E, 'frssi':0x00, 'frnrssi':0x00}
		print("add node:{}".format(node['addr']))
		if len(self.nodes)>0:
			node['faddr'] = self.nodes[-1]['addr']
		return node

	def updatenodeinfo(self, naddr, rssi, nrssi=0x00):
		node = self.getnode(naddr)
		if node is None:
			node = self.AddNode(naddr, rssi, nrssi)
		else:
			node['rssi'] = rssi
			node['time'] = time()
			if nrssi != 0x00:
				node['nrssi'] = nrssi

	def RemoveLost(self, rear, front):
		# It seems we've lost one or more nodes, we'll need
		# to purge our list to match our new node situation
		lostnodes = []
		print("remove lost:{}:{}".format(rear, front))

	def updatenodeneighborinfo(self, naddr, faddr, frssi, rnrssi, raddr, rrssi, rnrssi):
		node = self.getnode(naddr)
		if node is None:
			# we haven't deployed this node.  We should have heard a broadcast first
			# and added it, ignore this scenario.
			return

		if node['raddr'] != nraddr:
			print("something's wrong here")
			# we should reconnect our list, but we need to know the start.
			# Our nodes should be re-assembled now, so we'll wait for the 
			# rear neighbor to send its report and let the next 'if' re-build 
			# our list when that message arrives.
			return

		if node['faddr'] != nfaddr:
			self.RemoveLost(node['addr'], faddr)

		node['frssi'] = frssi
		node['fnrssi'] = fnrssi
		node['rrssi'] = rrssi
		node['rnrssi'] = rnrssi

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
			self.pingsuccess.append(stat)
			# if stat == 1:
			# 	print("ping    fail: adr:{:02x} t:{:02.2f} id:{:02x}".format(node['addr'], time() - packet['sent'], packet['id']))
			# if stat == 0:
			# 	print("ping success: adr:{:02x} t:{:02.2f} id:{:02x}".format(node['addr'], time() - packet['sent'], packet['id']))
			if len(self.pingsuccess) > 25:
				self.pingsuccess.remove(self.pingsuccess[0])

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
			self.serial.write(escape(new['msg']))

	# def inject(self, buff):
	# 	self.rxbuffer += buff

	def flushrx(self):
		sleep(0.200)
		for i in range(self.serial.inWaiting()):
			self.serial.read()

	def MY(self):
		self.serial.write(bytearray(b'\x7e\x00\x04\x08\x01\x4D\x59\x50'))

	def BXRSS(self):
		self.serial.write(bytearray(b'\x7e\x00\x07\x01\x00\xff\xff\x00\x00\x10\xf0'))

	def ACK(self, addr, fid):
		message = bytearray(b'\x7e\x00\x07\x01\x00\xee\xee\x00\xee\x27\x00')
		message[5] = (addr&0xFF00)>>8
		message[6] = addr&0xFF
		message[8] = fid # id (for ack)
		message[10] = checksum(message[3:])
		self.serial.write(message)

	def NeighborRSSResponse(self):
		first = self.getClosestDeployed()
		if first is not None:
			message = bytearray(b'\x7e\x00\x08\x01\x00\xee\xee\x00\x00\x12\xee\x00')
			message[5] = (first['addr']&0xFF00)>>8
			message[6] = first['addr']&0xFF
			message[10] = first['rssi']
			message[11] = checksum(message[3:])
			self.serial.write(escape(message))
			# print(hexformat(message))

	def PingNodes(self):
		if len(self.nodes) == 0:return
		first = self.getClosestDeployed()
		furthest = self.nodes.front()
		if furthest['deployed'] and closest['deployed']:
			fid = self.id()
			message = bytearray(b'\x7e\x00\x0B\x01\xee\xee\xee\x00\xee\x24\xee\xee\xee\x00\x00')
			message[4] = fid # frame id
			message[5] = (furthest['addr']&0xFF00)>>8
			message[6] = furthest['addr']&0xFF
			message[8] = fid     # id (for ack)
			message[10] = fid    # ping id
			message[11] = (closest['addr']&0xFF00)>>8
			message[12] = closest['addr']&0xFF
			message[14] = checksum(message[3:])
			self.AddPing(message, furthest['addr'], fid)

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

	def AssignAddress(self, fid, node):
		message = bytearray(b'\x7e\x00\x0B\x01\xee\xee\xee\x00\xee\x28\xee\xee\xee\xee\x00')
		message[4] = fid
		message[5] = (node['addr']&0xFF00)>>8
		message[6] = node['addr']&0xFF
		message[8] = fid
		message[10] = (node['raddr']&0xFF00)>>8
		message[11] = node['raddr']&0xFF
		message[12] = (node['faddr']&0xFF00)>>8
		message[13] = node['faddr']&0xFF
		message[14] = checksum(message[3:])
		self.buffout(message, addr, fid)
		# print(hexformat(message))

	def DeployMsg(self, fid, node):
		message = bytearray(b'\x7e\x00\x07\x01\xee\xee\xee\x00\xee\x30\x00')
		message[4] = fid
		message[5] = (node['addr']&0xFF00)>>8
		message[6] = node['addr']&0xFF
		message[8] = fid
		message[10] = checksum(message[3:])
		self.buffout(message, addr, fid)
		# print(hexformat(message))

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
					# print(hexformat(self.rxbuffer[place:end]))
				oldend = end+1
			elif not found:
				oldend = place + 1
		if oldend > 0:
			self.rxbuffer = self.rxbuffer[oldend:]