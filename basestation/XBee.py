from Xhelp import *
import serial
from threading import Thread, Event
from datetime import datetime
from time import sleep, time

class XBee(Thread):
	nodes = []
	pings = []
	outmsgs = []
	pingsuccess = [0]
	addr = 0
	tick = 1
	frameid = 0
	rxbuffer = bytearray()
	stop = Event()
	currdeploy = None
	logdata = []
	lostchain = False

	startup, listen, processing = range(0,3) # States

	def __init__(self, serialport):
		Thread.__init__(self)
		self.serial = serial.Serial(port=serialport, baudrate=57600, timeout=0, rtscts=True)
		self.serial.setRTS(True)
		self.starttime = time()
		self.start()

	def shutdown(self):
		self.stop.set()
		self.join()
		self.writelog()

	def log(self, message, verbose):
		if message.find("run time") != -1:
			out = message + "\n"
		out = datetime.utcnow().strftime("%H-%M-%S: ") + message + "\n"
		if message.find("Rx") != -1:
			out = "\n" + out
		if message.find("success rate") != -1:
			message = "\n" + message
			out = "\n" + out
		if verbose:
			print(message)
		self.logdata.append(out)

	def writelog(self):
		with open(datetime.utcnow().strftime("testrun_%Y-%m-%dT%H-%M-%S.log"), 'w') as f:
			f.writelines(self.logdata)

	def OutDebug(self):
		now = time()
		if len(self.nodes):
			self.log("success rate: {:.2f}".format(100-(100*(sum(self.pingsuccess) / len(self.pingsuccess)))), True)
		for node in self.nodes:
			if node['deployed']:
				self.log("Node:{} RSSI:{:.0f} NRSSI:{:.0f} age:{}".format(node['addr'], sum(node['rssi']) / len(node['rssi']), sum(node['nrssi']) / len(node['nrssi']), 'y' if (now-node['time']) > 1.5 else 'n'), True)
				if node['faddr'] != 0xFFFF:
					self.log("\tFront:{}  RSSI:{} NRSSI:{} age:{}".format(node['faddr'], node['frssi'], node['fnrssi'],  'y' if (now-node['nt']) > 3 else 'n'), True)
				self.log("\tRear:{}   RSSI:{} NRSSI:{} age:{}".format(node['raddr'], node['rrssi'], node['rnrssi'],  'y' if (now-node['nt']) > 3 else 'n'), True)
		if len(self.nodes):
			self.log("run time:{:.2f}".format(now-self.starttime), True)


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
		self.log("start", True)		
		while not self.stop.is_set():
			sleep(0.005) # 1ms
			self.Rx()
			
			if (((time() - self.starttime) > 10) and (self.state == self.listen)):
				self.log("stop listening, start deploying", True)
				self.state = self.processing

			if self.state == self.processing:
				if (self.tick % 18) == 0:  #.1s
					self.msgaudit()
					self.pingaudit()
					self.CheckOnChain()
			
				if (self.tick % 91) == 0:  #.4s
					self.PingNodes()

				if (self.tick % 147) == 0: #.8s
					self.NeighborRSSResponse()

				if (self.tick % 182) == 0: #1s
					self.evalDeploy()

			if (((self.tick) % 182) == 0) and (self.state == self.startup):
				self.MY()

			if (self.tick % 182) == 0: #1s
				self.OutDebug()
				self.tick = 1

			self.tick = self.tick+1
		self.serial.close()

	def Rx(self):	
		for i in range(self.serial.inWaiting()):
			self.rxbuffer += self.serial.read()
		if len(self.rxbuffer):
			self.parseRx()

	def parseXBee(self, message):
		rxtype = message[0]
		if rxtype == 0x81: # received message
			naddr = (message[1]<<8) + message[2]
			rssi = message[3]
			# options = message[4]
			msgid = message[5]
			appid = message[6]

			self.updatenodeinfo(naddr, rssi)

			if msgid != 0:
				self.ACK(naddr, msgid)
			
			if appid == 0x12:
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
				self.updatenodeneighborinfo(srcaddr, faddr, frssi, fnrssi, raddr, rrssi, rnrssi)
			
			elif appid == 0x26:
				pid = message[7]
				# nodeaddr = (message[8]<<8) + message[9]
				self.pingmark(pid)
			
			elif appid == 0x27:
				appmsgid = message[7]
				self.DeployAck(appmsgid)
				self.msgmark(appmsgid)

			elif appid == 0x31:
				if self.lostchain:
					# looks like the node in front of us went down
					self.RemoveLost(naddr, 0x00)
					self.LostAck(naddr)
		
		elif rxtype == 0x89: # transmit status
			frameid = message[1]
			status  = message[2]
			if status == 0:
			# 	self.msgmark(frameid)
				return
			if (status & 3):
				self.msgretry(frameid)
		
		elif rxtype == 0x88: # AT command response
			frameid = message[1]
			cmd = ((message[2]<<8) + message[3])
			status = message[4]
			if cmd == ((0x4D<<8) + 0x59):
				self.log("listening", True)
				self.state = self.listen
				self.starttime = time()
				self.addr = ((message[5]<<8) + message[6])

	def CheckOnChain(self):
		if self.lostchain is True:return
		node = self.getClosestDeployed()
		if node is not None:
			if ((time() - node['time']) > 5):
				self.log("lost link to first node in chain: {}".format(node['addr']), True)
				self.lostchain = True

		# remove any undeployed nodes that we haven't heard from
		# edge case, but couldn't hurt
		lostnodes = []
		for node in self.nodes:
			if node['deployed'] is False:
				if ((time() - node['time']) > 5):
					lostnodes.append(nodes)

		for node in lostnodes:
			self.log("removing silent node: {}".format(node['addr']))
			self.nodes.remove(node)


	def getClosestDeployed(self):
		rnode = None
		for node in self.nodes:
			if not node['deployed']:
				break
			else:
				rnode = node
		if rnode is not None:
			self.log("closest deployed:{}".format(rnode['addr']), False)
		return rnode

	def getNextnonDeployed(self):
		for node in self.nodes:
			if not node['deployed']:
				self.log("non-deployed:{}".format(node['addr']), False)
				return node
		return None

	def CheckNodeThreshold(self, node):
		nrssi = sum(node['rssi']) / len(node['rssi'])
		nerssi = sum(node['nrssi']) / len(node['nrssi'])
		if nrssi > nerssi:
			rss = nerssi
		else:
			rss = nrssi
		self.log("check threshold:{} rss:{}".format(node['addr'], rss), False)
		if rss > 30:
			return True
		return False

	def evalDeploy(self):
		if self.lostchain: return # we've got a situation here
		if self.currdeploy is not None:
			self.doNextDeploymentStep()
			return
		node = self.getClosestDeployed()
		if node is not None:
			if node is self.nodes[-1]:return # no more nodes to deploy
			if self.CheckNodeThreshold(node):
				next = self.getNextnonDeployed() # deploy the next one
				if next is not None:
					self.Deploy(next)
			return
		node = self.getNextnonDeployed()
		if node is not None:
			self.Deploy(node) # deploy the first

	def Deploy(self, node):
		steplist = []

		self.log("deploy:{}".format(node['addr']), True)
		front = self.getnode(node['faddr'])
		if front is not None:
			front['raddr'] = node['addr']
			front['rrssi'] = 0x00
			front['rnrssi'] = 0x00
			steplist.append({'name':'assign', 'fid':self.id(), 'node':front})

		steplist.append({'name':'assign', 'fid':self.id(), 'node':node})
		steplist.append({'name':'deploy', 'fid':self.id(), 'node':node})
		
		self.currdeploy = {'node':node, 'start':time(), 'steps':steplist}
		self.doNextDeploymentStep()

	def doNextDeploymentStep(self):
		if self.currdeploy is None:return
		if (time() - self.currdeploy['start']) > 5:
			self.currdeploy = None
			self.log("deployment timeout", True)
			return
		if len(self.currdeploy['steps']) == 0:
			self.log("steps are out?", True)
			self.currdeploy = None
			return
		nextstep = self.currdeploy['steps'][0]
		self.log("next step: " + nextstep['name'], False)
		if nextstep['name'] == 'assign':
			self.AssignAddress(nextstep['fid'], nextstep['node'])
		if nextstep['name'] == 'deploy':
			self.DeployMsg(nextstep['fid'], nextstep['node'])

	def DeployAck(self, frameid):
		if self.currdeploy is None:return
		for step in self.currdeploy['steps']:
			if frameid == step['fid']:
				self.log("step success:{} {}".format(step['name'], step['fid']), False)
				self.currdeploy['steps'].remove(step)
		if len(self.currdeploy['steps']) != 0:
			self.doNextDeploymentStep()
		else:
			self.log("deployment successful", True)
			fnode = self.getnode(self.currdeploy['node']['faddr'])
			if fnode is not None:
				fnode['rssi'] = [0x00]
				fnode['nrssi'] = [0x00]
			self.currdeploy['node']['deployed'] = True
			self.currdeploy = None


	def getnode(self, addr):
		for node in self.nodes:
			if node['addr'] == addr:
				return node
		return None

	def AddNode(self, addr, rssi, nrssi=0x00):
		node = {'addr':addr ,'rssi':[rssi] ,'nrssi':[rssi], 'time':time(), 'deployed':False, 'nt':time(),
			'raddr':0x0000, 'rrssi':0x00, 'rnrssi':0x00, 
			'faddr':0xFFFF, 'frssi':0x00, 'fnrssi':0x00}
		self.log("add node:{}".format(node['addr']), True)
		if len(self.nodes)>0:
			node['faddr'] = self.nodes[-1]['addr']
		self.nodes.append(node)
		return node

	def updatenodeinfo(self, naddr, rssi, nrssi=0x00):
		node = self.getnode(naddr)
		if node is None:
			node = self.AddNode(naddr, rssi, nrssi)
		else:
			self.log("update node: {} {} {}".format(naddr, rssi, nrssi), False)
			self.log("       node: {} {} {}".format(node['addr'], node['rssi'], node['nrssi']), False)
			node['rssi'].append(rssi)
			if len(node['rssi']) > 10:
				node['rssi'].remove(node['rssi'][0])
			node['time'] = time()
			if nrssi != 0x00:
				node['nrssi'].append(nrssi)
				if len(node['nrssi']) > 10:
					node['nrssi'].remove(node['nrssi'][0])
			if (node is self.getClosestDeployed()) and (self.lostchain):
				 # we heard back from our closest guy, long time no see
				self.lostchain = False


	def updatenodeneighborinfo(self, naddr, faddr, frssi, fnrssi, raddr, rrssi, rnrssi):
		node = self.getnode(naddr)
		if node is None:
			self.log("I hope this goes away...", True)
			# we haven't deployed this node.  We should have heard a broadcast first
			# and added it, ignore this scenario.
			return

		self.log("update ne: {}  front:{} rs:{} ns:{} rear:{} rs:{} ns:{}".format(naddr, faddr, frssi, fnrssi, raddr, rrssi, rnrssi), False)
		self.log("       ne: {}  front:{} rs:{} ns:{} rear:{} rs:{} ns:{}".format(node['addr'], node['faddr'], node['frssi'], node['fnrssi'], node['raddr'], node['rrssi'], node['rnrssi']), False)

		# if someone produced an erroneous one of these it could fuck our shit up
		if (node['faddr'] != faddr):
			self.RemoveLost(faddr, naddr)

		if (node['raddr'] != raddr):
			if (raddr != 0xFFFF): # improbable, but would suck
				self.RemoveLost(naddr, raddr)

		node['frssi'] = frssi
		node['fnrssi'] = fnrssi
		node['rrssi'] = rrssi
		node['rnrssi'] = rnrssi
		node['nt'] = time()

	def RemoveLost(self, front, rear):
		# It seems we've lost one or more nodes, we'll need
		# to purge our list to match our new node situation
		# and pray to our god that this message isn't false
		lostnodes = []
		fnode = None
		rnode = None
		start = False

		self.log("remove lost:{}:{}".format(front, rear), True)

		if front == 0xFFFF:
			start = True

		for node in self.nodes:
			if node['addr'] == front:
				fnode = node
				start = True
				continue
			if node['addr'] == rear:
				rnode = node
				break
			if start:
				lostnodes.append(node)

		for node in lostnodes:
			self.log("removing:{}".format(node['addr']), False)
			self.nodes.remove(node)

		if fnode is not None:
			fnode['raddr'] = rear
			fnode['rrssi'] = 0x00
			fnode['rnrssi'] = 0x00

		if rnode is not None:
			rnode['faddr'] = front
			rnode['frssi'] = 0x00
			rnode['fnrssi'] = 0x00

		postop = ""
		for node in self.nodes:
			postop = postop + "node: {} front:{} rear:{}\n".format(node['addr'], node['faddr'], node['raddr'])
		self.log("post op: " + postop, False)

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
			if stat == 1:
				self.log("ping    fail: adr:{:02x} t:{:02.2f} id:{:02x}".format(node['addr'], time() - packet['sent'], packet['id']), False)
			if stat == 0:
				self.log("ping success: adr:{:02x} t:{:02.2f} id:{:02x}".format(node['addr'], time() - packet['sent'], packet['id']), False)
			if len(self.pingsuccess) > 25:
				self.pingsuccess.remove(self.pingsuccess[0])

	# def msgsucceed(self, msg, stat):
		# if stat == 1:
		# 	message = "msg     fail: adr:{:02x} t:{:02.2f} id:{:02x} r:{}".format(msg['addr'], time() - msg['sent'], msg['id'], msg['retries'])
		# if stat == 0:
		# 	message = "msg  success: adr:{:02x} t:{:02.2f} id:{:02x} r:{}".format(msg['addr'], time() - msg['sent'], msg['id'], msg['retries'])

	def msgmark(self, frameid):
		self.log("ack: {}".format(frameid), False)
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
					self.log("too many retries time:{}".format(time() - msg['sent']), False)
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
		message = bytearray(b'\x7e\x00\x08\x01\x00\xee\xee\x00\x00\x27\xee\x00')
		message[5] = (addr&0xFF00)>>8
		message[6] = addr&0xFF
		message[10] = fid # id (for ack)
		message[11] = checksum(message[3:])
		self.serial.write(message)

	def LostAck(self, addr):
		fid = self.id()
		message = bytearray(b'\x7e\x00\x07\x01\xFF\xFF\xFF\x00\xFF\x32\x00')
		message[4] = fid
		message[5] = (addr&0xFF00)>>8
		message[6] = addr&0xFF
		message[8] = fid
		message[10] = checksum(message[3:])
		self.buffout(message, addr, fid, True)

	def NeighborRSSResponse(self):
		first = self.getClosestDeployed()
		if first is not None:
			message = bytearray(b'\x7e\x00\x08\x01\x00\xee\xee\x00\x00\x12\xee\x00')
			message[5] = (first['addr']&0xFF00)>>8
			message[6] = first['addr']&0xFF
			message[10] = first['rssi'][-1]
			message[11] = checksum(message[3:])
			self.serial.write(escape(message))

	def PingNodes(self):
		if len(self.nodes) == 0:return
		closest = self.getClosestDeployed()
		furthest = self.nodes[0]
		if furthest['deployed'] and closest['deployed']:
			fid = self.id()
			message = bytearray(b'\x7e\x00\x0B\x01\xee\xee\xee\x00\xee\x24\xee\xee\xee\x00\x00')
			message[4] = fid # frame id
			message[5] = (closest['addr']&0xFF00)>>8
			message[6] = closest['addr']&0xFF
			message[8] = fid     # id (for ack)
			message[10] = fid    # ping id
			message[11] = (furthest['addr']&0xFF00)>>8
			message[12] = furthest['addr']&0xFF
			message[14] = checksum(message[3:])
			self.log("Send Ping; closest:{}  furthest:{} id:{}".format(closest['addr'], furthest['addr'], fid), False)
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
		self.log("Assign node:{} front:{} rear:{}".format(node['addr'], node['faddr'], node['raddr']), True)
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
		self.buffout(message, node['addr'], fid)

	def DeployMsg(self, fid, node):
		self.log("deploy message: {}".format(node['addr']), False)
		message = bytearray(b'\x7e\x00\x07\x01\xee\xee\xee\x00\xee\x30\x00')
		message[4] = fid
		message[5] = (node['addr']&0xFF00)>>8
		message[6] = node['addr']&0xFF
		message[8] = fid
		message[10] = checksum(message[3:])
		self.buffout(message, node['addr'], fid)

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
					self.log("Rx: " + hexformat(self.rxbuffer[place:end]), False)
					self.parseXBee(msg)
				oldend = end+1
			elif not found:
				oldend = place + 1
		if oldend > 0:
			self.rxbuffer = self.rxbuffer[oldend:]