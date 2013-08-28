import serial
from xhelp import *
from XBee import XBee
import traceback
from time import sleep, time
from datetime import datetime
import logging

start = 0

def printnodes(nodes, maxage):
	ages = []
	now = time()
	ages.append(maxage)
	# print("\nreport")
	for node in nodes:
		ages.append(time()-node['time'])
		# print(node['failures'])
		print("Node:{} RSSI:{} NRSSI:{} age:{} success:%{:.0f} Neighbors:{}".format(node['addr'], node['rssi'], node['nrssi'], 'y' if (now-node['time']) > 1.5 else 'n', 100-((sum(node['failures'])/len(node['failures']))*100), len(node['neighbors'])))
		for neighbor in node['neighbors']:
			ages.append(time()-neighbor['time'])
			print("\tNeighbor:{} RSSI:{} NRSSI:{} age:{}".format(neighbor['addr'], neighbor['rssi'], neighbor['nrssi'],  'y' if (now-neighbor['time']) > 3 else 'n'))
	maxage = max(ages)
	# print("max age:{:.2f}".format(maxage))
	print("time:{:.2f}\n".format(now-start))
	return maxage

def loggit(nodes):
	now = time()
	nodefs = {'0':0}
	for node in nodes:
		nodefs[str(node['addr'])] = sum(node['failures'])/len(node['failures'])
	for node in nodes:
		logging.info("[{:.2f}],[{:.2f}],[0],[{}],[{}],[{}],[{}]".format(now-start, now-node['time'], node['rssi'], node['addr'], node['nrssi'], sum(node['failures'])/len(node['failures'])))
		for neighbor in node['neighbors']:
			logging.info("[{:.2f}],[{:.2f}],[{}],[{}],[{}],[{}],[{}]".format(now-start, now-neighbor['time'], node['addr'], neighbor['rssi'], neighbor['addr'], neighbor['nrssi'], nodefs[str(neighbor['addr'])]))
	
if __name__ == "__main__":
	maxage = 0.0
	start = time()
	# logname = "run-{}.log".format(datetime.utcnow().strftime("%Y%m%d-%H%M%S"))
	# logging.basicConfig(filename=logname, level=logging.DEBUG)
	# logging.info('timestamp,Age,Node,RSS,Neighbor,NRSS,failure')

	#XBee = XBee(serial.Serial(port="/dev/tty.usbserial-A1014JWM", baudrate=9600, timeout=0)) 
	XBee = XBee(serial.Serial(port="COM3", baudrate=57600, timeout=0, rtscts=True))

	# Continuously read and print packets
	try:
		while True:
			# maxage = printnodes(XBee.GetNodes(), maxage)
			# loggit(XBee.GetNodes()
			sleep(0.5)
	except KeyboardInterrupt:
		pass
	except Exception:
		print(traceback.format_exc())

	XBee.shutdown()
