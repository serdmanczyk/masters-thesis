from Xhelp import *
from time import time
from datetime import datetime
import csv

class Xout():
	maxage = 0
	oentries = []

	def __init__(self, outfile):
		self.start = time()
		self.outfile = str(outfile)

	def printnodes(self, nodes):
		ages = []
		now = time()
		ages.append(self.maxage)
		for node in nodes:
			ages.append(time()-node['time'])
			print("Node:{} RSSI:{} NRSSI:{} age:{} success:%{:.0f} Neighbors:{}".format(node['addr'], node['rssi'], node['nrssi'], 'y' if (now-node['time']) > 1.5 else 'n', 100-((sum(node['pingsuccess'])/len(node['pingsuccess']))*100), len(node['neighbors'])))
			for neighbor in node['neighbors']:
				ages.append(time()-neighbor['time'])
				print("\tNeighbor:{} RSSI:{} NRSSI:{} age:{}".format(neighbor['addr'], neighbor['rssi'], neighbor['nrssi'],  'y' if (now-neighbor['time']) > 3 else 'n'))
		self.maxage = max(ages)
		# print("max age:{:.2f}".format(self.maxage))
		print("time:{:.2f}\n".format(now-self.start))

	def addoentry(self, nodes):
		now = time()
		dt = datetime.utcnow().strftime("%H-%M-%S.%f")
		for node in nodes:
			entry = {'time':dt, 
					 'relationship':"N-0:N-{}".format(node['addr']),
					 'age':now - node['time'],
					 'success':100-((sum(node['pingsuccess'])/len(node['pingsuccess']))*100), 
					 'rssi':node['rssi'],
					 'nrssi':node['nrssi']}
			self.oentries.append(entry)
			for neighbor in node['neighbors']:
				entry = {'time':dt, 
						 'relationship':"N-{}:N-{}".format(node['addr'], neighbor['addr']),
						 'age':now - neighbor['time'],
						 'success':' ', 
						 'rssi':neighbor['rssi'],
						 'nrssi':neighbor['nrssi']}
				self.oentries.append(entry)


	def export(self):
		# First lets get a list of all different types of data we have
		fieldnames = []
		for entry in self.oentries:
			keys = entry.keys()
			for key in keys:
				found = False
				for KEY in fieldnames:
					if key == KEY:
						found = True
				if not found:
					fieldnames.append(key)

		with open(self.outfile, 'w') as f:
			writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=',', restval=" ", lineterminator='\n')
			writer.writeheader()
			writer.writerows(self.oentries)
			print(str(self.oentries))
			f.close()
