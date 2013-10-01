from XBee import XBee
import traceback
from time import sleep
from Xout import Xout
	
XBee = XBee("COM3")

try:
	while True:
		if XBee.Ready():
			XBee.OutDebug()
		sleep(0.5)
except KeyboardInterrupt:
	pass
except Exception:
	print(traceback.format_exc())

XBee.shutdown()