from XBee import XBee
import traceback
from time import sleep
# from Xout import Xout
	
try:
	XBee = XBee("COM3")
	while True:
		sleep(0.5)
except KeyboardInterrupt:
	pass
except Exception:
	print(traceback.format_exc())

XBee.shutdown()