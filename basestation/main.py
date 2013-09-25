from XBee import XBee
import traceback
from time import sleep
from Xout import Xout
	
if __name__ == "__main__":
	Xo = Xout()
	XBee = XBee("COM3")

	try:
		while True:
			nodes = XBee.GetNodes()
			Xo.printnodes(nodes)
			Xo.addoentry(nodes)			
			sleep(0.5)
	except KeyboardInterrupt:
		pass
	except Exception:
		print(traceback.format_exc())
	
	XBee.shutdown()
	Xo.export()