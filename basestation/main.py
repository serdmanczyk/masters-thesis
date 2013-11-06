from XBee import XBee
import traceback
from time import sleep

try: 
	XBee = XBee("COM3")
	while True:
		sleep(1)
		# if (len(msgs)):
		# 	XBee.FakeRx(msgs.pop(0))

except KeyboardInterrupt:
	pass
except Exception:
	print(traceback.format_exc())

XBee.shutdown()