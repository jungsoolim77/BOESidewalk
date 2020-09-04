from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.start_preview()
#sleep(10)
#camera.stop_preview()

for i in range(5):
	sleep(2)
	camera.capture('/home/pi/sidewalk/images/image%s.jpg' % i)
camera.stop_preview()
