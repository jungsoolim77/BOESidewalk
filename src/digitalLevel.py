import serial

def parseDigitalLevel(str):
    #if str.find('GGA') > 0:
    #    msg = pynmea2.parse(str)
    #    print "Timestamp: %s -- Lat: %s %s -- Lon: %s %s -- Altitude: %s %s" % (msg.timestamp,msg.lat,msg.lat_dir,msg.lon,msg.lon_dir,msg.altitude,msg.altitude_units)
    #s = str.replace("x", "")
    l = str.split()
    for x in l:
       if (len(x) < 20):
         print (x)
       else:
         print ("\n")
serialPort = serial.Serial("/dev/ttyS0", 9600, timeout=1)

while True:
    str = serialPort.read(9600)
    parseDigitalLevel(str)

serialPort.close()
