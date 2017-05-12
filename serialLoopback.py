import serial
import os
import time

ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM1"]
port = None
for p in ports:
    if os.path.exists(p):
        port = p
        break

if not port:
    raise RuntimeError("No COM port found!")

delay = 1
i = 1

ser = serial.Serial(port, 115200, timeout=1)
while True:
    rx = ser.readline()
    start = time.time()
    while(time.time() - start < delay / 1000.):
        time.sleep(0.001)
    print "   " + str((time.time() - start) * 1000)
    print "   " + str(ser.inWaiting())
    ser.flushInput()

    print rx.strip('\n')

    if len(rx) and rx[0] == 'm':
        ser.write(rx)
