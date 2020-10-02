from busio import JACDAC
import time
import board
import struct

display = board.DISPLAY
jd = JACDAC(board.P12)
rx_buff = bytearray(512)

p = None

while True:
    p = jd.receive(rx_buff)

    while p != 0:
        t = struct.unpack('Q',rx_buff[4:12])
        print(str(t))
        p = jd.receive(rx_buff)

    time.sleep(.1)