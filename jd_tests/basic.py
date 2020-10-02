from busio import JACDAC
import time
import board

j = JACDAC(board.P12)

b = bytearray("ABCDEFGHIJK")

while True:
    print('Hello World!')
    j.send(b)
    time.sleep(0.5)