import sys
print(sys.path)
sys.path.append('/home/pi/RobatROSPackage/src/drivers')

from time import sleep
from radio import Radio
from Packet import Packet
def main():
    radio = Radio('/dev/serial0', 7, 11, 12)
    i = 0
    while True:
        p = Packet(i, b'It was the best of times, it was the worst of times')
        radio.transmit(p.get_bytes())
        print(i)
        sleep(1)
        i += 1


if __name__ == "__main__":
    main()