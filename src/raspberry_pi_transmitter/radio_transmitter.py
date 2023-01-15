import sys
print(sys.path)
sys.path.append('/home/roboatpi/code/RobatROSPackage/src/drivers')

from time import sleep
from radio import Radio
from Packet import Packet
def main():
    radio = Radio('/dev/serial0', 7, 11, 12)
    while True:
        p = Packet(5000000, b'It was the best of times, it was the worst of times')
        radio.send(p.get_bytes())
        sleep(1)


if __name__ == "__main__":
    main()