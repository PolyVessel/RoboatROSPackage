import sys
print(sys.path)
sys.path.append('/home/pi/RobatROSPackage/src/drivers')

from time import sleep
from radio import Radio
from Packet import Packet
def main():
    radio = Radio('/dev/serial1', 7, 11, 12)
    if not radio.ping_radio():
        print("Radio self-test failed!")
        return
    else:
        print("Radio self-test passed!")
    i = 1
    while True:
        p = Packet(i, b'It was the best of times, it was the worst of times')
        radio.transmit(p.get_bytes())
        print(i)
        sleep(1)


if __name__ == "__main__":
    main()