import sys
print(sys.path)
sys.path.append('/home/pi/RobatROSPackage/src/drivers')

from time import sleep
from radio import Radio
from Packet import Packet
def main():
    if not radio.ping_radio():
        print("Radio self-test failed!")
        return
    else:
        print("Radio self-test passed!")

    radio = Radio('/dev/ttyAMA0', 7, 11, 12)
    i = 1
    while True:
        p = Packet(i, b'It was the best of times, it was the worst of times')
        radio.transmit(p.get_bytes())
        print(i)
        sleep(1)


if __name__ == "__main__":
    main()