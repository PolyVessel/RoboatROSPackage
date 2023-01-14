
from time import sleep
from drivers.radio import Radio
from drivers.Packet import Packet
def main():
    radio = Radio('/dev/ttyUSB0', 17, 27, 22)
    while True:
        p = Packet(5000000, b'It was the best of times, it was the worst of times')
        radio.send(p.get_bytes())
        sleep(1)


if __name__ == "__main__":
    main()