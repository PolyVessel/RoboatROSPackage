from time import sleep
from drivers.radio import Radio
from drivers.Packet import Packet
from drivers.Depacketizer import Depacketizer
def main():
    radio = Radio('/dev/ttyUSB0', 17, 27, 22)
    d = Depacketizer()
    while True:
        
        d.write(radio.receive())
        ps = d.read_packets_from_buffer()
        for p in ps:
            print(p.payload)
        sleep(1)


if __name__ == "__main__":
    main()