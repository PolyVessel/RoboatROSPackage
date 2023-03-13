# Written by Walden Hillegass 
# details of packet structure are in this document :
# https://docs.google.com/document/d/e/2PACX-1vRbm1WKCdh5X4Ji4nGYI22LL2Y-oLjkN6hBZEl6U7xsOcApdzz1ynh-aBq82YZGA1eBMbsIvnchxTQM/pub

import hashlib
from typing import List
import math

OVERHEAD = 17
MAX_PAYLOAD = 512 - OVERHEAD

class PacketizerException(Exception):
    pass

class Packet:
    def __init__(self, packet_id : int, payload : bytes, transmission_id : int = 0, transmission_count : int = 0):
        
        self.packet_id = packet_id
        self.transmission_id = transmission_id
        self.transmission_count = transmission_count
        self.payload = payload
    
    def get_bytes(self):
        if (len(self.payload) > MAX_PAYLOAD):
            raise PacketizerException("Payload is larger than max size. Use multi_packetizer() instead")
        id_bytes = self.packet_id.to_bytes(4, 'little', signed=False)
        transmission_id_bytes = self.transmission_id.to_bytes(4, 'little', signed=False)
        transmission_count_bytes = self.transmission_count.to_bytes(1, 'little', signed=False)
        payload_length = len(self.payload).to_bytes(2, 'little', signed=False)

        b = []
        b.append(b'\x83\x83')
        b.append(id_bytes)
        b.append(payload_length)
        b.append(transmission_id_bytes)
        b.append(transmission_count_bytes)
        thingtohash = id_bytes + transmission_id_bytes + transmission_count_bytes + self.payload
        print(thingtohash)
        b.append(hashlib.sha256(id_bytes + transmission_id_bytes + transmission_count_bytes + self.payload).digest()[:4])
        b.append(self.payload)

        self.bytes = b"".join(b)
        return self.bytes
    
    def __str__(self):
        return "Packet ID: " + str(self.packet_id) + " Payload: " + str(self.payload)
        

# Returns a list of packets
def multi_packetizer(payload : bytes, starting_packet_id : int) -> List[Packet]:
    transmission_count = math.ceil(len(payload) / MAX_PAYLOAD)
    output = []
    for i in range(transmission_count):
        output.append(Packet(starting_packet_id + i, payload[i*MAX_PAYLOAD:(i+1)*MAX_PAYLOAD], starting_packet_id, transmission_count))
    return output
        
if __name__ == "__main__":
    p = Packet(5000000, b'It was the best of times, it was the worst of times')
    print(packet_bytes(p.get_bytes()))
    print(p.get_bytes())
    print(de_packetizer(p.bytes))




