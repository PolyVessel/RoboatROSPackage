# Written by Walden Hillegass 
# details of packet structure are in this document :
# https://docs.google.com/document/d/e/2PACX-1vRbm1WKCdh5X4Ji4nGYI22LL2Y-oLjkN6hBZEl6U7xsOcApdzz1ynh-aBq82YZGA1eBMbsIvnchxTQM/pub

import hashlib
from typing import List

MAX_PAYLOAD = 500
OVERHEAD = 12

class PacketizerException(Exception):
    pass

class Packet:
    def __init__(self, packet_id : int, payload : bytes):
        
        self.packet_id = packet_id
        self.payload = payload
    
    def get_bytes(self):
        if (len(self.payload) > MAX_PAYLOAD):
            raise PacketizerException("Payload is larger than max size. Use multi_packetizer() instead")
        id_bytes = self.packet_id.to_bytes(4, 'little')
        payload_length = len(self.payload).to_bytes(2, 'little')
        b = []
        b.append(b'\x69\x69')
        b.append(id_bytes)
        b.append(payload_length)
        b.append(hashlib.sha256(id_bytes + self.payload).digest()[:4])
        b.append(self.payload)

        self.bytes = b"".join(b)
        return self.bytes
    
    def __str__(self):
        return "Packet ID: " + str(self.packet_id) + " Payload: " + str(self.payload)
        

# Returns a list of packets
def multi_packetizer(payload : bytes, starting_packet_id : int) -> List[Packet]:
    packets = []
    packet_id = starting_packet_id
    while len(payload) > 0:
        if len(payload) > MAX_PAYLOAD:
            packets.append(Packet(packet_id, payload[:MAX_PAYLOAD]))
            payload = payload[MAX_PAYLOAD:]
        else:
            packets.append(Packet(packet_id, payload))
            payload = b""
        packet_id += 1
    return packets







        
if __name__ == "__main__":
    p = Packet(5000000, b'It was the best of times, it was the worst of times')
    print(packet_bytes(p.get_bytes()))
    print(p.get_bytes())
    print(de_packetizer(p.bytes))




